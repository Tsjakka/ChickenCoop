/***************************************************************************
  This sketch automates my chicken coop. It closes and opens the door
  based on the time of day or on the outside temperature, sunrise and sunset.
  A DC motor is used to move the door up and down.
  Optionally, optical reflection sensors are used to control the up and down
  movement of the door.
  Various information, such as the status of the door, is presented on a web
  page. The position of the door can be calibrated using the web page.

  This sketch uses code from the following websites:
  https://randomnerdtutorials.com/esp8266-web-server/
  https://www.bananarobotics.com/shop/How-to-use-the-HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
  https://www.arduino.cc/en/Tutorial/UdpNTPClient
  https://github.com/sfrwmaker/sunMoon

  Written by Tsjakka from the Netherlands.
  BSD license, all text above must be included in any redistribution.
 ***************************************************************************/

#include <math.h>
#include <Time.h>
#include <TimeLib.h>
#include <sunMoon.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BME280.h>
#include <ESPMail.h>
#include <ESP_EEPROM.h>

// ************************************************************************
// Change the constants below to adjust the software to your situation
// ************************************************************************

const float Latitude = 0.0000000;           // Replace with your coordinates
const float Longitude = 0.000000;
int Timezone = 120;                         // UTC difference in minutes (including DST)

// At least one of the following booleans must be true.
const bool UseTemperature = false;          // Use the temperature for deciding when to open and close the door (otherwise it's time based)
const bool UseClock = true;                 // Use the clock for deciding when to open and close the door (otherwise it's temperature based)

// For reading the temperature.
const float ClosingTemperature = 0;         // Close the door at night when the temperature goes below this value
const time_t LoopPeriod = 180;              // Number of seconds between checks

// Open and closing times
uint8_t CloseBeforeSunriseMinutes = 120;    // The number of minutes before sunrise we close the coop
uint8_t HourOpen = 7;                       // The time the chickens may leave the coop
uint8_t MinuteOpen = 20;
uint8_t WeekendHourOpen = 7;                // The time the chickens may leave the coop in the weekend
uint8_t WeekendMinuteOpen = 20;
time_t closingTime = 0;                     // The time the door will close today
uint8_t HourCalibration = 1;                // Hour when to calibrate the position of the door
uint8_t MinuteCalibration = 0;              // Minute when to calibrate the position of the door

// For the Wifi connection. Replace with your network credentials
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";
const time_t connectPeriod = 90;            // Max period (in s) for setting up a wifi connection

// Variable to store the HTTP request
String header;

// NTP stuff
char* NtpServer = "time.windows.com";       // A reliable NTP server ("time.nist.gov" didn't work so well)
const unsigned int localPort = 2390;        // Local port to listen for UDP packets
const int NtpPacketSize = 48;               // NTP time stamp is in the first 48 bytes of the message
const time_t UpdateTimeTimeout = 300;       // Time (in seconds) we will try updating the clock

byte packetBuffer[NtpPacketSize];           // Buffer to hold incoming and outgoing packets
time_t updateTimeStarted;                   // The time when we started updating the clock
bool ntpTimeSet = false;                    // Has the time been set through NTP?
bool emailSent = false;                     // Has an email been sent because setting the time failed?
int lastSecond = -1;                        // The second we last did stuff for NTP
bool settingSunRiseSunSet = false;          // True when setting the sunrise and sunset

// Email
char* fromAddress = "REPLACE_WITH_YOUR_EMAIL";       // The address you want alarm messages sent from
char* toAddress = "REPLACE_WITH_YOUR_EMAIL";    // The email address you want alarm messages sent to
char* smtpServer = "REPLACE_WITH_YOUR_SERVER";     // The server to use for sending email
char* emailAccount = "REPLACE_WITH_YOUR_ACCOUNT";              // The email account on the server
char* emailPassword = "REPLACE_WITH_YOUR_PASSWORD"; // The password for the email account

// Wired connections
const int L9110_A_IA = 12;                  // Motor A Input A --> MOTOR A +
const int L9110_A_IB = 14;                  // Motor A Input B --> MOTOR A -
const int UpperSensorPin = 13;              // Digital output of upper TCRT5000 Tracking Sensor Module
const int LowerSensorPin = 16;              // Digital output of lower TCRT5000 Tracking Sensor Module

// The actual values for "fast" and "slow" depend on the motor
const int PwmSlow = 750;                    // Slow speed PWM duty cycle
const int PwmFast = 1023;                   // Fast speed PWM duty cycle
const int DirDelay = 1000;                  // Delay to prevent abrupt motor changes

// Stuff related to controlling the motor
long UpMoveMillis = 12900;                  // The default duration of an up move.
long DownMoveMillis = 12700;                // The default duration of a down move. When using a lower sensor this is used when the sensor fails
long CalibrationMoveMillis = 2500;          // The max. duration of the move used for calibrating the position of the door

bool Bme280Present = true;                  // Initialize/use the BME280 or not

// ************************************************************************
// Change the constants above to adjust the software to your situation
// ************************************************************************

enum Direction {
  DownDir,
  UpDir,
};

enum Speed {
  Stop,
  Slow,
  Fast
};

enum Command {
  DownCmd,
  UpCmd,
  StopCmd,
  Move1SecUpCmd,
  Move1SecDownCmd,
  NoCmd
};

// Stuff related to the state machine controlling the door
enum StateMachineState
{
  NotRunning = 0,
  Initial = 1,
  Up = 2,
  MovingIntoSensor = 3,
  ClearingSensor = 4,
  MovingDown = 5,
  Down = 6,
  MovingUp = 7,
  Moving1Sec = 8
};

// Set web server port number to 80
WiFiServer server(80);
WiFiClient client;
bool clientActive = false;                  // A web client is active
String currentLine = "";                    // A string to hold incoming data from the client
time_t clientConnectedAt;
const time_t webConnectPeriod = 30;         // Max timeout period (in s) for HTTP connections

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

Command webCommand = NoCmd;

// For sunrise and sunset
sunMoon sm;
time_t sunRise = 0;
time_t sunSet = 0;

enum StateMachineState stateMachineState = NotRunning;
enum StateMachineState previousStateMachineState = NotRunning;
enum StateMachineState prevState = NotRunning;                  // Used for returning to the previous state when moving 1 second
bool stateChanged = true;  // Start with entry code

bool CalibrateUsingSensor = true;           // Calibrate the door position using a sensor at the top position of the door
bool UseLowerSensor = false;                // Use a sensor to detect when the door is closed

bool lowerSensorDetected = false;
bool upperSensorDetected = false;
bool doorCalibrated = false;
bool calibrationFailed = false;
bool previousLowerSensorDetected = false;
bool previousUpperSensorDetected = false;
unsigned long startMovingMillis = 0;

// The class that controls the BME280 sensor
Adafruit_BME280 bme;
volatile time_t previousCheckAt = 0;
float temperature;
float humidity;                             // Informational
float pressure;                             // Informational

// The setup function that initializes everything
void setup() {
  Serial.begin(115200);

  // Initialize Wi-Fi. Force the ESP to reset Wi-Fi and initialize correctly.
  Serial.print("WiFi status = ");
  Serial.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("Wi-Fi status = ");
  Serial.println(WiFi.getMode());
  // End initialization

  // Connect to Wi-Fi network with SSID and password. Reboot if it continuously fails.
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);

  time_t firstCheckAt = now();
  while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (now() < firstCheckAt + connectPeriod)) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    // Print local IP address and start web server
    Serial.println("Wi-Fi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Print the SSID of the network you're attached to
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // Print the received signal strength
    long rssi = WiFi.RSSI();
    Serial.print("Signal strength (RSSI): ");
    Serial.print(rssi);
    Serial.println(" dBm");
  }
  else {
    WiFi.disconnect();
    Serial.println();
    Serial.println("No Wi-Fi connection established, rebooting");
    delay(5000);
    ESP.restart();
  }

  // Initialize motor
  pinMode(L9110_A_IB, OUTPUT);
  pinMode(L9110_A_IA, OUTPUT);
  digitalWrite(L9110_A_IB, LOW);
  digitalWrite(L9110_A_IA, LOW);

  // Sensors
  pinMode(UpperSensorPin, INPUT);
  pinMode(LowerSensorPin, INPUT);

  // When using the temperature, initialize the BME280
  if (Bme280Present) {
    // Check for the BME280 sensor
    Serial.println(F("Starting BME280 sensor"));
    if (bme.begin(0x76)) {
      // Scenario for weather monitoring
      Serial.println("Setting BME280 to weather station scenario:");
      Serial.println("  Forced mode, 1x temperature / 1x humidity / 1x pressure oversampling, filter off");
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1, // Temperature
        Adafruit_BME280::SAMPLING_X1, // Pressure
        Adafruit_BME280::SAMPLING_X1, // Humidity
        Adafruit_BME280::FILTER_OFF);
    }
    else {
      Serial.println(F("Could not find a valid BME280 sensor, please check wiring."));
      Bme280Present = false;
    }
  }

  // The begin() call is required to initialize the EEPROM library
  EEPROM.begin(32);

  // Read from EEPROM
  bool dataPresent = false;
  EEPROM.get(0, dataPresent);
  if (dataPresent) {
    EEPROM.get(1, Timezone);
    EEPROM.get(5, CloseBeforeSunriseMinutes);
    EEPROM.get(6, HourOpen);
    EEPROM.get(7, MinuteOpen);
    EEPROM.get(8, WeekendHourOpen);
    EEPROM.get(9, WeekendMinuteOpen);
    EEPROM.get(10, CalibrateUsingSensor);
    EEPROM.get(11, UseLowerSensor);
    EEPROM.get(12, UpMoveMillis);
    EEPROM.get(16, DownMoveMillis);
    EEPROM.get(20, CalibrationMoveMillis);
  }

  // For NTP
  Udp.begin(localPort);
  Serial.println("Started listening for UDP packets.");
  updateTimeStarted = now();

  // Start the webserver
  server.begin();
}

// Send an NTP request to the time server at the given address
int sendNtpPacket(const char* host) {
  int result = 0;

  // Set all bytes in the buffer to 0
  memset(packetBuffer, 0, NtpPacketSize);

  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode

  // All needed NTP fields have been given values, now
  // send a packet requesting a timestamp
  result = Udp.beginPacket(host, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NtpPacketSize);
  Udp.endPacket();

  return result;
}

time_t getNtpTime() {
  time_t result = 0;

  // Check if a reply is available
  if (Udp.parsePacket()) {
    Serial.println("Packet received");

    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NtpPacketSize); // Read the packet into the buffer

    // The timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

    // Combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;

    // Subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;

    Serial.print("UTC time is: ");
    printDateTime(epoch);
    Serial.println();

    result = epoch;
  }

  return result;
}

//void printDigits(int digits) {
//  // Utility function for digital clock display: leading 0
//  if (digits < 10) Serial.print('0');
//  Serial.print(digits);
//}
//
//void printDateTime(time_t dateTime, bool includeDate) {
//  if (includeDate) {
//    Serial.print(year(dateTime));
//    Serial.print("-");
//    printDigits(month(dateTime));
//    Serial.print("-");
//    printDigits(day(dateTime));
//    Serial.print(" ");
//  }
//  printDigits(hour(dateTime));
//  Serial.print(":");
//  printDigits(minute(dateTime));
//  Serial.print(":");
//  printDigits(second(dateTime));
//  Serial.print(" ");
//}

void printDateTime(time_t date) {
  char buff[20];
  sprintf(buff, "%2d-%02d-%4d %02d:%02d:%02d",
    day(date), month(date), year(date), hour(date), minute(date), second(date));
  Serial.print(buff);
}

void moveDoor(Speed speed, Direction direction) {
  // Always stop motor before running it to avoid abrupt changes
  digitalWrite(L9110_A_IB, LOW);
  digitalWrite(L9110_A_IA, LOW);
  delay(DirDelay);

  // Write new motor direction and speed
  if (speed == Fast && direction == DownDir) {
    Serial.println("Moving down fast...");
    analogWrite(L9110_A_IA, PwmFast); // PWM speed = fast
  }
  else if (speed == Slow && direction == DownDir) {
    Serial.println("Moving down slowly...");
    analogWrite(L9110_A_IA, PwmSlow); // PWM speed = slow
  }
  else if (speed == Fast && direction == UpDir) {
    Serial.println("Moving up fast...");
    analogWrite(L9110_A_IB, PwmFast); // PWM speed = fast
  }
  else if (speed == Slow && direction == UpDir) {
    Serial.println("Moving up slowly...");
    analogWrite(L9110_A_IB, PwmSlow); // PWM speed = slow
  }
  else if (speed == Stop) {
    Serial.println("Stop...");
    // Already done at top of function
  }
}

void handleWebClient() {
  char buf[20];

  if (!clientActive) {
    client = server.available();            // Listen for incoming clients
    if (client) {                           // If a new client connects
      clientConnectedAt = now();
      Serial.print("New client connection at ");
      printDateTime(clientConnectedAt);
      Serial.println();
      clientActive = true;
      currentLine = "";
    }
  }
  else {
    if (client.connected()) {               // Check if the client is still connected
      if (client.available()) {             // If there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // If the byte is a newline character
          // If the current line is blank, you got two newline characters in a row.
          // That's the end of the client HTTP request, so send a response
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Handle user input
            if (header.indexOf("GET /up1sec") >= 0) {
              Serial.println("Up for 1 second selected");
              webCommand = Move1SecUpCmd;
            }
            else if (header.indexOf("GET /down1sec") >= 0) {
              Serial.println("Down for 1 second selected");
              webCommand = Move1SecDownCmd;
            }
            else if (header.indexOf("GET /down") >= 0) {
              Serial.println("Down selected");
              webCommand = DownCmd;
            }
            else if (header.indexOf("GET /up") >= 0) {
              Serial.println("Up selected");
              webCommand = UpCmd;
            }
            else if (header.indexOf("GET /stop") >= 0) {
              Serial.println("Stop selected");
              webCommand = StopCmd;
            }
            else if (header.indexOf("GET /params") >= 0) {
              // Find first parameter
              int begin = header.indexOf('?');
              int end = header.indexOf('&');
              Serial.print("begin=");
              Serial.print(begin);
              Serial.print(" end=");
              Serial.println(end);
              if (begin > -1 && end > begin + 1) {
                String sub = header.substring(begin + 1, end);
                Serial.print("Substring: ");
                Serial.println(sub);

                // Split parameter in name and value. Repeat for all parameters
                bool dataPresent = true;
                bool invalidParam = false;
                do {
                  int equalsPos = sub.indexOf('=');
                  if (equalsPos > -1) {
                    String param = sub.substring(0, equalsPos);
                    String value = sub.substring(equalsPos + 1, sub.length());
                    Serial.print("Parsed ");
                    Serial.print(param);
                    Serial.print(" = ");
                    Serial.println(value);

                    // Check for and set the variables
                    long temp;
                    if (param.indexOf("Revert") == 0) {
                      temp = value.toInt();
                      if (temp == 0) dataPresent = false;
                    }
                    else if (param.indexOf("Timezone") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) Timezone = temp;
                    }
                    else if (param.indexOf("CloseBeforeSunriseMinutes") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) CloseBeforeSunriseMinutes = temp;
                    }
                    else if (param.indexOf("HourOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) HourOpen = temp;
                    }
                    else if (param.indexOf("MinuteOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) MinuteOpen = temp;
                    }
                    else if (param.indexOf("WeekendHourOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) WeekendHourOpen = temp;
                    }
                    else if (param.indexOf("WeekendMinuteOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) WeekendMinuteOpen = temp;
                    }
                    else if (param.indexOf("CalibrateUsingSensor") == 0) {
                      temp = value.toInt();
                      CalibrateUsingSensor = (temp == 1);
                    }
                    else if (param.indexOf("UseLowerSensor") == 0) {
                      temp = value.toInt();
                      UseLowerSensor = (temp == 1);
                    }
                    else if (param.indexOf("UpMoveMillis") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) UpMoveMillis = temp;
                    }
                    else if (param.indexOf("DownMoveMillis") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) DownMoveMillis = temp;
                    }
                    else if (param.indexOf("CalibrationMoveMillis") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) CalibrationMoveMillis = temp;
                    }
                    else {
                      invalidParam = true;
                    }
                  }
                  else {
                    invalidParam = true;
                  }

                  // Go to next parameter
                  begin = end + 1;
                  end = header.indexOf('&', begin);
                  if (end < 0 || end > header.indexOf(' ', begin)) {
                    end = header.indexOf(' ', begin);
                    if (end < 0) {
                      end = header.length();
                    }
                  }
                  Serial.print("begin=");
                  Serial.print(begin);
                  Serial.print(" end=");
                  Serial.println(end);

                  if (end > begin + 1) {
                    sub = header.substring(begin, end);
                    Serial.print("Substring: ");
                    Serial.println(sub);
                  }
                } while (!invalidParam && end > begin + 1);

                // Put new settings into EEPROM
                EEPROM.put(0, dataPresent);
                EEPROM.put(1, Timezone);
                EEPROM.put(5, CloseBeforeSunriseMinutes);
                EEPROM.put(6, HourOpen);
                EEPROM.put(7, MinuteOpen);
                EEPROM.put(8, WeekendHourOpen);
                EEPROM.put(9, WeekendMinuteOpen);
                EEPROM.put(10, CalibrateUsingSensor);
                EEPROM.put(11, UseLowerSensor);
                EEPROM.put(12, UpMoveMillis);
                EEPROM.put(16, DownMoveMillis);
                EEPROM.put(20, CalibrationMoveMillis);
  
                // Write the data to EEPROM
                bool ok = EEPROM.commit();
                Serial.println((ok) ? "Commit OK" : "Commit failed");
              }
            }

            time_t t_now = now();
            printDateTime(t_now);
            Serial.println();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");

            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP8266 Web Server</h1>");
            client.print("<p>Today date/time: ");
            sprintf(buf, "%2d-%02d-%4d %02d:%02d:%02d",
              day(t_now), month(t_now), year(t_now), hour(t_now), minute(t_now), second(t_now));
            client.print(buf);
            client.println("</p>");

            client.print("<p>Today's sunrise and sunset: ");
            sprintf(buf, "%2d-%02d-%4d %02d:%02d:%02d",
              day(sunRise), month(sunRise), year(sunRise), hour(sunRise), minute(sunRise), second(sunRise));
            client.print(buf);
            client.print(", ");
            sprintf(buf, "%2d-%02d-%4d %02d:%02d:%02d",
              day(sunSet), month(sunSet), year(sunSet), hour(sunSet), minute(sunSet), second(sunSet));
            client.print(buf);
            client.println("</p>");

            client.print("<p>Today's closing time: ");
            sprintf(buf, "%02d:%02d", hour(closingTime), minute(closingTime));
            client.print(buf);
            client.println("</p>");

            client.print("<p>Today's opening time: ");
            if (dayOfWeek(now()) == 1 || dayOfWeek(now()) == 7) {
              sprintf(buf, "%02d:%02d", WeekendHourOpen, WeekendMinuteOpen);
            }
            else {
              sprintf(buf, "%02d:%02d", HourOpen, MinuteOpen);
            }
            client.print(buf);
            client.println("</p>");

            if (Bme280Present) {
              client.print("<p>Current temperature, humidity and pressure: ");
              sprintf(buf, "%0.1f*C, ", temperature);
              client.print(buf);
              sprintf(buf, "%0.f%%, ", humidity);
              client.print(buf);
              sprintf(buf, "%0.1f hPa", pressure);
              client.print(buf);
              client.println("</p>");
            }

            // 
            client.print("<p>State: ");
            switch (stateMachineState) {
            case NotRunning:
              client.print("NotRunning</p>");
              break;
            case Initial:
              client.print("Initial</p>");
              break;
            case Up:
              client.print("Up</p>");
              client.println("<p><a href=\"/down\"><button class=\"button\">Down</button></a></p>");
              client.println("<p><a href=\"/up1sec\"><button class=\"button\">1 Second Up</button></a></p>");
              client.println("<p><a href=\"/down1sec\"><button class=\"button\">1 Second Down</button></a></p>");
              break;
            case MovingDown:
              client.print("MovingDown</p>");
              client.println("<p><a href=\"/stop\"><button class=\"button\">Stop</button></a></p>");
              break;
            case Down:
              client.print("Down</p>");
              client.println("<p><a href=\"/up\"><button class=\"button\">Up</button></a></p>");
              client.println("<p><a href=\"/up1sec\"><button class=\"button\">1 Second Up</button></a></p>");
              client.println("<p><a href=\"/down1sec\"><button class=\"button\">1 Second Down</button></a></p>");
              break;
            case MovingUp:
              client.print("MovingUp</p>");
              client.println("<p><a href=\"/stop\"><button class=\"button\">Stop</button></a></p>");
              break;
            case Moving1Sec:
              client.print("Moving1Sec</p>");
              break;
            default:
              client.print("Unknown</p>");
              break;
            }
            client.println("<p><a href=\"/\"><button class=\"button\">Refresh</button></a></p><br>");

            // Print the form for uploading settings
            int revert = 1;
            client.println("<form action=\"/params\">");
            client.println("Enter 0 to revert to default values after reset:<input type=\"text\" name=\"Revert\" value=\""); client.print(revert); client.print("\"><br>");
            client.println("Timezone:<input type=\"text\" name=\"Timezone\" value=\""); client.print(Timezone); client.print("\"><br>");
            client.println("CloseBeforeSunriseMinutes:<input type=\"text\" name=\"CloseBeforeSunriseMinutes\" value=\""); client.print(CloseBeforeSunriseMinutes); client.print("\"><br>");
            client.println("HourOpen:<input type=\"text\" name=\"HourOpen\" value=\""); client.print(HourOpen); client.print("\"><br>");
            client.println("MinuteOpen:<input type=\"text\" name=\"MinuteOpen\" value=\""); client.print(MinuteOpen); client.print("\"><br>");
            client.println("WeekendHourOpen:<input type=\"text\" name=\"WeekendHourOpen\" value=\""); client.print(WeekendHourOpen); client.print("\"><br>");
            client.println("WeekendMinuteOpen:<input type=\"text\" name=\"WeekendMinuteOpen\" value=\""); client.print(WeekendMinuteOpen); client.print("\"><br>");
            client.println("CalibrateUsingSensor:<input type=\"text\" name=\"CalibrateUsingSensor\" value=\""); client.print(CalibrateUsingSensor); client.print("\"><br>");
            client.println("UseLowerSensor:<input type=\"text\" name=\"UseLowerSensor\" value=\""); client.print(UseLowerSensor); client.print("\"><br>");
            client.println("UpMoveMillis:<input type=\"text\" name=\"UpMoveMillis\" value=\""); client.print(UpMoveMillis); client.print("\"><br>");
            client.println("DownMoveMillis:<input type=\"text\" name=\"DownMoveMillis\" value=\""); client.print(DownMoveMillis); client.print("\"><br>");
            client.println("CalibrationMoveMillis:<input type=\"text\" name=\"CalibrationMoveMillis\" value=\""); client.print(CalibrationMoveMillis); client.print("\"><br>");
            client.println("<input type=\"submit\" value=\"Submit\">");
            client.println("</form>");

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();

            // Clear the header variable
            header = "";

            // Close the connection
            client.stop();
            Serial.println("Client disconnected");
            Serial.println("");

            clientActive = false;
          }
          else { // If you got a newline, then clear currentLine
            currentLine = "";
          }
        }
        else if (c != '\r') {  // If you got anything else but a carriage return character,
          currentLine += c;    // add it to the end of the currentLine
        }
      }
      else {
        // The client is connected but no data is available. Time out if this takes too long.
        if (now() > clientConnectedAt + webConnectPeriod) {
          clientActive = false;
          Serial.println("Client connection timed out");
        }
      }
    }
    else {
      clientActive = false;
      Serial.println("Client no longer connected");
    }
  }
}

// Calculate the next sunrise and sunset and time for closing the door
void setSunriseSunsetClosingTime() {
  // Initialize sunMoon
  sm.init(Timezone, Latitude, Longitude);
  sunRise = sm.sunRise();
  sunSet = sm.sunSet();
  Serial.print("Today's sunrise and sunset: ");
  printDateTime(sunRise);
  Serial.print(", ");
  printDateTime(sunSet);
  Serial.println();

  // The time for closing the door is <CloseBeforeSunriseMinutes> before sunset 
  // or opening time, whichever comes first.
  tmElements_t openingTimeElements;
  if (dayOfWeek(now()) > 1 && dayOfWeek(now()) < 7) {
    openingTimeElements = { second(), MinuteOpen, HourOpen, weekday(), day(), month(), year() - 1970 };
  }
  else {
    openingTimeElements = { second(), WeekendMinuteOpen, WeekendHourOpen, weekday(), day(), month(), year() - 1970 };
  }
  time_t openingTime = makeTime(openingTimeElements);

  if (openingTime < sunRise) {
    Serial.println("The time for opening is before sunset.");
    closingTime = openingTime - (CloseBeforeSunriseMinutes * 60);
  }
  else {
    closingTime = sunRise - (CloseBeforeSunriseMinutes * 60);
  }

  Serial.print("Closing the door today at: ");
  printDateTime(closingTime);
  Serial.println();
}

bool detectSensors() {
  upperSensorDetected = !digitalRead(UpperSensorPin);
  lowerSensorDetected = !digitalRead(LowerSensorPin);

  if (upperSensorDetected != previousUpperSensorDetected) {
    Serial.print("Upper sensor ");
    if (upperSensorDetected) {
      Serial.println("triggered");
    }
    else {
      Serial.println("released");
    }
    previousUpperSensorDetected = upperSensorDetected;
  }
  if (lowerSensorDetected != previousLowerSensorDetected) {
    Serial.print("Lower sensor ");
    if (lowerSensorDetected) {
      Serial.println("triggered");
    }
    else {
      Serial.println("released");
    }
    previousLowerSensorDetected = lowerSensorDetected;
  }

  return true;
}

void sendEmail(int message) {
  ESPMail mail;
  mail.begin();
  mail.setSubject(fromAddress, "Problem with chicken door");
  mail.addTo(toAddress);

  if (message == 1) {
    mail.setBody("The clock could not be set.");
  }
  else if (message == 2) {
    mail.setBody("Calibration failed.");
  }

  if (mail.send(smtpServer, 587, emailAccount, emailPassword) == 0) {
    Serial.println("Mail sent OK");
  }
  else {
    Serial.println("Error when sending mail");
  }
}

void loop() {
  // Set the time when not set
  if (!ntpTimeSet) {
    // If not set, check for the time, but only for a few minutes.
    // In order not to do these checks every loop, proceed only once a second
    if ((now() < updateTimeStarted + UpdateTimeTimeout) && (lastSecond != second())) {
      // Send a packet every 30 seconds and check for an answer the other times
      if (second() % 30 == 8) {
        Serial.print("Requesting time from NTP server at ");
        printDateTime(now());
        Serial.println();

        // Send an NTP packet to a time server
        if (sendNtpPacket(NtpServer) == 0) {
          Serial.print(F("DNS lookup failed for "));
          Serial.println(NtpServer);
        }
      }
      else {
        // Check for received packets
        time_t epoch = getNtpTime();
        if (epoch > 0) {
          Serial.print("Setting the system time to UTC + ");
          Serial.print(Timezone);
          Serial.println(" minutes");
          setTime(epoch + Timezone * 60);
          ntpTimeSet = true;

          setSunriseSunsetClosingTime();

          // Time was set, start the state machine for the door
          stateMachineState = Initial;
          stateChanged = (stateMachineState != previousStateMachineState);
          previousStateMachineState = stateMachineState;
        }
      }
      lastSecond = second();
    }
    else if (now() > updateTimeStarted + 3600) {
      // After an hour, try it again.
      updateTimeStarted = now();
    }
    else if (now() > updateTimeStarted + UpdateTimeTimeout) {
      // No time packet was received, send a warning email once.
      // Chances are the internet connection is down anyway.
      if (!emailSent) {
        emailSent = true;
        sendEmail(1);
      }
    }
  }
  else {
    // Make sure the clock and sunrise/sunset are updated once a week on wednesdays afternoons
    if (ntpTimeSet && dayOfWeek(now()) == 4 && hour() == 12 && minute() == 5 && second() == 59) {
      Serial.print("Invalidating time at ");
      printDateTime(now());
      Serial.println();
      ntpTimeSet = false;
      updateTimeStarted = now();
      emailSent = false;

      // Stop running the state machine, make sure it only runs on a correct clock
      stateMachineState = NotRunning;
      stateChanged = (stateMachineState != previousStateMachineState);
      previousStateMachineState = stateMachineState;
    }
  }

  // Web server stuff. Only run in stable state, because the state can be changed by the user.
  if (!stateChanged) {
    handleWebClient();
  }

  // Sample the door sensors
  if (CalibrateUsingSensor || UseLowerSensor) {
    detectSensors();
  }

  time_t t_now = now();   // The number of seconds since Jan 1 1970

  // Every day, a little after midnight, calculate the next sunrise and sunset and time for closing the door
  if (!settingSunRiseSunSet && hour() == 0 && minute() == 5 && second() == 0) {
    settingSunRiseSunSet = true;
    Serial.println("Calculating sunrise and sunset at ");
    printDateTime(t_now);
    Serial.println();

    setSunriseSunsetClosingTime();
  }
  else if (settingSunRiseSunSet && second() != 0) {
    settingSunRiseSunSet = false;
  }

  // Update the temperature every LoopPeriod seconds
  if (Bme280Present && (t_now >= previousCheckAt + LoopPeriod)) {
    previousCheckAt = t_now;

    // Print timer interrupt count and time
    Serial.print("Reading BME280 at ");
    printDateTime(now());
    Serial.println();

    // Start measurement
    bme.takeForcedMeasurement();

    // Temperature
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    printf("Temperature: %0.1f*C, humidity: %0.f%%, pressure: %0.1f hPa\r\n", temperature, humidity, pressure);
  }

  //******************************************************************
  // The state machine for controlling the door of the chicken coop //
  //******************************************************************

  switch (stateMachineState)
  {
  //==============================================================
  // State 'Initial' 
  //==============================================================
  case Initial:

    // Actions on entry
    if (stateChanged) {
      Serial.println("State: Initial");
    }

    // Transitions
    if (!stateChanged) {
      // Because we move away from the upper sensor after moving up, 
      // only use the lower sensor to determine the position of the door
      if (UseLowerSensor && lowerSensorDetected) {
        stateMachineState = Down;
      }
      else {
        stateMachineState = Up;
      }
    }
    break;

  //==============================================================
  // State 'Up' 
  //==============================================================
  case Up:

    // Actions on entry
    if (stateChanged) {
      Serial.println("State: Up");
      moveDoor(Stop, DownDir);
    }

    // Continuous actions

    // Transitions
    if (!stateChanged) {
      // In the night, when there is no interference from the sun on the IR sensor, the
      // position of the door is calibrated by moving it up until the sensor is triggered.
      if (CalibrateUsingSensor && !calibrationFailed && !doorCalibrated &&
          hour() == HourCalibration && minute() == MinuteCalibration) {
        if (upperSensorDetected) {
          // Calibrate by moving down a little bit so the sensor isn't triggered
          stateMachineState = ClearingSensor;
        }
        else {
          // Move up until the sensor is triggered
          stateMachineState = MovingIntoSensor;
        }
        doorCalibrated = true;
      }

      // Close the door if the temperature goes below the treshold and it is night OR
      // when it is just before sunset and we want to keep the chicken inside a bit longer
      if ((UseTemperature && (temperature < ClosingTemperature) && (t_now > sunSet || t_now < sunRise)) ||
        (UseClock && (t_now >= closingTime) && (t_now < closingTime + 4)) ||
        webCommand == DownCmd)
      {
        webCommand = NoCmd;
        stateMachineState = MovingDown;
      }
      else if (webCommand == Move1SecUpCmd || webCommand == Move1SecDownCmd) {
        prevState = Up;
        stateMachineState = Moving1Sec;
      }
    }
    break;

  //==============================================================
  // State 'MovingIntoSensor'
  // This state if for calibrating the position of the door using
  // the upper sensor. The door will move up a bit to find the
  // sensor. If the sensor is not found, the calibration
  // procedure is aborted and not done again until a reboot.
  //==============================================================
  case MovingIntoSensor:

    // Actions on entry
    if (stateChanged) {
      Serial.println("State: MovingIntoSensor");
      startMovingMillis = millis();
      moveDoor(Slow, UpDir);
    }

    // Continuous actions

    // Transitions
    if (!stateChanged) {
      if (upperSensorDetected) {
        moveDoor(Stop, DownDir);
        stateMachineState = ClearingSensor;
      }
      else if (millis() - startMovingMillis > CalibrationMoveMillis) {
        Serial.println("Calibration timed out");
        stateMachineState = Up;
        calibrationFailed = true;
        sendEmail(2);
      }
    }
    break;
    
  //==============================================================
  // State 'ClearingSensor' 
  //==============================================================
  case ClearingSensor:

    // Actions on entry
    if (stateChanged) {
      Serial.println("State: ClearingSensor");
      startMovingMillis = millis();
      moveDoor(Slow, DownDir);
    }

    // Continuous actions

    // Transitions
    if (!stateChanged) {
      if (!upperSensorDetected) {
        stateMachineState = Up;
      }
      else if (millis() - startMovingMillis > CalibrationMoveMillis) {
        Serial.println("Calibration timed out");
        stateMachineState = Up;
        calibrationFailed = true;
        sendEmail(2);
      }
    }
    break;

  //==============================================================
  // State 'MovingDown' 
  //==============================================================
  case MovingDown:

    // Actions on entry
    if (stateChanged) {
      Serial.println("State: MovingDown");

      startMovingMillis = millis();
      moveDoor(Fast, DownDir);
    }

    // Continuous actions

    // Transitions
    if (!stateChanged) {
      if ((UseLowerSensor && lowerSensorDetected) ||
        (millis() - startMovingMillis > DownMoveMillis) ||
        (webCommand == StopCmd))
      {
        webCommand = NoCmd;
        stateMachineState = Down;
      }
    }
    break;

  //==============================================================
  // State 'Down' 
  //==============================================================
  case Down:

    // Actions on entry
    if (stateChanged) {
      Serial.println("State: Down");
      moveDoor(Stop, DownDir);
    }

    // Continuous actions

    // Transitions
    if (!stateChanged) {
      if ((UseTemperature && ((temperature > ClosingTemperature) || (t_now > sunRise && t_now < sunSet))) ||
          (UseClock && (dayOfWeek(now()) > 1) && (dayOfWeek(now()) < 7) && (hour() == HourOpen) && (minute() == MinuteOpen)) ||
          (UseClock && (dayOfWeek(now()) == 1 || dayOfWeek(now()) == 7) && (hour() == WeekendHourOpen) && (minute() == WeekendMinuteOpen)) ||
          (webCommand == UpCmd))
      {
        webCommand = NoCmd;
        stateMachineState = MovingUp;
      }
      else if (webCommand == Move1SecUpCmd || webCommand == Move1SecDownCmd) {
        prevState = Down;
        stateMachineState = Moving1Sec;
      }
    }
    break;

  //==============================================================
  // State 'MovingUp' 
  //==============================================================
  case MovingUp:

    // Actions on entry
    if (stateChanged) {
      Serial.println("State: MovingUp");
      startMovingMillis = millis();
      moveDoor(Fast, UpDir);
    }

    // Continuous actions

    // Transitions
    if (!stateChanged) {
      if ((millis() - startMovingMillis > UpMoveMillis) ||
          (webCommand == StopCmd))
      {
        webCommand = NoCmd;
        stateMachineState = Up;
        doorCalibrated = false;
      }
    }
    break;

  //==============================================================
  // State 'Moving1Sec' 
  //==============================================================
  case Moving1Sec:

    // Actions on entry
    if (stateChanged) {
      Serial.print("State: Moving1Sec ");
      if (webCommand == Move1SecUpCmd) {
        Serial.println("Up");
        moveDoor(Fast, UpDir);
      }
      else {
        Serial.println("Down");
        moveDoor(Fast, DownDir);
      }
      webCommand = NoCmd;
      startMovingMillis = millis();
    }

    // Continuous actions

    // Transitions
    if (!stateChanged) {
      if (millis() - startMovingMillis > 1000) {
        if (prevState == Up) {
          stateMachineState = Up;
        }
        else {
          stateMachineState = Down;
        }
      }
    }
    break;
  }

  // Update state variables
  stateChanged = (stateMachineState != previousStateMachineState);
  previousStateMachineState = stateMachineState;

  if (clientActive) {
    delay(1);
  }
  else {
    delay(10);
  }
}

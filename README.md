# ChickenDoor
An ESP8285 / Arduino project for controlling a motorized chicken coop door.

Maybe you are under the impression that only roosters, not hens, can make a lot of noise. You would be mistaken. Although roosters crow more loudly than hens cackle, the cackling of hens can sure wake you up, or your neighbors for that matter. This is a lesson we learned after we bought 6 Wyandotte chicks, a pretty and rather large breed of chickens. From the internet we learned that roosters will start crowing at 6 months of age. Ours started at 4 months. After a visit from one of our neighbors, I started contemplating making an automated door in our chicken coop. But the need for such a device went away when our two roosters ended up in our frying pan. After that, we had no noise problems with our four hens, until they started laying eggs. Again, one of our neighbors, another one this time, came to our door with noise complaints. The hens would get out of their coop at 5:15 am and start cackling loudly. As we sleep on the other side of the house, we weren't aware of this. I promised the neighbor I would keep them inside until at least 7:00 a.m. and he was happy with that. But now I had to make some haste with getting the automated door, since I don't want to go outside at 7:00 every day. On this web page you can read all about how I created the hard- and software for a motorized chicken coop door. Perhaps you have a similar reason as I had for needing an automated door. Or, you may want to prevent foxes or ferrets getting in the coop at night. Or you may want to protect your chickens from the cold. Regardless, the software is easy to adapt to handle a number of use cases. The hardware is cheap and, as it turned out, pretty reliable (in a trial of one).

# The software

I started out with putting my requirements on paper. In a nutshell:
* I want to keep the door open as long as possible for fresh air, but close it before the chickens want to go outside. In practice this means that the door has to go down some time before sunrise. (But not in winter, when they go outside before sunrise, we found out.)
* The door has to be controlled by moving up and down a fixed period.
* For improving reliability, the software must be able to handle sensors that detect the up and down position of the door.

Before I started writing code, I made a state machine diagram that captures the behavior of the software. The image below shows this diagram. A state machine diagram shows the various states the system can be in and the conditions that trigger a transition from one state to another. Having this diagram made it a lot easier for me to write the core functionality of the software. 

![The state machine](https://github.com/Tsjakka/ChickenCoop/blob/master/Photos/StateMachine.gif)

# The test hardware

For my first experiments, I used a DC motor I took from an old printer and a small motor driver. I stuck all the parts I thought I would need on a breadboard and started making the software. Here a photo of this experimental setup:

![Experimenting](https://github.com/Tsjakka/ChickenCoop/blob/master/Photos/IMG_20200118_175027487.jpg)

This worked very well for creating and testing the software. Another important element is the microcontroller that is used to control the system. I selected an ESP8285 for this task and it works very well. It is also extremely cheap, just a few euros on the well known Chinese websites. I used the Arduino development environment to write the software. It's not the best IDE, but it's free and does the job.

After I created an initial version of the software, I started testing it on the test setup shown above. This way I learned how the system behaved and where I could improve it. In this period I made a lot of changes to the software and it got better and better. For instance, I decided to remove the retry mechanism as shown in the state machine diagram, as it turned out to be difficult to predict if it would do good or harm when something goes wrong. I also started to add more features into the software, such as:

What I also included in the software was:
* A Wi-Fi client that connects the system to the internet.
* A web server serving up a page that allows me to move the door up and down.
* An NTP client that reads the current time from the internet.
* Code for reading the temperature from a Bosch BME280 weather sensor.

In the meantime I started working on the final hardware, looking for the parts that would make it a working system. Especially the choice of motor is important in this respect, as it is supposed to lift the door with ease. After I finished the hardware, I installed it in the chicken coop and again started testing the system. Now, I also had to calibrate the different constants in my software, such as the time it takes to move the door up or down. In the first iteration I just used these time values to move the door, it logically turned out that the time to move up must be a little higher that the time to move down because of gravity. However, it is almost impossible this way to ensure that the door moves to the same top position every day for a longer period of time. Because of several reasons, such as the temperature, the distance the door moves varies from day to day. Therefor, I had to adjust the position of the door almost every week. I decided to start using an optical sensor to detect the top of door and stop motion. I drilled a hole in the chicken coop just above the topmost position of the door and installed a sensor in it. But when I started testing it, it turned out to be very unreliable. More testing showed that this was due to (indirect) sunlight hitting the sensor. I couldn't get it to work and started looking for a different kind of sensor. Unfortunately, this also turned out to be a dead end. So, I started thinking again how I could use the optical sensor in a more reliable way and finally it hit me; I can use it to calibrate the top position of the door in the middle of the night! All in all, this led to the revised state machine shown below.

![The state machine](https://github.com/Tsjakka/ChickenCoop/blob/master/Photos/StateMachine_Final.gif)

# The hardware

Below you can see what the final door opener looks like. The motor I used for testing is absolutely too weak for lifting the actual door of the chicken coop, a triplex board of 300 x 200 x 4 mm. So I replaced it with a motor with a built-in transmission and really high torque that can be run on 12V. The system in use for more than a year now and it has never missed a beat.

![The door opener](https://github.com/Tsjakka/ChickenCoop/blob/master/Photos/IMG_20200528_203406210.jpg)

The full list of hardware is:
* Aslong JGB37-3530 12V DC motor with built-in transmission - 20RPM
* L298N Bipolar stepper motor and DC-motor driver
* ESP8285 Development Board
* 12-pin header for ESP8285 (2x)
* Bosch BME280 weather sensor (optional)
* 4-pin header for BME280 (optional)
* TCRT5000 Tracking Sensor Module (2x) (optional)
* 3-pin Screw Terminal Block Connector 2.54mm for TCRT5000 (4x) (optional)
* Kradex Enclosure 176x126x57mm - Grey - Z74J
* Universal Single Side PCB Board Glass Fiber 5cm*7cm
* Motor coupling - 5mm to 8mm
* A piece of threaded rod, 8mm
* Part of a broomstick
* USB Power supply (5V)
* Power supply for the motor (12V)
* Micro USB cable - 1.8m
* Various pieces of wire

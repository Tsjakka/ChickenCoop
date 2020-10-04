# ChickenCoop
An ESP8285 / Arduino project for controlling a motorized chicken coop door.

Maybe you are under the impression that only roosters, not hens, can make a lot of noise. You would be mistaken. Although roosters crow more loudly than hens cackle, the cackling of hens can sure wake you up, or your neighbors for that matter. This is a lesson we learned after we bought 6 Wyandotte chicks, a pretty and rather large breed of chickens. From the internet we learned that roosters will start crowing at 6 months of age. Ours started at 4 months. After a visit from one of our neighbors, I started contemplating making an automated door in our chicken coop. But the need for such a device went away when our two roosters ended up in our frying pan. After that, we had no noise problems with our four hens, until they started laying eggs. Again, one of our neighbors, another one this time, came to our door with noise complaints. The hens would get out of their coop at 5:15 am and start cackling loudly. As we sleep on the other side of the house, we weren't aware of this. I promised the neighbor I would keep them inside until at least 7:00 a.m. and he was happy with that. But now I had to make some haste with getting the automated door, since I don't want to go outside at 7:00 every day. On this web page you can read all about how I created the hard- and software for a motorized chicken coop door. Perhaps you have a similar reason as I had for needing an automated door. Or, you may want to prevent foxes or ferrets getting in the coop at night. Or you may want to protect your chickens from the cold. Regardless, the software is easy to adapt to handle a number of use cases. The hardware is cheap and, as it turned out, pretty reliable (in a trial of one).

# The software

I started out with putting my requirements on paper. In a nutshell:
* I want to keep the door open as long as possible for fresh air, but close it before the chickens want to go outside. In practice this means that the door has to go down some time before sunrise. (But not in winter, when they go outside before sunrise, we found out.)
* The door has to be controlled by moving up and down a fixed period.
* For improving reliability, the software must be able to handle sensors that detect the up and down position of the door.

Before I started writing code, I made a state machine diagram that captures the behavior of the software. The image below shows this diagram. A state machine diagram shows the various states the system can be in and the conditions that trigger a transition from one state to another. Having this diagram made it a lot easier for me to write the core functionality of the software. 

![The state machine](https://github.com/Tsjakka/ChickenCoop/blob/master/Photos/StateMachine.gif)

What I also included in the software was:
* A Wi-Fi client that connects the system to the internet.
* A web server serving up a page that allows me to move the door up and down.* An NTP client that reads the current time from the internet.* Code for reading the temperature from a Bosch BME280 weather sensor.

# The hardware

In this project, I used a DC motor to move the door up and down. I used a DC motor because I found one with a really high torque, which fits my use case very well. The second important element is a microcontroller to control the system. I selected an ESP8285 for this task and it works very well. It is also extremely cheap, just a few euros on the well known Chinese websites. I used the Arduino development environment to write the software. It's not the best IDE, but it's free and does the job.

The full list of hardware is:
* Aslong JGB37-3530 12V DC motor with built-in transmission - 20RPML298N Bipolar stepper motor and DC-motor driver
* ESP8285 Development Board12 pin header (2x) for ESP8285
* Bosch BME280 weather sensor (optional)
* 4 pin header for BME280 (optional)
* TCRT5000 Tracking Sensor Module 2x (optional)
* 3 Pin Screw Terminal Block Connector 2.54mm for TCRT5000 4x (optional)
* Kradex Enclosure 176x126x57mm - Grey - Z74J
* Universal Single Side PCB Board Glass Fiber 5cm*7cm
* Motor coupling - 5mm to 8mm
* A piece of threaded rod, 8mm
* Part of a broomstick
* USB Power supply
* Micro USB cable - 1.8m
* Various pieces of wire

At the time, I considered a stepper motor as well. It brings the advantage of not needing any sensors to detect the up- or down position of the door, since a fixed number of steps can be used to control the movement. The downside, however, is that you need a lot heavier power supply. As I started out with the DC motor, I decided to stay the course. However, it would be interesting to see a fork of this project in which a stepper motor is used.

![The door opener](https://github.com/Tsjakka/ChickenCoop/blob/master/Photos/IMG_20200528_203406210.jpg)

# SpotMicro AI 
This Project is work in progress and documenting my progress on the goal of making a four-legged robot walk.
It is NOT a working or even finished Project you might want to use! 

I started this Project because i got inspired by some very smart people/companies and projects out there and want to 
understand and adapt their work on my personal wish to create a small "clone" of a "BostonDynamics SpotMini"-kind-of-looking but self-learning Robot.

My Goal is to create a) a working physical Robot, b) a simulated Environment to be able to c) do RL training to make it walk.
There are other ways of achiving this, i think. The use of InverseKinematics only, combined with a robust ground detection
could also solve the problem and might be "more straight-forward" / yet already very challenging. 
The reason why i want to try a combination of both is because i believe that a well trained RL-Model could be move stable and
robust in different situations where the physical robot leaves controlled environments or parts like Servos or Legs become unstable or even break.

Nevertheless we will have to create a precise IK-Model to be able to have some kind of guided training. 

## a) The Robot

![SpotMicroAI](/Images/SpotMicroAI_1.jpg)

First of all thanks to Deok-yeon Kim aka KDY0523 who made this incredible work 
https://www.thingiverse.com/thing:3445283
This basically already is the physical Robot. 
Here is my Make https://www.thingiverse.com/make:654812

Since my setup required some additional Hardware, i recreated some parts using FreeCAD - see /Parts-Directory.

![Parts](/Images/SpotMicroAI_FreeCad.png)

The Controller-Firmware for the Arduino-Mega can be found in /Controller.

This Controller communicates with a NVIDIA Jetson Nano via UART. The Jetson Nano will serve as Locomotion Controller.
Since i am not a robotics company, i can't easily invest some hundreds of Euros/Dollars. So i will use Sonar Sensors instead of visual sensors like RGB or RGBD-Cams. Maybe i will try with ESPEyes or something in the near future.

Sensors used:
4 x HC-SR04-Sensors. 2x as in the original model in the front looking forward/down. 2x at the bottom (front/back) looking down to measure the ground-distance. 
An IMU MPU-6050 is used to measure pitch,roll and velocities. Yaw will be ignored since it drifts quickly. 

I also connected a SSD1306 128x64 OLED Display and a NeoMatrix LED Circle, but in my Tests it seems that the ArduinoMega is simply too slow to handle 12 Servos + Sonar + nice LED and OLED visual. The used ClockTicks prevent the Board from handling the PWD-Signals appropriately and the Servos start to "klick" on update-cycles or even shake when too much CPU-time has been used for custom code. This is why i decided to modify the Hardware-Setup and use the ArduinoMega as ServoController only. 

An additional NodeMCU or WemosD1 (not yet decided because of 3V/5V issues) will then be the OLED and LED(s) and Sonar-Controller.


## b) Simulation and c) Training

See https://arxiv.org/pdf/1804.10332.pdf by
Jie Tan, Tingnan Zhang, Erwin Coumans, Atil Iscen, Yunfei Bai, Danijar Hafner, Steven Bohez, and Vincent Vanhoucke
Google Brain,Google DeepMind

First of all a URDF needs to be created. The Model will be in URDF/
I will start with a simple and very approximatly model of the real Robot, before i go into Detail for every Part. This gives me some feeling of progress during all the work. So don't be sad to see sad cubes everywhere ;)

## Credits  
Deok-yeon Kim creator of SpotMicro, 
Boston Dynamics who built this incredible SpotMini,
Ivan Krasin - https://ivankrasin.com/about/ - thanks inspiration and chatting



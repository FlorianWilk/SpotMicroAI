# SpotMicro AI 
![urdf](/Images/SpotMicroAI_rviz_2.png)
## How to build a cheap four-legged Robot and make it learn how to walk

This Project is heavily work in progress and documenting my progress on the goal of making a four-legged robot walk.
It is NOT a working or even finished Project you might want to use! 

I started this Project because i got inspired by some very smart People/Companies and Projects out there and want to 
understand and adapt their work on my personal wish to create a small "clone" of a "BostonDynamics SpotMini"-kind-of-looking but self-learning Robot.

My Goal is to create a) a working physical Robot with cheap components everyone can build, b) a simulated Environment to be able to c) do RL training to make it walk.

There are other ways of achiving this, i think. The use of InverseKinematics only, combined with a robust ground detection could also solve the problem and might be "more straight-forward" / yet already very challenging. 

I want to try a combination of both, because i believe that a well trained RL-Model could be move stable and
robust in different situations where the physical robot leaves controlled environments or parts like Servos or Legs become unstable or may even break. 

Nevertheless we will have to create a precise IK-Model to be able to have some kind of guided training. 

## a) The Robot

![SpotMicroAI](/Images/SpotMicroAI_1.jpg)

First of all thanks to Deok-yeon Kim aka KDY0523 who made [this incredible work](https://www.thingiverse.com/thing:3445283)
This basically is the physical Robot. It will take some Days to print and assemble all the Parts, but it's worth all the effort. I also sanded, primed and painted all the Parts to give it a nicer Look.

[Here is my Make](https://www.thingiverse.com/make:654812)

Since my setup required some additional Hardware, i recreated some parts using FreeCAD - see /Parts-Directory.
![Parts](/Images/SpotMicroAI_FreeCad.png)

I will use a NVIDIA Jetson Nano as Locomotion-Controller, which will be connected to the ArduinoMega via UART. 

![JetsonNano](/Images/jetsonNano.jpg)

To have some Protection for the NVIDIA, i printed [this Case](https://www.thingiverse.com/thing:3603594).

![JetsonNano-Case](/Images/jetsonNanoCase.jpg)

The Controller-Firmware for the Arduino-Mega can be found in /Controller. It will receive and send Commands and Information via UART(Serial) to our Locomotion-Controller.

### Sensors

Since i am not a robotics company, i can't easily invest some hundreds of Euros/Dollars. So i will use Sonar Sensors instead of visual sensors like RGB or RGBD-Cams. Maybe i will try with ESPEyes or something in the near future.

Sensors used:
4 x HC-SR04-Sensors. 2x as in the original model in the front looking forward/down. 2x at the bottom (front/back) looking down to measure the ground-distance. 
An IMU MPU-6050 is used to measure pitch,roll and velocities. Yaw will be ignored since it drifts quickly. 

I also connected a SSD1306 128x64 OLED Display and a NeoMatrix LED Circle, but in my Tests it seems that the ArduinoMega is simply too slow to handle 12 Servos + Sonar + nice LED and OLED visual. The used ClockTicks prevent the Board from handling the PWM-Signals properly and the Servos start to "klick" on Update-Cycles or even shake when too much CPU-Time has been used for Custom Code. This is why I decided to modify the Hardware-Setup and use the ArduinoMega as ServoController only. 

An additional NodeMCU or WemosD1 (not yet decided because of 3V/5V issues) will then be the OLED and LED(s) and Sonar-Controller.

I am not sure if the Hardware i use here will be enough to finally have a very smooth walking Robot like for Example the real SpotMini. See this more as a Research-Project where I try to use cheap Hardware and other People's Work to learn more about how this all works. 

UPDATE: The Arduino simply takes too much space when acting as Servo-Controller only. I think i will completely replace the Arduino with the Jetson Nano + 16 Channel PCA9685 I2C-Servo Driver.


## b) Simulation and c) Training

![urdf](/Images/SpotMicroAI_urdf2.png)

I will try to implement [this Paper](https://arxiv.org/pdf/1804.10332.pdf) by
Jie Tan, Tingnan Zhang, Erwin Coumans, Atil Iscen, Yunfei Bai, Danijar Hafner, Steven Bohez, and Vincent Vanhoucke
Google Brain,Google DeepMind

![PyBullet](/Images/SpotMicroAI_pybullet_2.png)

The URDF Model is very basic and work in progress. Masses and Inertias are guesses and not correct. I will have to disassemble the Robot to have correct weights. And i am still exploring PyBullet, so a lot of code is still Try-Outs.

The Simulation-Implementation in PyBullet is still a bunch of Copy&Pastes from Tutorials. 

## Credits and thanks
Deok-yeon Kim creator of SpotMicro, 
Boston Dynamics who built this incredible SpotMini,
Ivan Krasin - https://ivankrasin.com/about/ - thanks for inspiration and chatting



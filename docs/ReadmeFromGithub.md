# SpotMicroAI 

I started this Project because i got inspired by some very smart People/Companies and Projects out there and want to 
understand and adapt their work. It is based on existing OpenSource-Projects and uses affordable Hardware to enable other people to build their own Bots and help us to understand how to control it the way we want.

This Project is heavily work in progress and may change every day. It is NOT a working or even finished Project.
It is intended to be a community-project, so please feel invited to contribute.

![PyBullet Simulation](/Images/SpotMicroAI_pybullet_lidar3.png)

[See the first movements of SpotMicroAI on YouTube](https://www.youtube.com/watch?v=vayiiII4xVQ) and [the first contact with the NVIDIA Jetson Nano](https://www.youtube.com/watch?v=no4voMsa7ZI)

Parts of this Project:
1. build a working physical Robot with cheap components everyone can build 
2. create a simulated Environment and be able to control the Robot 
3. to do RL training to make it learn how to stand/walk/run 

## 1. The physical Robot

![SpotMicroAI](/Images/SpotMicroAI_complete_1.jpg)


![Parts](/Images/SpotMicroAI_FreeCad.png)

### NVIDIA Jetson Nano

The Brain of all this is the NVIDIA JetsonNano. It has a 16 Channel PCA9685 I2C-Servo Driver connected, which is used to control the servos. The IMU (GY-521) is also connected via I2C and provides roll and pitch angles of the Robot.
The OLED-Display is used to have some nice output. 
I will provide a Fritzing-Layout in the near future.

![JetsonNano-Case](/Images/SpotMicroAI_jetson.jpg)

[You can find the all the Code for the Jetson Nano here](/JetsonNano).

### Sensors

SpotMicroAI has a RPi-Cam, Sonar-Sensors and an IMU Gyro/Accel-Sensor (as designed by KDY). In one of the first versions i added two additional Sonars at the bottom, but i had to remove them again to have space for the voltage regulators. 
TODO: We will need a Bottom-Case version 2 here.

The Rear-Part has space for an OLED-Display (SSD-1306) and a LED Power-Button.
In a first version i used an Arduino Mega as kind of Servo/Sensor-Controller and a Raspberry PI as Locomotion-Controller (communication via UART). But it showed up that the Arduino is too slow to handle Sensor-Signals and Servo-PWM properly at the same time. Now i use the Jetson with the PCA as described above.

I am not sure if the Hardware i use now will be enough to finally have a very smooth walking Robot like for example the real SpotMini. See this more as a Research-Project where I try to use cheap Hardware and other People's Work to learn more about how this all works. 



### Hardware-Todos

 - the whole Servo-Setup. MG996R not powerfull enough. try the CLS6336HV Servos. PCA might make the Servos jitter.
 - use an additional VoltageRegulator 7,4->5V for the Jetson Nano
 - build a PowerPack ? x 18650 7,4V ? - not sure if i really should build a PowerPack :) Need help here! 
 - build a nice Adapter-Cable to solve issues with cable-length of all 12 Servos. 
 
### Software-Todos / Ideas 
 
 - write a basic RL-based Implementation to support example_automaticgait.py with a "BodyBalancer". ActionSpace is x,y,z of the Body, ObservationSpace pitch,roll,ground_distance,kinematic_motion_function_index
 - create a ROS-Sim-Adapter to map the joint_states to the leg_topic 
 - create an Implementation for the Jetson/RealWorldBot which handles leg_topics -> Servos (incl. calibration)
 - write a Controller-Node, which uses the Kinematics/RL-Model to control the Bot via the leg_topics. This Controller-Node will just be a wrapper for the same logic we use for the PyBullet-Simulation. 
 - Finish the OpenAI-Gym-Env? or
 - try to adapt the "Neural Network Walker"-Example from Bullet to SpotMicroAI? not sure..
 - understand the Marc H. Raibert's Balancing Controller from 1984/86 to possibly merge this ideas into the Action/Observation-Space. Does this make sense?

### Inspiration/Papers

I like the Ideas of:
- [this Paper](https://arxiv.org/pdf/1804.10332.pdf) by
Jie Tan, Tingnan Zhang, Erwin Coumans, Atil Iscen, Yunfei Bai, Danijar Hafner, Steven Bohez, and Vincent Vanhoucke
Google Brain,Google DeepMind
- [this Paper](https://openreview.net/pdf?id=BklHpjCqKm) by Michael Lutter, Christian Ritter & Jan Peters ∗
- [and this one](https://arxiv.org/pdf/1810.03842.pdf) by Abhik Singla, Shounak Bhattacharya, Dhaivat Dholakiya,
Shalabh Bhatnagar, Ashitava Ghosal, Bharadwaj Amrutur and Shishir Kolathaya
- [also this one](https://arxiv.org/pdf/1903.02993.pdf) by Krzysztof Choromanski,Aldo Pacchiano,Jack Parker-Holder,Yunhao Tang,Deepali Jain,Yuxiang Yang,Atil Iscen,Jasmine Hsu,Vikas Sindhwani

## Credits and thanks

- Kim Deok-yeon creator of SpotMicro
- Boston Dynamics who built this incredible SpotMini,
- Ivan Krasin - https://ivankrasin.com/about/ - thanks for inspiration and chatting
- My colleagues at REWE digital / Research & Innovation for inspiration and feedback
- Jie Tan, Tingnan Zhang, Erwin Coumans, Atil Iscen, Yunfei Bai, Danijar Hafner, Steven Bohez, and Vincent Vanhoucke
Google Brain,Google DeepMind 

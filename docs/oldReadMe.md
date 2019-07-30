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

First of all thanks to Kim Deok-Yeon aka KDY0523 who made [this incredible work on Thingiverse](https://www.thingiverse.com/thing:3445283)

This basically is the physical Robot. It will take some days to print and assemble all the parts, but it's worth all the effort. I also sanded, primed and painted all the Parts to give it a nicer look.

[Here is my Thingiverse-Make](https://www.thingiverse.com/make:654812)

Since my setup required some additional Hardware, i recreated some parts using FreeCAD - see /urdf/FreeCAD-Directory.
You might notice the redunancy of some STL-files, which is caused by the "ROSification" and the slightly different Model-Structure. I will have to clean it up.

![Parts](/Images/SpotMicroAI_FreeCad.png)

Kim Deok-Yeon has updated his model for use with CLS6336HV-Servos. I will keep on using the MG996R-Servos - although they will not have enough power - but they are cheaper and i expect unwanted behaviours and servo-crashes in the first stages. 

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

### Next Steps

- I am still working on the PowerSupply. Currently SpotMicroAI is powered by two separate AC-Adapters for Jetson and PCA. I need another voltage-regulator for the Jetson (7,4v -> 5V). 

## 2. Simulation

Masses and Inertias of the URDF-Model are still not correct.

There is also a Blender-File included which i used to create the STLs for the simulation. 
Of course you could also do some nice renderings with it! :)

### Quickstart PyBullet

![PyBullet](/Images/SpotMicroAI_stairs.png)

This example can be found in the Repository. You need a GamePad for this to work:
```
pip3 install numpy
pip3 install pybullet
pip3 install inputs
...TODO: provide setup.py

cd Core/
python3 example_automatic_gait.py
```

### Quickstart for ROS

![urdf](/Images/SpotMicroAI_rviz_urdf.png)

There is also a first ROSification of SpotMicroAI.

First of all install ROS. I use Melodic, but it should work with Kinetic, too.
I will not go into detail on how to install ROS because there are many good Tutorials out there.

When finished installing ROS:

```
cd ~/catkin_ws/src
git clone https://github.com/FlorianWilk/SpotMicroAI.git
cd ..
catkin_make
source ./devel/setup.bash
roslaunch spotmicroai showmodel.launch
```

This will show up RVIZ with the Model of SpotMicroAI. 

### Kinematics

In order to be able to move the Robot or even make it walk, we need something which tells us what servo-angles
will be needed for a Leg to reach position XYZ.
This is what InverseKinematics does. We know all the constraints, the length of the legs, how the joints rotate and where they are positioned. 

You can find [some a first draft of the calculations here](https://github.com/FlorianWilk/SpotMicroAI/tree/master/Kinematics). There is also a [Jupyter Notebook explaining the Kinematics](https://github.com/FlorianWilk/SpotMicroAI/tree/master/Kinematics/Kinematic.ipynb) and a [YouTube-Video](https://www.youtube.com/watch?v=VSkqhFok17Q).

## 3. Training

There is no real Training-Code yet.
We need an environment which allows fast multiagent learning. A C++/Bullet-Implementation might be a good solution. 
We are also currently evaluating Unity as an Enviroment, but tbh i prefer to have a clean and tight C++ implementation instead of Mono/C#. On the other hand Unity would allow us to have far more realistic (virtual) VideoInputs for CV-Training. Not sure about this, would be great to have feedback here from you guys.
I also started to create a simple OpenAI-Gym-Environment, but have not finished it yet :/ 

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


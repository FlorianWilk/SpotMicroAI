## Simulation

Masses and Inertias of the URDF-Model are still not correct.

There is also a Blender-File included which i used to create the STLs for the simulation. 
Of course you could also do some nice renderings with it! :)

### Quickstart PyBullet

![PyBullet](assets/SpotMicroAI_stairs.png)

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

![urdf](assets/SpotMicroAI_rviz_urdf.png)

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


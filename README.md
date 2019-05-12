# SpotMicro AI 
This Project is work in progress and documenting my progress on the goal of making a four-legged robot walk.
It is NOT a working or even finished Project you might want to use! 

I started this Project because i got inspired by some very smart people/companies and projects out there and want to 
understand and adapt their work on my personal wish to create a small "clone" of BostonDynamics SpotMini.

My Goal is to create a) a working physical Robot, b) a simulated Environment to be able to c) do RL training to make it walk.
There are other ways of achiving this, i think. The use of InverseKinematics only, combined with a robust ground detection
could also solve the problem and might be "more straight-forward" / yet already very challenging. 
The reason why i want to try a combination of both is because i believe that a well trained RL-Model could be move stable and
robust in different situations where the physical robot leaves controlled environments or parts like Servos or Legs become unstable or even break.

Nevertheless we will have to create a precise IK-Model to be able to have some kind of guided training. 

## a) The Robot

First of all thanks to Deok-yeon Kim aka KDY0523 who made this incredible work 
https://www.thingiverse.com/thing:3445283
This basically already is the physical Robot. 
Since my setup required some additional Hardware, i recreated some parts using FreeCAD - see /Parts-Directory

## b) Simulation and c) Training

See https://arxiv.org/pdf/1804.10332.pdf by
Jie Tan, Tingnan Zhang, Erwin Coumans, Atil Iscen, Yunfei Bai, Danijar Hafner, Steven Bohez, and Vincent Vanhoucke
Google Brain,Google DeepMind

First of all a URDF needs to be created. The Model will be in URDF/

## Credits
Deok-yeon Kim creator of SpotMicro
Boston Dynamics who built this incredible SpotMini
Ivan Karsin - thanks inspiration and chatting



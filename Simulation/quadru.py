import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", 1.306569,-0.861674,0.123352,0.030554,-0.029004,0.136916,0.989686)]
objects = p.loadMJCF("../urdf/spotmicroai_gen.urdf.xml")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[1.306569,-0.861674,0.123352],[0.030554,-0.029004,0.136916,0.989686])
jointPositions=[ 0.000000, 0.000000, -0.218386, -1.499965, 2.470036, 0.000000, 0.060346, -1.499946, 2.469895, 0.000000, -0.008545, -1.499821, 2.469786, 0.000000, -0.195515, -1.499378, 2.474604, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.setGravity(0.000000,0.000000,-10.000000)
p.stepSimulation()
p.disconnect()

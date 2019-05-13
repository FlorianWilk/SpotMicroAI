import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", 1.269193,-0.882457,0.137345,-0.071467,0.330220,0.199073,0.919901)]
objects = p.loadMJCF("../urdf/spotmicroai_gen.urdf.xml")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[1.269193,-0.882457,0.137345],[-0.071467,0.330220,0.199073,0.919901])
jointPositions=[ 0.000000, 0.000000, -0.009910, 0.105063, 0.075391, 0.000000, 0.001461, 0.093942, 0.097545, 0.000000, -0.036768, 0.330368, 0.517599, 0.000000, -0.039455, 0.335885, 0.524092, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.setGravity(0.000000,0.000000,-10.000000)
p.stepSimulation()
p.disconnect()

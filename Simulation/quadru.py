import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", 1.126635,-0.990664,0.094553,0.126286,0.073189,-0.713761,0.685011)]
objects = p.loadMJCF("../urdf/spotmicroai_gen.urdf.xml")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[1.126635,-0.990664,0.094553],[0.126286,0.073189,-0.713761,0.685011])
jointPositions=[ 0.000000, 0.000000, -0.221213, -1.499316, 2.469956, 0.000000, -0.090642, -1.499110, 2.470026, 0.000000, -0.745084, -1.498874, 2.478846, 0.000000, 0.556185, -1.498443, 2.478495, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.setGravity(0.000000,0.000000,-9.810000)
p.stepSimulation()
p.disconnect()

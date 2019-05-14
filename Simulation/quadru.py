import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", 1.019864,-0.976667,0.129847,0.041700,-0.031063,0.154790,0.986578)]
objects = p.loadMJCF("../urdf/spotmicroai_gen.urdf.xml")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[1.019864,-0.976667,0.129847],[0.041700,-0.031063,0.154790,0.986578])
jointPositions=[ 0.000000, 0.000000, -0.268521, -1.499458, 2.470289, 0.000000, 0.049039, -1.499396, 2.469628, 0.000000, -0.036335, -1.499316, 2.472407, 0.000000, -0.249185, -1.498936, 2.475876, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.setGravity(0.000000,0.000000,-9.810000)
p.stepSimulation()
p.disconnect()

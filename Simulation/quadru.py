import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", 1.192231,-0.869173,0.035383,-0.664986,0.746856,0.000259,-0.000499)]
objects = p.loadMJCF("../urdf/spotmicroai_gen.urdf.xml")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[1.192231,-0.869173,0.035383],[-0.664986,0.746856,0.000259,-0.000499])
jointPositions=[ 0.000000, 0.000000, -0.015373, 0.115742, 0.025821, 0.000000, -0.020660, 0.078061, 0.062234, 0.000000, -0.020197, 0.317828, 0.501950, 0.000000, -0.023614, 0.328609, 0.505581, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.setGravity(0.000000,0.000000,-10.000000)
p.stepSimulation()
p.disconnect()

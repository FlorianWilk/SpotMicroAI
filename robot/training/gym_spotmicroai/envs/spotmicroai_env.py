import pybullet as p
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import pybullet_data
import time
class SpotMicroAIEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
    p.setRealTimeSimulation(1)
#    cubeId = p.loadURDF("cube_collisionfilter.urdf", [0, 0, 3], useMaximalCoordinates=False)
  def step(self, action):
    p.stepSimulation()
    time.sleep(.1)
  def reset(self):
    pass
  def render(self, mode='human', close=False):
    pass

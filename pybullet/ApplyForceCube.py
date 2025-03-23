import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadSDF("stadium.sdf")
p.setGravity(0, 0, 0)

sphere = p.loadURDF("cube_small.urdf")
p.resetBasePositionAndOrientation(sphere, [0, 0, 1], [0, 0, 0, 1])

forward = 0
left = 0
up = 0

cameraDistance = 1
cameraYaw = 35
cameraPitch = -35

while (1):

  spherePos, orn = p.getBasePositionAndOrientation(sphere)

  cameraTargetPosition = spherePos
  p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
  camInfo = p.getDebugVisualizerCamera()
  camForward = camInfo[5]
  camUp = camInfo[4]
  camLeft = np.cross(camUp, camForward)

  keys = p.getKeyboardEvents()
  for k, v in keys.items():

    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      left = -1
    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
      left = 0
    if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      left = 1
    if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
      left = 0

    if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      forward = 1
    if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
      forward = 0
    if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      forward = -1
    if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
      forward = 0

    if (k == p.B3G_RETURN and (v & p.KEY_WAS_TRIGGERED)):
      up = 1
    if (k == p.B3G_RETURN and (v & p.KEY_WAS_RELEASED)):
      up = 0
    if (k == p.B3G_SHIFT and (v & p.KEY_WAS_TRIGGERED)):
      up = -1
    if (k == p.B3G_SHIFT and (v & p.KEY_WAS_RELEASED)):
      up = 0

  forceF = [100*forward * camForward[0], 100*forward * camForward[1], 0]
  forceL = [100*left * camLeft[0], 100*left * camLeft[1], 0]
  forceU = [0, 0, 100*up*camUp[2]]


  offset = [0, 0, 0]
  forcePos1 = np.array(spherePos) + offset

  if (forward):
    p.applyExternalForce(sphere, -1, forceF, forcePos1, flags=p.WORLD_FRAME)

  if (left):
    p.applyExternalForce(sphere, -1, forceL, forcePos1, flags=p.WORLD_FRAME)

  if (up):
    p.applyExternalForce(sphere, -1, forceU, forcePos1, flags=p.WORLD_FRAME)

  p.stepSimulation()
  time.sleep(1. / 240.)
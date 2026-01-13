import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('The_Cube/The_Cube.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  while True:
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
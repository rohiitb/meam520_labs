"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
import rospy
import numpy as np
from math import pi

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(0.2)

arm.close_gripper()

q = arm.neutral_position()
arm.safe_move_to_position(q)
arm.open_gripper()

q = np.array([0.5,-1 ,0.2,-2,0,1,1]) # TODO: try changing this!
# q = np.array([-0.01779206022777, -0.7601235411041661, 0.019782607023391807, -2.342050140544315, 0.029840531355804868, 1.5411935298621688, 0.7534486589746342])
# q = np.array([-0.2779206022777, -0.7601235411041661, 0.019782607023391807, -2.342050140544315, 0.029840531355804868, 1.5411935298621688, 0.7534486589746342])





arm.safe_move_to_position(q)
print("Finished")
arm.close_gripper()

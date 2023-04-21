import sys
import numpy as np
from copy import deepcopy
from math import pi
from time import perf_counter

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

from lib.calculateFK import FK
from lib.calcJacobian import FK
from lib.solveIK import IK

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

def my_ik(ik, target, seed):
    start = perf_counter()
    
    q, success, rollout = ik.inverse(target, seed)
    stop = perf_counter()
    time_taken = stop - start

    if success:
        print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=time_taken, it=len(rollout)))
        # arm.safe_move_to_position(q)
    else:
        print('IK Failed for this target using this seed.')

    return q  # return the q as a backup for further use

def pick_static_block(team, start_position, arm, detector, ik, fk):
    if team == 'red':
        # Starting configuration to view the blocks
        start_position = np.array([-0.01779206-pi/9, -0.76012354+pi/6,  0.01978261, -2.34205014+pi/8, 0.02984053, 1.54119353+pi/11, 0.75344866]) # Can change this to b emore closer to the blocks
        
        # Lifting position above the stacked blocks
        # lift_up = np.array([0.24883 , 0.12151,  0.1251 , -1.2471 , -0.01544 , 1.36769  ,1.1553])

        lifting_pos = [np.array([0.23626,  0.10206,  0.12696, -2.05665, -0.0155 ,  2.15784,  1.15655]),
                        np.array([0.20915,  0.0544 ,  0.15375, -1.98374, -0.00932,  2.03749,  1.15227]),
                        np.array([0.19296,  0.02448,  0.16897, -1.89087, -0.00437,  1.91499,  1.14876]),
                        np.array([0.18715,  0.0134 ,  0.17428, -1.77656, -0.00238,  1.78976,  1.14733])]

    else:
        start_position = np.array([-0.01779206+pi/9, -0.76012354+pi/6,  0.01978261, -2.34205014+pi/8, 0.02984053, 1.54119353+pi/11, 0.75344866])
        # lift_up = np.array([0.24883 , 0.12151,  0.1251 , -1.2471 , -0.01544 , 1.36769  ,1.1553])     # Need to change this for blue

        lifting_pos = [np.array([-0.16884,  0.1032 , -0.19605, -2.05661,  0.0241 ,  2.15771,  0.40817]),
                        np.array([-0.17478,  0.05473, -0.18874, -1.98374,  0.01149,  2.03747,  0.41698]),
                        np.array([-0.17813,  0.02454, -0.18396, -1.89086,  0.00477,  1.91499,  0.42175]),
                        np.array([-0.17933,  0.01342, -0.18216, -1.77656,  0.00249,  1.78976,  0.42339])]

    arm.safe_move_to_position(start_position) # on your mark!


    _, T0e = fk.forward(start_position)

    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()

    # Detect some blocks...
    arm.open_gripper()

    ik_q = []

    for (name, pose) in detector.get_detections():
        # Get the block transform in base frame
        target = T0e @ H_ee_camera @ pose
        print(name,'\n',target)

        # Transform the axes so that Z points downwards
        here = transform_axes(target)

        # Solve IK
        q_found = my_ik(ik, here, start_position)

        # Append ik solutions for all blocks
        ik_q.append(q_found)

        print()
        print("-----------------------------------------------------")

    if team == 'red':
        # Working seeds for red dropping positions
        # For the location - target[:3,-1] = np.array([0.531, 0.2, 0.225+0.01+0.35]) , target[:3,:3] = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        dropping_seeds = [np.array([0.2755 ,  0.1664 ,  0.08685, -2.1101 , -0.01887,  2.27579,  1.15878]),
                        np.array([0.23626,  0.10206,  0.12696, -2.05665, -0.0155 ,  2.15784,  1.15655]),
                        np.array([0.20915,  0.0544 ,  0.15375, -1.98374, -0.00932,  2.03749,  1.15227]),
                        np.array([0.19296,  0.02448,  0.16897, -1.89087, -0.00437,  1.91499,  1.14876])]
    else:
        # Working seeds for blue dropping positions
        # For the location - target[:3,-1] = np.array([0.531, -0.2, 0.225+0.01+0.35]) , target[:3,:3] = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        dropping_seeds = [np.array([-0.16047,  0.16926, -0.20484, -2.10994,  0.04498,  2.27522,  0.39379]),
                            np.array([-0.16884,  0.1032 , -0.19605, -2.05661,  0.0241 ,  2.15771,  0.40817]),
                            np.array([-0.17478,  0.05473, -0.18874, -1.98374,  0.01149,  2.03747,  0.41698]),
                            np.array([-0.17813,  0.02454, -0.18396, -1.89086,  0.00477,  1.91499,  0.42175])]            

    for i in range(len(ik_q)):
        arm.open_gripper()
        arm.safe_move_to_position(ik_q[i])
        arm.exec_gripper_cmd(0.049, 50)   # arm.close_gripper()

        arm.safe_move_to_position(start_position)
        arm.safe_move_to_position(dropping_seeds[i])
        arm.open_gripper()
        arm.safe_move_to_position(lifting_pos[i])
        arm.safe_move_to_position(start_position)
    

def Rot_x(theta):
    return np.array([[1,0,0],[0,np.cos(theta), -np.sin(theta)],[0,np.sin(theta), np.cos(theta)]])

def Rot_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],[0,1,0],[-np.sin(theta), 0, np.cos(theta)]])

def Rot_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), 0],[0,0,1]])

def transform_axes(mat):
    up_vec = mat[:3,:3].T @ np.array([0,0,1])
    idx = int(np.where(np.abs(np.round(up_vec,1)) == 1)[0])
    sign_ = int(np.round(up_vec,1)[idx] / 1)
    
    if sign_ > 0 and idx == 0:
        int_rot = Rot_y(pi/2) @ Rot_x(pi)
    elif sign_ < 0 and idx == 0:
        int_rot = Rot_y(-pi/2) @ Rot_x(pi)
    elif sign_ > 0 and idx == 1:
        int_rot = Rot_x(-pi/2) @ Rot_x(pi)
    elif sign_ < 0 and idx == 1:
        int_rot = Rot_x(pi/2) @ Rot_x(pi)
    elif sign_ > 0 and idx == 2:
        int_rot = Rot_x(pi)
    else: 
        int_rot = np.eye(3)

    
    R_new = np.eye(4)
    R_new[:3, -1] = mat[:3, -1]
    R_new[:3,:3] = mat[:3,:3] @ int_rot
    if R_new[0,0] < 0:
        R_new[:3,:3] = R_new[:3,:3] @ Rot_z(pi)
    return R_new


if __name__ == "__main__":

        try:
            team = rospy.get_param("team") # 'red' or 'blue'
        except KeyError:
            print('Team must be red or blue - make sure you are running final.launch!')
            exit()

        rospy.init_node("team_script")
        arm = ArmController()
        detector = ObjectDetector()
        ik = IK()
        fk = FK()

        print("\n****************")
        if team == 'blue':
            print("** BLUE TEAM  **")
        else:
            print("**  RED TEAM  **")
        print("****************")
        input("\nWaiting for start... Press ENTER to begin!\n") # get set!
        print("Go!\n") # go!

        # STUDENT CODE HERE
        if team == 'red':
            start_position = np.array([-0.01779206-pi/9, -0.76012354+pi/6,  0.01978261, -2.34205014+pi/8, 0.02984053, 1.54119353+pi/11, 0.75344866]) # Can change this to be more closer to the blocks
        else:
            start_position = np.array([-0.01779206+pi/9, -0.76012354+pi/6,  0.01978261, -2.34205014+pi/8, 0.02984053, 1.54119353+pi/11, 0.75344866]) # Can change this to be more closer to the blocks

        # Static block stacking
        pick_static_block(team, start_position, arm, detector, ik, fk)

        

        # Move around...

            


    # END STUDENT CODE
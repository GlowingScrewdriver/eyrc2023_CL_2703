#!/usr/bin/python3

# ============================================================
# Team ID:          CL#2703
# Theme:            Cosmo Logistic
# Author List:		Vedanth Padmaraman, Vamshi Vishruth
# Filename:		    task3b.py
#
# Functions:        get_package_config
# ============================================================

from cl2703.task2b import RackShift
from cl2703.task2a import PickAndDrop
import rclpy, threading
from config_utils import get_package_config

if __name__ == "__main__":
    rclpy.init ()

    executor = rclpy.executors.MultiThreadedExecutor (4)

    # Ebot navigation and docking control
    docker = RackShift ()
    executor.add_node (docker)

    # Arm and gripper control
    arm_control = PickAndDrop ("arm_control")
    executor.add_node (arm_control)

    rack_pose_info = get_package_config ('config.yaml')

    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    for rack in rack_pose_info:
        docker.rack_shift (rack)
        input ('Press [enter] to continue with the task')
        #arm_control.pick_and_drop ([ rack['box'] ])
        #arm_control.motion ()
    
    rclpy.shutdown ()
    exit ()
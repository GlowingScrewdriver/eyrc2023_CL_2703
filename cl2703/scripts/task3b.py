#!/usr/bin/python3

# ============================================================
# Team ID:          CL#2703
# Theme:            Cosmo Logistic
# Author List:		Vedanth Padmaraman, Vamshi Vishruth
# Filename:		    task3b.py
#
# Functions:        get_package_config
# ============================================================

from task2b import RackShift
from task2a import PickAndDrop
import tf2_ros, rclpy, threading
import yaml

def get_package_config (filename):
    '''
    Gets the task requirement information

    Args:
        filename (str): configuration filename. Usually `config.yaml`, provided by eYRC
    Returns:
        rack info dictionary. Refer rack_pose_info assignment under __name__ == "__main__" for details
    '''
    config_f = open ('config.yaml')
    rack_info = yaml.safe_load (config_f)
    config_f.close ()

    rack_poses = []
    box_ids = rack_info['package_id']
    for box in box_ids:
        for rack in rack_info['position']:
            if f'rack{box}' in rack:
                rack = rack[f'rack{box}']
                rack_poses += [{
                    'pickup': {'trans': [rack[0], rack[1]], 'rot': rack[2]},
                    'drop': {'trans': [0.85, -2.455], 'rot': 3.14},
                    'rack': f'rack{box}',
                    'box': box,
                }]
                break

    return rack_poses


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
        #input ('Press [enter] to continue with the task')
        docker.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))
        arm_control.pick_and_drop ([ rack['box'] ])
        arm_control.motion ()
    
    rclpy.shutdown ()
    exit ()
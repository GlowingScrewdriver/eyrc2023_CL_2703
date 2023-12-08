#!/usr/bin/python3

from task1a import aruco_tf
from task2b import RackShift
from task2a import PickAndDrop
import tf2_ros, rclpy, threading


if __name__ == "__main__":
    rclpy.init ()

    executor = rclpy.executors.MultiThreadedExecutor (4)

    # Ebot navigation and docking control
    docker = RackShift ()
    executor.add_node (docker)

    # Arm and gripper control
    arm_control = PickAndDrop ("arm_control")
    executor.add_node (arm_control)

    rack_pose_info = {
        # Rack pickup and drop poses
        #'pickup': {'trans': [2.02, -8.09], 'rot': 1.57},
        'pickup': {'trans': [1.26, 4.34], 'rot': 3.14},
        'drop': {'trans': [0.7, -2.455], 'rot': 3.14},
        'rack': 'rack1',
    }
    # Note: These are NOT the expected robot poses; these are the exact rack poses

    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    docker.rack_shift (rack_pose_info)
    input ('Press [enter] to continue with the task')
    arm_control.pick_and_drop ([1])
    arm_control.motion ()
    
    rclpy.shutdown ()
    exit ()
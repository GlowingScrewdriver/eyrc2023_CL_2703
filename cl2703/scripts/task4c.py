#!/usr/bin/python3

from task3b import get_package_config
from task2b import RackShift
from task2a import PickAndDrop
import rclpy, threading

if __name__ == "__main__":
    rclpy.init ()

    executor = rclpy.executors.MultiThreadedExecutor (4)

    # Ebot navigation and docking control
    docker = RackShift ()
    executor.add_node (docker)

    # Arm and gripper control
    arm_control = PickAndDrop ("arm_control")
    executor.add_node (arm_control)
    # Separate thread for the motion function
    arm_motion_th = threading.Thread ()

    rack_pose_info = sorted (
        get_package_config ('config.yaml'),
        key = lambda rack: abs(rack['pickup']['trans'][1] - rack['drop']['trans'][1])
    )

    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    for rack in rack_pose_info:
        docker.rack_shift (rack)
        input ('Press [enter] to continue with the task')
        docker.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))
        arm_control.pick_and_drop ([ rack['box'] ])

        if not arm_motion_th.is_alive ():
            arm_motion_th = threading.Thread (target = arm_control.motion)
            arm_motion_th.start ()
    
    rclpy.shutdown ()
    exit ()
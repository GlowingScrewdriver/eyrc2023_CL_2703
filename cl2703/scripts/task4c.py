#!/usr/bin/python3

from cl2703.task3b import get_package_config
from cl2703.task2b import RackShift
from cl2703.task2a import PickAndDrop
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

    rack_pose_info = get_package_config ('config.yaml')

    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    for rack in rack_pose_info:
        if (rack['pickup']['trans'][0] - rack['drop']['trans'][0])**2 + (rack['pickup']['trans'][1] - rack['drop']['trans'][1])**2 > 0.5**2: # Rack distance from drop pose
            docker.rack_shift (rack)
        input ('Press [enter] to continue with the task')
        docker.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))
        arm_control.pick_and_drop ([ rack['box'] ])

        if not arm_motion_th.is_alive ():
            arm_motion_th = threading.Thread (target = arm_control.motion)
            arm_motion_th.start ()
    
    rclpy.shutdown ()
    exit ()
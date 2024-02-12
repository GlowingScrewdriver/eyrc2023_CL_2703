#!/usr/bin/python3

from cl2703.task3b import get_package_config
#from cl2703.task2b import RackShift
from cl2703.task4a import RackShiftHW
#from cl2703.task2a import PickAndDrop
from cl2703.task4b import PickAndDropHW
import rclpy, threading
from time import sleep
from cl2703.task2b import normalize_angle
from math import pi

if __name__ == "__main__":
    rclpy.init ()

    # Task instructions
    # This list also serves as the main control signal for
    # the entire routine. Threads running in the background
    # end when this list becomes empty
    rack_pose_info = get_package_config ('config.yaml')

    executor = rclpy.executors.MultiThreadedExecutor (4)

    # Ebot navigation and docking control
    docker = RackShiftHW ()
    executor.add_node (docker)

    # Arm and gripper control
    arm_control = PickAndDropHW ("arm_control")
    executor.add_node (arm_control)
    # Separate thread for the motion function
    # This allows us to simply push new destinations to arm_control.destinations
    # without having to explicitly call arm_control.motion ()
    arm_motion_th = threading.Thread (target = arm_control.motion)
    def arm_motion_async (): # Dispatch a call to arm_control.motion in the background
        if not arm_motion_th.is_alive ():
            #arm_motion_th = threading.Thread (target = arm_control.motion)
            threading.Thread.__init__ (arm_motion_th, target = arm_control.motion) # So that arm_motion_th is not treated as a local variable
            arm_motion_th.start ()

    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    # This pose, with the `shoulder` field set appropriately, preempts
    # the next box's pose
    preempt_pose = {
        'position': None,
        'shoulder': None,
        'callback': lambda: None,
        'retract': False,
    }

    for rack in rack_pose_info:
    #while rack_pose_info:
        #rack = rack_pose_info[0]
        if (rack['pickup']['trans'][0] - rack['drop']['trans'][0])**2 + (rack['pickup']['trans'][1] - rack['drop']['trans'][1])**2 > 0.5**2: # Rack distance from drop pose
            # Preemptively move to shoulder angle of upcoming rack
            p = preempt_pose.copy ()
            p['shoulder'] = normalize_angle(rack['drop']['rot'] + pi)
            arm_control.destinations += [p]
            arm_motion_async ()

            # Shift the rack
            docker.rack_shift (rack)

        #input ('Press [enter] to continue with the task')
        docker.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))
        arm_control.pick_and_drop ([ rack['box'] ])

        arm_motion_async ()
        # When there is only one element left in rack_pose_info, this serves
        # as a signal to stop. This loop, as well as Thread arm_loop_th will exit.
        #rack_pose_info.pop (0)

    arm_motion_th.join ()

    rclpy.shutdown ()
    exit ()

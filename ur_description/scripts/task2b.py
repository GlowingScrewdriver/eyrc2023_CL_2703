#!/usr/bin/python3
from task1c import Navigator
from geometry_msgs.msg import Pose, PoseStamped
from linkattacher_msgs.srv import AttachLink, DetachLink
from ebot_docking.srv import DockSw
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from math import sin, cos
import time, threading

if __name__ == "__main__":
    rclpy.init ()

    navigator = Navigator ()
    navigator.waitUntilNav2Active ()

    docker = rclpy.create_node ('docking_client')
    executor = rclpy.executors.MultiThreadedExecutor (2)
    executor.add_node (docker)
    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    callback_group = ReentrantCallbackGroup ()
    # Clients and corresponding request objects
    if False:
        dock_cli, dock_req, attach_cli, attach_req, detach_cli, detach_req = [None]*6
        for c in ([dock_cli, DockSw, '/dock_control', dock_req], [attach_cli, AttachLink, '/ATTACH_LINK', attach_req], [detach_cli, DetachLink, '/DETACH_LINK', detach_req]):
            # Believe me, it's more painful to read when expanded XD
            c[0] = node.create_client (c[1], c[2], callback_group=callback_group)
            c[3] = c[1].Request ()

    if True:
        dock_cli   = docker.create_client (DockSw, '/dock_control', callback_group=callback_group)
        dock_req   = DockSw.Request ()
        attach_cli = docker.create_client (AttachLink, '/ATTACH_LINK', callback_group=callback_group)
        attach_req = AttachLink.Request (model1_name='ebot', link1_name='ebot_base_link', link2_name='link')
        detach_cli = docker.create_client (DetachLink, '/DETACH_LINK', callback_group=callback_group)
        detach_req = DetachLink.Request (model1_name='ebot', link1_name='ebot_base_link', link2_name='link')

    pose_list = [
        # Rack pickup and drop poses
        {   # Rack 1
            'pickup': {'trans': [1.26, 4.35], 'rot': 3.14},
            'drop': {'trans': [0.5, -2.455], 'rot': 3.14},
            'rack': 'rack1',
        },
        {   # Rack 2
            'pickup': {'trans': [2.03, 3.30], 'rot': -1.57},
            'drop': {'trans': [0.8, -1.9], 'rot': -1.57},
            'rack': 'rack2',
        },
        {   # Rack 3
            'pickup': {'trans': [2.03, -8.09], 'rot': 1.57},
            'drop': {'trans': [0.8, -3.1], 'rot': 1.57},
            'rack': 'rack3',
        },
        # Note: These are NOT the expected robot poses; these are the exact rack poses
    ][0:1]

    while not navigator.robot_pose:
        time.sleep (2)
    navigator.setInitialPose (navigator.robot_pose)

    for pose in pose_list:
        print (f'Starting at {navigator.get_clock().now()}')

        # Navigate to the rack
        pickup_pose = pose['pickup']
        # Request the robot to move half a metre away from the actual rack position
        pickup_pose['trans'][0] += cos(pickup_pose['rot'])*0.7
        pickup_pose['trans'][1] += sin(pickup_pose['rot'])*0.7
        navigator.navigate (pickup_pose)
        print (f'Reached rack')

        # Dock and pick up the rack
        dock_req.orientation = pickup_pose['rot']
        dock_cli.call(dock_req)
        print (f'Reached rack, position is {navigator.robot_pose}')
        attach_req.model2_name = pose['rack']
        print (attach_cli.call (attach_req))
        print (f'Attached rack')

        # Navigate, again, half a metre away from desired pose
        drop_pose = pose['drop']
        drop_pose['trans'][0] += cos(drop_pose['rot'])*0.5
        drop_pose['trans'][1] += sin(drop_pose['rot'])*0.5
        navigator.navigate (drop_pose)
        print (f'Reached arm pose')

        # Detach the rack
        detach_req.model2_name = pose['rack']
        print (detach_cli.call (detach_req))
        print (f'Detached rack, position is {navigator.robot_pose}')

        # Back to home pose
        #navigator.navigate ({'trans': [0.0, 0.0], 'rot': 0.0})

        print (f'Finishing at {navigator.get_clock().now()}')

    rclpy.shutdown ()
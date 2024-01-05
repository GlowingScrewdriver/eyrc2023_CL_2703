#!/usr/bin/python3
import os
import launch
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ctx = launch.launch_context.LaunchContext ()
    return launch.LaunchDescription([

        # Navigator2 stack
        IncludeLaunchDescription (PythonLaunchDescriptionSource (
            os.path.join (get_package_share_directory('ebot_nav2'), 'launch/ebot_bringup_launch.py')
        )),

        # Docking service
        launch_ros.actions.Node(
            package='ebot_docking',
            executable='ebot_docking_service_task2b.py',
        ),

        IncludeLaunchDescription (PythonLaunchDescriptionSource (
            os.path.join (get_package_share_directory('ur5_moveit'), 'launch/spawn_controllers.launch.py')
        )),

        # UR5 Arm and Moveit! stack
        launch.actions.TimerAction (period = 2.0, actions = [
            IncludeLaunchDescription (PythonLaunchDescriptionSource (
                os.path.join (get_package_share_directory('ur5_moveit'), 'launch/spawn_ur5_launch_moveit.launch.py')
            )),
        ]),

        # Servo node trigger
        launch.actions.ExecuteProcess(
            cmd='ros2 service call /servo_node/start_servo std_srvs/srv/Trigger'.split(' '),
            output='both',
        ),

        # Aruco detector
        launch_ros.actions.Node(
            package='cl2703',
            executable='task1a.py',
        ),

        # Warehouse in Gazebo
        # This MUST be the last entry in the launch. Else the launch crashes
        # Not sure why. Could be something related to the error handling process of launch
        IncludeLaunchDescription ( PythonLaunchDescriptionSource (
            os.path.join (get_package_share_directory('eyantra_warehouse'), 'launch/task4c.launch.py')
        )),

    ])
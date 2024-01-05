#!/usr/bin/python3

from task2a import PickAndDrop
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController # module call
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
import threading

class PickAndDropHW (PickAndDrop):
    def __init__(self, name):
        """
        Includes tweaks for the hardware arena
        """
        PickAndDrop.__init__(self, name)
        callback_group = ReentrantCallbackGroup ()

        # Gripper control
        self.gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io', callback_group=callback_group)
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')

        # Controller mode switching
        self.__contolMSwitch = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=callback_group)
        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Service control Manager is not yet available...")


    def pose_goal (self, *pargs, **kargs):
        """
        Wrapper around `PickAndDrop.pose_goal`. Handles controller mode switching
        """
        self.control_mode_switch (servo = False)
        PickAndDrop.pose_goal (self, *pargs, **kargs)
        self.control_mode_switch (servo = True)


    def control_mode_switch (self, servo = False):
        '''
        Switch controller mode between joint trajectory and servo controller
        servo (bool): true if servo is desired, else false
        '''
        #print (f"testing controller switch: servo is {servo}")

        modes = {
            True: ["forward_position_controller"],         # For servo
            False: ["scaled_joint_trajectory_controller"], # For normal usage
        }

        # Parameters to switch controller
        switchParam = SwitchController.Request()
        switchParam.activate_controllers = modes[servo]
        switchParam.deactivate_controllers = modes[not servo]
        switchParam.strictness = 2
        switchParam.start_asap = False

        print ("switching to ", switchParam.activate_controllers)

        #self.__contolMSwitch.call_async(switchParam)
        self.__contolMSwitch.call(switchParam)
        print("[CM]: Switching Complete")


    def gripper_call(self, state):
        '''
        based on the state given as i/p the service is called to activate/deactivate
        pin 16 of TCP in UR5
        i/p: node, state of pin:Bool
        o/p or return: response from service call
        '''
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        #self.gripper_control.call_async(req)
        self.gripper_control.call(req)
        return state


    def attach_box_with_gripper(self):
        self.gripper_call (True)

    def detach_box_from_gripper(self):
        self.gripper_call (False)


if __name__ == "__main__":
    rclpy.init ()
    executor = rclpy.executors.MultiThreadedExecutor (3)

    node = PickAndDropHW ("arm_control")
    executor.add_node (node)

    th = threading.Thread (target = executor.spin)
    th.start ()

    node.pick_and_drop ([2])
    node.motion ()

    rclpy.shutdown ()
    exit ()

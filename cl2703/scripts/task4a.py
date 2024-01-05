#!/usr/bin/python3

import rclpy
import threading
from config_utils import get_package_config
from usb_relay.srv import RelaySw
from task2b import RackShift

class RackShiftHW (RackShift):
    def __init__ (self):
        RackShift.__init__(self)
        self.grip_cli = self.create_client (RelaySw, '/usb_relay_sw', callback_group=self.callback_group)

    def rack_grip (self, rack, state = True):
        '''
        Attach/detach a rack
        '''
        # ros2 service call /usb_relay_sw usb_relay/srv/RelaySw "{relaychannel: 0, relaystate: true}"
        req = RelaySw.Request ()
        req.relaystate = state
        req.relaychannel = 0

        print (self.grip_cli.call (req))


if __name__ == "__main__":
    rclpy.init ()

    executor = rclpy.executors.MultiThreadedExecutor (2)

    # Ebot navigation and docking control
    docker = RackShiftHW ()
    executor.add_node (docker)

    rack_pose_info = get_package_config ('config.yaml')

    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    for rack in rack_pose_info:
        docker.rack_shift (rack)
        input ('Press [enter] to continue with the task')
    
    rclpy.shutdown ()
    exit ()
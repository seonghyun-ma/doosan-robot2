import rclpy
from rclpy.logging import get_logger
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

import DR_init
DR_init.__dsr__id = 'dsr02'
DR_init.__dsr__model = 'm0609'

logger = get_logger('multi_robot_2')

j_vel = 150
j_acc = 150

def main(args=None):
    print('## start ##')

    rclpy.init(args=args)
    node = rclpy.create_node('multi_robot_2', namespace='dsr02')
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, set_robot_mode, set_tool_digital_output,
            ROBOT_MODE_AUTONOMOUS
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # Sync Subscriber
    class SyncSubscriber(Node):
        def __init__(self):
            super().__init__('sync_subscriber', namespace='dsr02')
            self.subscription = self.create_subscription(
                Bool,
                '/sync_start',
                self.callback,
                10
            )
            self.sync_received = False

        def callback(self, msg):
            if msg.data:
                self.sync_received = True

    sync_sub = SyncSubscriber()

    def grasp():
        set_tool_digital_output(2, 0)
        set_tool_digital_output(3, 1)
        time.sleep(0.5)

    def release():
        set_tool_digital_output(2, 1)
        set_tool_digital_output(3, 0)
        time.sleep(0.5)

    movej([0, 0, 90, 0, 90, 0], vel=j_vel, acc=j_acc)

    print(">>> Waiting for sync signal...")

    try:
        while rclpy.ok():

            # waiting Sync signal
            while rclpy.ok():
                rclpy.spin_once(sync_sub, timeout_sec=0.1)
                if sync_sub.sync_received:
                    print(">>> Sync received!")
                    break

            # Sunc signal init
            sync_sub.sync_received = False

            movej([0,  0, 90, 0, 90, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 80, 0, 90, 0], vel=j_vel, acc=j_acc)
            movej([0,-10,100, 0, 80, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 90,10, 70, 0], vel=j_vel, acc=j_acc)
            movej([10,  0, 90, 0, 80, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 90, 0, 90, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 80, 0, 90, 0], vel=j_vel, acc=j_acc)
            movej([0,-10,100, 0, 80, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 90,10, 70, 0], vel=j_vel, acc=j_acc)
            movej([10,  0, 90, 0, 80, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 90, 0, 90, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 80, 0, 90, 0], vel=j_vel, acc=j_acc)
            movej([0,-10,100, 0, 80, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 90,10, 70, 0], vel=j_vel, acc=j_acc)
            movej([10,  0, 90, 0, 80, 0], vel=j_vel, acc=j_acc)
            movej([0,  0, 90, 0, 90, 0], vel=j_vel, acc=j_acc)

            
            break

    except KeyboardInterrupt:
        print('## Shutdown requested ##')

    rclpy.shutdown()
    print('## fin ##')

if __name__ == '__main__':
    main()
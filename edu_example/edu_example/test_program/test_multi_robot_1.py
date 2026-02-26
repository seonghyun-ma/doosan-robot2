import rclpy
from rclpy.logging import get_logger
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

import DR_init
DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = 'm0609'

logger = get_logger('multi_robot_1')

j_vel = 150
j_acc = 150

def main(args=None):
    print('## start ##')

    rclpy.init(args=args)
    node = rclpy.create_node('multi_robot_1', namespace='dsr01')
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

    # Sync Publisher
    sync_pub = node.create_publisher(Bool, '/sync_start', 10)

    def grasp():
        set_tool_digital_output(2, 0)
        set_tool_digital_output(3, 1)
        time.sleep(1)

    def release():
        set_tool_digital_output(2, 1)
        set_tool_digital_output(3, 0)
        time.sleep(1)

    movej([0, 0, 90, 0, 90, 0], vel=j_vel, acc=j_acc)

    try:
        while rclpy.ok():

            # Publish Sync Start signal
            msg = Bool()
            msg.data = True
            sync_pub.publish(msg)
            print(">>> Sync Start Published")

            # Sync stop signal
            msg.data = False
            sync_pub.publish(msg)
            print(">>> Sync Stop Published")

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
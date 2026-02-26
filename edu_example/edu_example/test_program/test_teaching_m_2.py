import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from rclpy.logging import get_logger

import DR_init
DR_init.__dsr__id = 'dsr02'
DR_init.__dsr__model = 'm0609'

logger = get_logger('test_teaching_m_2_node')

def main(args=None):
    print('## start ##')

    rclpy.init(args=args)
    node = rclpy.create_node('test_teaching_m_2_node', namespace='dsr02')
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, set_robot_mode,
            ROBOT_MODE_AUTONOMOUS,
            add_tcp, set_tcp
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    class SyncSubscriber(Node):
        def __init__(self):
            super().__init__('sync_sub', namespace='dsr02')

            self.pos_list = None
            self.start = False

            self.create_subscription(
                Float64MultiArray,
                '/pos_list',
                self.pos_callback,
                10
            )

            self.create_subscription(
                Bool,
                '/start_sync',
                self.start_callback,
                10
            )

        def pos_callback(self, msg):
            if len(msg.data) == 0:
                return

            self.pos_list = [
                [float(x) for x in msg.data[i:i+6]]
                for i in range(0, len(msg.data), 6)
            ]

        def start_callback(self, msg):
            if msg.data:
                self.start = True

    sub = SyncSubscriber()

    try:
        while rclpy.ok():

            j_vel = 60
            j_acc = 60
            l_velx = [300,300]
            l_accx = [300,300]
            movej([0,0,90,0,90,0], vel=j_vel, acc=j_acc)

            print('waiting for data...')

            # wait until both pose and start signal received
            while rclpy.ok():
                rclpy.spin_once(sub, timeout_sec=0.1)

                if sub.pos_list is not None and sub.start:
                    print('sync received')
                    break

            pos_list = sub.pos_list

            print('moving start')

            for pos in pos_list:
                pos = [float(x) for x in pos]
                movej(pos, vel=j_vel, acc=j_acc)

            movej([0,0,90,0,90,0], vel=j_vel, acc=j_acc)

            break

    except KeyboardInterrupt:
        print("shutdown")

    rclpy.shutdown()
    print("fin")

if __name__ == '__main__':
    main()
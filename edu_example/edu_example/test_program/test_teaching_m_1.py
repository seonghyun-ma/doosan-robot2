import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from rclpy.logging import get_logger

import DR_init
DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = 'm0609'

logger = get_logger('test_teaching_m_1_node')

def main(args=None):
    print('## start ##')

    rclpy.init(args=args)
    node = rclpy.create_node('test_teaching_m_1_node', namespace='dsr01')
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, set_robot_mode,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
            get_current_posj, get_digital_input, wait,
            add_tcp, set_tcp
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    pose_pub = node.create_publisher(Float64MultiArray, '/pos_list', 10)
    start_pub = node.create_publisher(Bool, '/start_sync', 10)

    try:
        while rclpy.ok():

            pos_list = []
            j_vel = 60
            j_acc = 60
            l_velx = [300,300]
            l_accx = [300,300]

            movej([0,0,90,0,90,0], vel=j_vel, acc=j_acc)
            set_robot_mode(ROBOT_MODE_MANUAL)

            print('\nDigital Input 5 = save pose')
            print('Digital Input 6 = wait run program after saving pose')
            print('Digital Input 7 = run program\n')
            print("let's save pose\n")

            while True:
                if get_digital_input(5) == 1:
                    pos = get_current_posj()
                    pos_list.append(pos)

                    print(f"pos save {len(pos_list)}")

                    while get_digital_input(5) == 1:
                        pass

                if get_digital_input(6) == 1:
                    break
            print('\nsaving process fin')

            # pos_list = [
            #     [0,  0, 90, 0, 90, 0],
            #     [0,  0, 80, 0, 90, 0],
            #     [0,-10,100, 0, 80, 0],
            #     [0,  0, 90,10, 70, 0],
            # ]

            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            movej([0,0,90,0,90,0], vel=j_vel, acc=j_acc)
            print('\nready')

            while get_digital_input(7) == 0:
                pass

            print('\nmoving start')

            # publish pose list
            msg = Float64MultiArray()
            flat = []
            for p in pos_list:
                flat.extend([float(x) for x in p])
            msg.data = flat
            pose_pub.publish(msg)

            print('pose_list published')

            # wait briefly to ensure delivery
            import time
            time.sleep(1)

            # publish start signal
            start_msg = Bool()
            start_msg.data = True
            start_pub.publish(start_msg)

            print('start signal sent')

            # execute motion
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
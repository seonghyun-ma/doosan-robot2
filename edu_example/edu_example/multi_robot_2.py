




##### Import necessary libraries (initial process)
import rclpy
from rclpy.logging import get_logger
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import GetCtrlBoxDigitalOutput
from dsr_msgs2.srv import GetCurrentPosj
import numpy as np





##### Import Doosan robot configuration module
import DR_init
DR_init.__dsr__id       = 'dsr02'
DR_init.__dsr__model    = 'm0609'





##### Create logger
logger = get_logger('multi_robot_2')

j_vel = 60
j_acc = 60
l_vel = [500,500]
l_acc = [500,500]

def main(args=None):
    print('## start ##')





    ##### Initial setup
    rclpy.init(args=args)
    node = rclpy.create_node('multi_robot_2', namespace='dsr02')
    DR_init.__dsr__node = node





    ##### Import Doosan robot operation module
    try:
        from DSR_ROBOT2 import (
            movej, movel, set_robot_mode, set_digital_output, get_digital_output, set_tool_digital_output,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        )
    except ImportError as e: print(f"Error importing DSR_ROBOT2: {e}"); return





    ##### Set robot mode
    set_robot_mode(ROBOT_MODE_AUTONOMOUS) # ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS





    ##### Create Class
    class JointStateSubscriber2(Node):
        def __init__(self, target_robot):
            super().__init__('joint_state_subscriber_2', namespace='dsr02')
            self.subscription = self.create_subscription(
                Float64MultiArray,
                f'/{target_robot}/msg/joint_state',
                self.listener_callback,
                10
            )
            self.subscription  # prevent unused variable warning
            self.received_data = None

        def listener_callback(self, msg):
            self.received_data = msg.data
            # self.get_logger().info(f'Received joint state: {self.received_data}')

    joint_state_subscriber_2 = JointStateSubscriber2('dsr01')





    ##### Define functions
    a_l_vel = [200,200]
    a_l_acc = [200,200]
    def grasp():
        print('# grasp')
        set_tool_digital_output(index=2, val=0)
        set_tool_digital_output(index=3, val=1)
        time.sleep(0.5)
        return
    def release():
        print('# release')
        set_tool_digital_output(index=2, val=1)
        set_tool_digital_output(index=3, val=0)
        time.sleep(0.5)
        return
    def pick():
        print('# pick')
        movel([0,0,-131,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) # -240
        grasp()
        movel([0,0, 131,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) #  240
        return
    def place():
        print('# place')
        movel([0,0,-115,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) # -220
        release()
        movel([0,0, 115,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) #  220
        return





    ######################## main ########################
    i = 1
    time.sleep(1)
    movej([0,   0,  90, 0, 90,0], vel=j_vel, acc=j_acc)
    release()
    time.sleep(5) # Start later (The first robot starts moving and then it starts.)

    try:
        while rclpy.ok():

            # 다른 로봇의 각도 획득
            my_robot = 'dsr02'
            target_robot = 'dsr01'

            # Obtain angles of another robot
            rclpy.spin_once(joint_state_subscriber_2, timeout_sec=0.1)
            target_joint = joint_state_subscriber_2.received_data
            
            print(f'wait... [{my_robot}] ({i}) : {target_robot} - {target_joint}'); i+=1

            # Execute when the target robot's angles are at home position (Set absolute error)
            if i>4 and target_joint is not None and np.allclose(target_joint, [ 0.,  0., 90.,  0., 90.,  0.], atol=1):
                
                # Operation cmmand
                movel([193.51, -497.32, 310, 0, 180, 0], vel=l_vel, acc=l_acc)
                pick()
                movel([460.21, -165.96, 310, 0, 180, 0], vel=l_vel, acc=l_acc) # 
                place()
                movej([0,   0,  90, 0, 90,0], vel=j_vel, acc=j_acc)

                # The part where it waits until another robot starts moving
                time.sleep(3)
                i = 1
            time.sleep(0.5)

    except KeyboardInterrupt:
        print('## Shutdown requested ##')
    rclpy.shutdown()
    print('## fin ##')

if __name__ == '__main__':
    main()




import rclpy
from rclpy.logging import get_logger
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import GetCtrlBoxDigitalOutput
from dsr_msgs2.srv import GetCurrentPosj
import numpy as np


## 두산 로봇 설정 모듈 import
import DR_init
DR_init.__dsr__id       = 'dsr01'
DR_init.__dsr__model    = 'm0609'

logger = get_logger('multi_robot_1_pp')


######################## 함수 ########################

# # ros2 service call /dsr01/aux_control/get_current_posj dsr_msgs2/srv/GetCurrentPosj "{}"
# def get_target_joint(node, target_robot): # 
#     client = node.create_client(GetCurrentPosj, f'/{target_robot}/aux_control/get_current_posj')
#     while not client.wait_for_service(timeout_sec=1.0):
#         node.get_logger().info('Service not available, waiting...')
#     req = GetCurrentPosj.Request()
#     future = client.call_async(req)
#     rclpy.spin_until_future_complete(node, future)
#     response = future.result()
#     if response is not None:
#         logger.info(f"{target_robot} posj")
#         if response.success:
#             return response.pos
#         else:
#             print("Success flag is False.")
#             return None
#     else:
#         logger.error("Service call failed")
#         # print("Failed to get signal.")
#         return None


j_vel = 60
j_acc = 60
l_vel = [500,500]
l_acc = [500,500]





def main(args=None):
    print('## start ##')

    ######################## 설정 ########################
    rclpy.init(args=args)
    node = rclpy.create_node('multi_robot_1_pp', namespace='dsr01')
    DR_init.__dsr__node = node
    try:
        from DSR_ROBOT2 import (
            movej, movel, set_robot_mode, set_digital_output, get_digital_output, set_tool_digital_output, movejx,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        )
    except ImportError as e: print(f"Error importing DSR_ROBOT2: {e}"); return
    set_robot_mode(ROBOT_MODE_AUTONOMOUS) # ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS

    class JointStateSubscriber1(Node):
        def __init__(self, target_robot):
            super().__init__('joint_state_subscriber_1', namespace='dsr01')
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

    joint_state_subscriber_1 = JointStateSubscriber1('dsr02')

    ######################## def ########################
    
    a_l_vel = [200,200]
    a_l_acc = [200,200]
    def grasp():
        print('# grasp')
        set_tool_digital_output(index=2, val=0)
        set_tool_digital_output(index=3, val=1)
        time.sleep(1)
        return
    def release():
        print('# release')
        set_tool_digital_output(index=2, val=1)
        set_tool_digital_output(index=3, val=0)
        time.sleep(1)
        return
    def pick():
        print('# pick')
        movel([0,0,-209,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) # -240
        grasp()
        movel([0,0, 209,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) #  240
        return
    def place():
        print('# place')
        movel([0,0,-200,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) # -220
        release()
        movel([0,0, 200,0,0,0], vel=a_l_vel, acc=a_l_acc, mod=1) #  220
        return
    
    



    '''
    기본
    367, 6, 294,   0,180,0
    픽 위치
    206.38, -504.71, 50.65
    플레이스 위치
    462.73, -160.25, 53.21
    
    '''

    ######################## 메인 ########################
    
    i = 1
    time.sleep(1)
    movej([0,   0,  90, 0, 90,0], vel=j_vel, acc=j_acc)
    release()
    
    try:
        while rclpy.ok():

            # 다른 로봇의 각도 획득
            my_robot = 'dsr01'
            target_robot = 'dsr02'
            # target_joint = get_target_joint(node, target_robot=target_robot)

            rclpy.spin_once(joint_state_subscriber_1, timeout_sec=0.1)
            target_joint = joint_state_subscriber_1.received_data
            
            print(f'wait... [{my_robot}] ({i}) : {target_robot} - {target_joint}'); i+=1

            # 절대 오차 설정 # 타겟 로봇의 각도가 home이면
            if i>4 and target_joint is not None and np.allclose(target_joint, [ 0.,  0., 90.,  0., 90.,  0.], atol=1):
                
                # 교육용
                movejx([197.68, -493.31, 260,   0,180,0], vel=j_vel, acc=j_acc, sol=2)
                pick()
                movejx([460.42, -159.00, 260,   0,180,0], vel=j_vel, acc=j_acc, sol=2) # 
                place()
                movej([0,   0,  90, 0, 90,0], vel=j_vel, acc=j_acc)

                # # 기존
                # movel([355,-220,340,0,180,0], vel=l_vel, acc=l_acc)
                # pick()
                # movel([668,  79,350,0,180,0], vel=l_vel, acc=l_acc) # 
                # place()
                # pick()
                # movel([355,-220,340,0,180,0], vel=l_vel, acc=l_acc)
                # place()
                # movej([0,   0,  90, 0, 90,0], vel=j_vel, acc=j_acc)

                time.sleep(3) # 다른 로봇 이동 시작까지 기다리는 부분
                i = 1
            time.sleep(0.5)


    ######################## fin ########################
    except KeyboardInterrupt:
        print('## Shutdown requested ##')
    rclpy.shutdown()
    print('## fin ##')

if __name__ == '__main__':
    main()




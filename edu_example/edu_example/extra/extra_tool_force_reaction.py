


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.logging import get_logger
import time


## 두산 로봇 설정 모듈 import
import DR_init
DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = 'm0609'

logger = get_logger('tool_force_subscriber')



def main(args=None):
    print('## start ##')

    ######################## 설정 ########################

    ## 초기 설정
    rclpy.init(args=args) # ROS2 클라이언트 초기화
    node = rclpy.create_node('force_node', namespace='dsr01') # 노드생성
    DR_init.__dsr__node = node # 두산 로봇 설정 모듈에 노드 설정    

    ## 두산 로봇 작동 모듈 임포트
    try:
        from DSR_ROBOT2 import (
            movej, movel, amovej, mwait, set_robot_mode, ROBOT_MODE_AUTONOMOUS
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    ## 로봇 모드 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS) # ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS

    ######################## 클래스 ########################

    class ToolForceSubscriber(Node):
        def __init__(self):
            super().__init__('tool_force_subscriber', namespace='dsr01')
            self.subscription = self.create_subscription(
                Float64MultiArray,
                '/dsr01/msg/tool_force',
                self.listener_callback,
                10
            )
            self.subscription  # prevent unused variable warning
            self.force_data = None

        def listener_callback(self, msg):
            self.force_data = msg.data
            # self.get_logger().info(f'Received tool force data: {self.force_data}')

    ######################## 함수 ########################

    ######################## 메인 ########################
    i=0
    tool_force_subscriber = ToolForceSubscriber()
    try:
        while rclpy.ok():

            # 토픽 데이터 확인
            rclpy.spin_once(tool_force_subscriber, timeout_sec=0.1)
            tool_force_data = tool_force_subscriber.force_data
            
            # 
            i+=1
            if tool_force_data is not None:
                x_force = tool_force_data[0]
                print(f'{i} x_force : {x_force}')
                if i > 3 and abs(x_force) > 20:
                    i=0
                    print(f'### reaction : {x_force}')
                    print()
                    # movel 명령 수행
                    movel([0, 0, 0, 0, 0, 0], vel=[100,100], acc=[100,100], mod=1) # stop
                    movej([0, 0, 0, 0, 0, 20], vel=150, acc=150, mod=1)
                    movej([0, 0, 0, 0, 0,-20], vel=150, acc=150, mod=1)

    ######################## 종료 ########################
    except KeyboardInterrupt:
        print("## Shutdown requested ##")

    
    
    
    

    rclpy.shutdown()
    print("## Node shut down ##")

    print('## fin ##')

if __name__ == '__main__':
    main()




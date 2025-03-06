import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
import time

class MoveJointClient(Node):
    def __init__(self):
        super().__init__('move_joint_client')
        self.client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /dsr01/motion/move_joint...')
        self.get_logger().info('Service /dsr01/motion/move_joint available.')

    def send_request(self, pos, vel, acc):
        request = MoveJoint.Request()
        request.pos = pos
        request.vel = vel
        request.acc = acc
        self.get_logger().info(f'Sending request: pos={pos}, vel={vel}, acc={acc}')
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response received: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveJointClient()

    # 첫 번째 요청
    node.send_request([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=50.0, acc=50.0)
    rclpy.spin_once(node, timeout_sec=0)  # 서비스 요청 대기

    # 3초 대기
    time.sleep(3)

    # 두 번째 요청
    node.send_request([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=50.0, acc=50.0)
    rclpy.spin_once(node, timeout_sec=0)  # 서비스 요청 대기

    # 종료
    time.sleep(1)  # 응답 처리 시간 확보
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
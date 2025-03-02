import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CurrentPosxSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/dsr01/msg/joint_state',
            self.listener_callback,
            10  # QoS Depth
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscribed to /dsr01/msg/joint_state')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPosxSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

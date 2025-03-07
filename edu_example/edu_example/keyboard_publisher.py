




##### Import necessary libraries (initial process)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import sys
import select
import termios
import tty





##### Create Class
class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_topic', 10)
        self.get_logger().info('Keyboard Publisher Node Initialized.')
        self.get_logger().info('q: stop')
        self.get_logger().info('w: up (+X)')
        self.get_logger().info('s: up (-X)')
        self.get_logger().info('a: up (+Y)')
        self.get_logger().info('d: up (-Y)')
        self.get_logger().info('r: up (+Z)')
        self.get_logger().info('f: up (-Z)')
        self.run()





    ##### method for message publish
    def publish_message(self, data: str):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Pub: {data}')





    ##### Input recognition and topic message publishing
    def run(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == 'q':
                        self.publish_message('stop')
                    elif key == 'w':
                        self.publish_message('+X')
                    elif key == 's':
                        self.publish_message('-X')
                    elif key == 'a':
                        self.publish_message('+Y')
                    elif key == 'd':
                        self.publish_message('-Y')
                    elif key == 'r':
                        self.publish_message('+Z')
                    elif key == 'f':
                        self.publish_message('-Z')
                    elif key == 'e':
                        self.publish_message('home')
                    elif key == 't':
                        self.publish_message('release')
                    elif key == 'g':
                        self.publish_message('grasp')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)




def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher() # Create KeyboardPublisher instance -> start "run" method
    rclpy.shutdown()


if __name__ == '__main__':
    main()









##### Import necessary libraries (initial process)
import rclpy
from rclpy.logging import get_logger
import time
from rclpy.node import Node
from std_msgs.msg import String
from dsr_msgs2.msg import JogMultiAxis


from dsr_msgs2.srv import GetCtrlBoxDigitalOutput
import numpy as np





##### Import Doosan robot configuration module
import DR_init
DR_init.__dsr__id       = 'dsr01'
DR_init.__dsr__model    = 'm0609'





##### Create logger

logger = get_logger('keyboard_control')

def main(args=None):
    print('## start ##')





    ##### Initial setup
    rclpy.init(args=args) # Initialize ROS2 client
    node = rclpy.create_node('test_node', namespace='dsr01') # Create node
    DR_init.__dsr__node = node # Set node in Doosan robot configuration module





    ##### Import Doosan robot operation module
    try:
        from DSR_ROBOT2 import (
            movej, movel, amovel, jog, set_robot_mode, set_digital_output, get_digital_output, check_motion, amovej, set_tool_digital_output,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        )
    except ImportError as e: print(f"Error importing DSR_ROBOT2: {e}"); return





    ##### Set robot mode
    set_robot_mode(ROBOT_MODE_MANUAL) # ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS





    ##### Create Class : Publisher
    class JogMultiAxisPublisher(Node):
        def __init__(self):
            super().__init__('jog_multi_axis_publisher')
            self.publisher = self.create_publisher(JogMultiAxis, '/dsr01/jog_multi', 10)

        def publish_jog(self, jog_axis, move_reference, speed):
            msg = JogMultiAxis()
            msg.jog_axis = [float(x) for x in jog_axis]
            msg.move_reference = int(move_reference)
            msg.speed = float(speed)
            self.publisher.publish(msg)
            self.get_logger().info(f'Published jog multi-axis command: {msg}')





    ##### Create Class : Subscriber
    class KeyboardSubscriber(Node):
        def __init__(self):
            super().__init__('keyboard_subscriber')
            self.subscription = self.create_subscription(
                String,
                'keyboard_topic',
                self.listener_callback,
                10
            )
            self.received_data = None
            self.new_data_received = False

        def listener_callback(self, msg):
            self.received_data = msg.data
            self.new_data_received = True
            # self.get_logger().info(f'I received: "{msg.data}"')





    ##### Define grasp & release
    def grasp():
        # print('# grasp')
        set_tool_digital_output(index=2, val=0)
        set_tool_digital_output(index=3, val=1)
        return
    def release():
        # print('# release')
        set_tool_digital_output(index=2, val=1)
        set_tool_digital_output(index=3, val=0)





    ######################## main ########################
    previous_value = ''
    keyboard_subscriber = KeyboardSubscriber()
    jog_multi_axis_publisher = JogMultiAxisPublisher()

    movej([0,0,90,0,90,0], vel=100, acc=100)
    try: 
        while rclpy.ok():
            rclpy.spin_once(keyboard_subscriber, timeout_sec=0.1)

            # Execute only when new data is entered
            if keyboard_subscriber.new_data_received:
                keyboard_subscriber.new_data_received = False
                key_value = keyboard_subscriber.received_data
                print(f'key_value : {key_value}')
                
                # Only if the input value is different from the previous one
                if previous_value != key_value : 
                    if   key_value == '+X':
                        print(key_value)
                        jog_multi_axis_publisher.publish_jog([1,0,0,0,0,0], 0, 50)

                    elif key_value == '-X':
                        print(key_value)
                        jog_multi_axis_publisher.publish_jog([-1,0,0,0,0,0], 0, 50)
                        
                    elif key_value == '+Y':
                        print(key_value)
                        jog_multi_axis_publisher.publish_jog([0,1,0,0,0,0], 0, 50)
                        
                    elif key_value == '-Y':
                        print(key_value)
                        jog_multi_axis_publisher.publish_jog([0,-1,0,0,0,0], 0, 50)
                        
                    elif key_value == '+Z':
                        print(key_value)
                        jog_multi_axis_publisher.publish_jog([0,0,1,0,0,0], 0, 50)
                        
                    elif key_value == '-Z':
                        print(key_value)
                        jog_multi_axis_publisher.publish_jog([0,0,-1,0,0,0], 0, 50)
                    
                    elif key_value == 'stop':
                        print(key_value)
                        jog_multi_axis_publisher.publish_jog([0,0,0,0,0,0], 0, 50)
                        
                    elif key_value == 'home':
                        jog_multi_axis_publisher.publish_jog([0,0,0,0,0,0], 0, 20)
                        time.sleep(1)
                        print(key_value); movej([0,0,90,0,90,0], vel=100, acc=100)

                    elif key_value == 'release':
                        print(key_value); release()

                    elif key_value == 'grasp':
                        print(key_value); grasp()

                    previous_value = key_value

    except KeyboardInterrupt: print("## Shutdown requested ##")






    keyboard_subscriber.destroy_node()
    rclpy.shutdown()
    print("## Node shut down ##")
    print('## fin ##')

if __name__ == '__main__':
    main()
##### Import necessary libraries (initial process)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.logging import get_logger
import time



##### Import Doosan robot configuration module
import DR_init
DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = 'm0609'



##### Create logger
logger = get_logger('test_teaching_node')

def main(args=None):
    print('## start ##')


    ##### Initial setup
    rclpy.init(args=args) # Initialize ROS2 client
    node = rclpy.create_node('test_teaching_node', namespace='dsr01') # Create node
    DR_init.__dsr__node = node # Set node in Doosan robot configuration module





    ##### Import Doosan robot operation module
    try:
        from DSR_ROBOT2 import (
            movej, movel, amovej, mwait, set_robot_mode, 
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_tool_digital_output,
            get_current_posx, get_digital_input, wait
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return





    ##### Set robot mode
    set_robot_mode(ROBOT_MODE_AUTONOMOUS) # ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS






    ##### Define grasp & release
    def grasp():
        print('# grasp')
        set_tool_digital_output(index=4, val=0)
        set_tool_digital_output(index=5, val=1)
        return
    def release():
        print('# release')
        set_tool_digital_output(index=4, val=1)
        set_tool_digital_output(index=5, val=0)





    ######################## main ########################
    i=0
    j=0
    try:
        while rclpy.ok():
        
            pos_list = []
            sol_list = []
            j_vel = 60
            j_acc = 60
            a_l_vel = [300,300]
            a_l_acc = [300,300]
            
            movej([0, 0, 90, 0, 90,0], vel=j_vel, acc=j_acc)
            set_robot_mode(ROBOT_MODE_MANUAL)

            print('\nDigital Input 5 = save pose')
            print('Digital Input 6 = wait run program after saving pose')
            print('Digital Input 7 = run program\n')
            print("let's save pose\n")

            while True:
                if get_digital_input(5) == 1:
                    a, b = get_current_posx()

                    pos_list.append(a)
                    sol_list.append(b)

                    print(f"pos save {len(pos_list)}")

                    while get_digital_input(5) == 1:
                        pass

                if get_digital_input(6) == 1:
                    break

            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            movej([0, 0, 90, 0, 90,0], vel=j_vel, acc=j_acc)
            print('\nready')

            while get_digital_input(7) == 0:
                pass

            print('\nmoving start')

            for pos_line in pos_list:
                movel(pos_line, vel=a_l_vel, acc=a_l_acc)

            movej([0, 0, 90, 0, 90,0], vel=j_vel, acc=j_acc)
            
            break
            

    except KeyboardInterrupt:
        print("## Shutdown requested ##")






    rclpy.shutdown()
    print("## Node shut down ##")
    print('## fin ##')

if __name__ == '__main__':
    main()




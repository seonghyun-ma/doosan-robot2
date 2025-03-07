
##### Import necessary libraries (initial process)

import rclpy
from rclpy.node import Node
import os
import sys
import argparse
from rclpy.logging import get_logger
import random
import time
import threading

from dsr_msgs2.srv import Fkin
from sensor_msgs.msg import JointState
import numpy as np






##### Create logger

logger = get_logger('test_function')

def main(args=None):






    ##### Enter arguments
    parser = argparse.ArgumentParser(description='Test Function')
    parser.add_argument('--name',     type=str, default="dsr01",  help='ID of the robot')
    parser.add_argument('--model',    type=str, default="m0609",  help='Model of the robot')
    parser.add_argument('--run_mode', type=str, default="test", help='Run mode of the robot')
    parsed_args = parser.parse_args(args=args)

    '''
    ### run_mode
    
    test_1
    test_2
    ikin
    fkin
    COS
    TCPTool
    axis
    coord
    getIO
    setDO
    move_j
    move_jx
    move_l
    move_c
    move_sj
    move_sx
    move_b
    move_spiral
    move_periodic
    move_home
    '''

    ROBOT_ID       = parsed_args.name
    ROBOT_MODEL    = parsed_args.model
    ROBOT_RUN_MODE = parsed_args.run_mode






    ##### Import Doosan robot configuration module
    import DR_init
    DR_init.__dsr__id       = ROBOT_ID
    DR_init.__dsr__model    = ROBOT_MODEL






    ##### Initial setup
    rclpy.init(args=args) # Initialize ROS2 client
    node = rclpy.create_node('test_function', namespace=ROBOT_ID) # Create node
    DR_init.__dsr__node = node # Set node in Doosan robot configuration module






    ##### Import Doosan robot operation module
    try:

        from DSR_ROBOT2 import (

            # motion
            movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb,
            move_home,trans,ikin,fkin,set_ref_coord,change_operation_speed,jog,jog_multi,
            enable_alter_motion,alter_motion,disable_alter_motion,set_singular_handling,
            
            # system
            get_last_alarm,get_robot_state,set_robot_system,get_robot_system,set_robot_mode,get_robot_mode,
            set_robot_speed_mode,get_robot_speed_mode,get_current_pose,set_safe_stop_reset_type,
            
            # drl
            get_drl_state,check_motion,drl_script_run,drl_script_pause,drl_script_resume,drl_script_stop,
            
            # aux_control
            get_current_posj,get_current_velj,get_desired_posj,get_desired_velj,
            get_current_posx,get_current_velx,get_desired_posx,get_desired_velx,
            get_current_solution_space,get_solution_space,get_current_tool_flange_posx,get_orientation_error,
            get_control_mode,get_control_space,get_current_rotm,get_joint_torque,get_external_torque,get_tool_force,
            
            # tcp, tool
            get_tcp,add_tcp,set_tcp,del_tcp,add_tool,set_tool,get_tool,del_tool,set_tool_shape,
            
            # force
            parallel_axis,align_axis,set_user_cart_coord,overwrite_user_cart_coord,get_user_cart_coord,task_compliance_ctrl,
            set_desired_force,release_force,release_compliance_ctrl,coord_transform,get_workpiece_weight,reset_workpiece_weight,
            check_position_condition,check_force_condition,check_orientation_condition,is_done_bolt_tightening,
            
            # io
            get_digital_input,get_digital_output,get_tool_digital_input,get_tool_digital_output,get_analog_input,
            set_digital_output,set_tool_digital_output,set_mode_analog_input,set_mode_analog_output,set_analog_output,

            # etc
            posj, posx, posb,
            DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS,
            ROBOT_MODE_MANUAL, ROBOT_MODE_MEASURE, DR_AVOID, DR_TASK_STOP, DR_VAR_VEL, DR_COND_NONE, DR_MV_MOD_REL
        )        
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return






    ##### Set robot mode
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    # ROBOT_MODE_MANUAL                     = 0
    # ROBOT_MODE_AUTONOMOUS                 = 1
    # ROBOT_MODE_MEASURE                    = 2






    ##### Set speed and acceleration
    d_vel = 30; d_velx = [200, 200]
    d_acc = 30; d_accx = [200, 200]






    ##### Specify initial position
    zero = posj(   0,   0,   0,   0,   0,   0)
    home = posj(   0,   0,  90,   0,  90,   0)
    
    ## Set DRL parameters
    RorV = 0 # 0: Real, 1: Virtual
    cus_code = """\
    movej(posj(   0,   0,  90,   0,  90,   0), vel= 60, acc= 60); 
    movel(posx( 400,   0, 500,   0, 180,   0), vel=120, acc=120); 
    movel(posx( 400, 200, 500,   0, 180,   0), vel=120, acc=120); 
    movel(posx( 400,   0, 500,   0, 180,   0), vel=120, acc=120); 
    movej(posj(   0,   0,  90,   0,  90,   0), vel= 60, acc= 60)
    """






    ##### IO show function
    def get_all_IO():
        print('### ControlBox Digital Input ###')
        for index in range(1,17): print(f'| CDI_{index}:', get_digital_input(index), end=' ')
        print(); print()
        print('### ControlBox Digital Output ###')
        for index in range(1,17): print(f'| CDO_{index}:', get_digital_output(index), end=' ')
        print(); print()
        print('### Tool Digital Input ###')
        for index in range(1,7): print(f'| TDI_{index}:', get_tool_digital_input(index), end=' ')
        print(); print()
        print('### Tool Digital Output ###')
        for index in range(1,7): print(f'| TDO_{index}:', get_tool_digital_output(index), end=' ')
        print(); print()
        print('### Analog Input ###')
        for ch in range(1,3): print(f'| CH_{ch}:', get_analog_input(ch), end=' ')
        print(); print()






    ###### test_1
    if ROBOT_RUN_MODE == 'test_1': # joint Coordinate (move joint)
        print(); print('##### Start (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('########################'); print()

        print('test')

        print(); print('##### Fin (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('######################'); print(); print(); print(); print(); print(); print()

        rclpy.shutdown()
        return
    



    ###### test_2
    if ROBOT_RUN_MODE == 'test_2':
        set_robot_mode(robot_mode=0) # manual mode
        print(f'get_robot_mode() = {get_robot_mode()} (manual)')
        print('posj')
        print(f'axis_1 : {get_current_posj()[0]}')
        print(f'axis_2 : {get_current_posj()[1]}')
        print(f'axis_3 : {get_current_posj()[2]}')
        print(f'axis_4 : {get_current_posj()[3]}')
        print(f'axis_5 : {get_current_posj()[4]}')
        print(f'axis_6 : {get_current_posj()[5]}')
        print('posx')
        print(f'x     : {get_current_posx()[0][0]}')
        print(f'y     : {get_current_posx()[0][1]}')
        print(f'z     : {get_current_posx()[0][2]}')
        rclpy.shutdown()
        return







    ############################## main ##############################

    ############# Start #############
    print(); print('##### Start #####')
    print('move home'); movej(home, vel=d_vel, acc=d_acc)
    print('#################'); print()
    

    
    ############# ikin ############# # task -> joint
    if ROBOT_RUN_MODE == 'ikin':
        pos = posx(559.0, 34.5, 651.5, 45.0, 180.0, 45.0); sol_space = 2
        print('ikin')
        print(np.round(ikin(pos, sol_space, ref=DR_BASE), 2) )



    ############# fkin ############# # joint -> task
    elif ROBOT_RUN_MODE == 'fkin':
        pos = posj(367.38607788, 4.22731638, 322.96060181, 90.14829254, 179.98649597, 90.4562912 )
        print('fkin')
        print(np.round(fkin(pos, ref=DR_BASE)) )



    ############# change_operation_speed (speed) (time measurement) #############
    elif ROBOT_RUN_MODE == 'COS':
        # define position
        x0 = posx( 400, 200, 310,   0, 180,   0)
        x1 = posx( 400,-200, 310,   0, 180,   0)

        # initial
        print('# inital pose')
        movejx(x1, vel=d_vel, acc=d_acc, sol=2)
        print()

        # low speed  (virtual : 12.3s / real : 11.48s)
        print('# speed = 50')
        change_operation_speed(50)
        low_speed_1 = time.time()
        movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        movejx(x1, vel=d_vel, acc=d_acc, sol=2)
        low_speed_2 = time.time()

        # high speed  (virtual : 6.27s / real : 5.94s)
        print('# speed = 100')
        change_operation_speed(100)
        high_speed_1 = time.time()
        movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        movejx(x1, vel=d_vel, acc=d_acc, sol=2)
        high_speed_2 = time.time()

        # time measurement
        print(f'low_speed_time = {np.round(low_speed_2-low_speed_1, 2)}s')
        print()
        print(f'high_speed_time = {np.round(high_speed_2-high_speed_1, 2)}s')
        print()



    ############# TCPTool #############
    elif ROBOT_RUN_MODE == 'TCPTool':
        name = 'test_tcp'
        pos = [0,0,0,0,0,0]
        print('get_tcp()'); print(get_tcp()); print()
        print('add_tcp(name, pos)'); add_tcp(name, pos); print(get_tcp()); print()
        print('set_tcp(name)'); set_tcp(name); print(get_tcp()); print()
        print('del_tcp(name)'); del_tcp(name); print(get_tcp()); print()

        name = 'test_tool'
        print('get_tool()'); print(get_tool()); print()
        print('add_tool'); add_tool(name, weight=10, cog=[0,0,0], inertia=[0,0,0,0,0,0]); print(get_tool()); print()
        print('set_tool(name)'); set_tool(name); print(get_tool()); print()
        print('del_tool(name)'); del_tool(name); print(get_tool()); print()
        print('set_tool_shape(name)'); set_tool_shape(name); print(get_tool()); print()



    ############# Axis example #############
    elif ROBOT_RUN_MODE == 'axis':
        # Manual mode conversion required
        set_robot_mode(robot_mode=0) # manual
        print(f'get_robot_mode() = {get_robot_mode()} (manual)')
        a_axis = DR_AXIS_Z
        a_ref  = DR_BASE

        ## initial pos : show normal vector direction
        up_pos = posx( 300,-100, 400,   0, 180,   0)
        dn_pos = posx( 300,   0, 300,   0, 180,   0)
        print('1 movejx'); movejx(up_pos, vel=d_vel, acc=d_acc, sol=2)
        print('2 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)

        ## ParallelAxis1 : operation error occurred (stop issue)
        x1     = [ 100,-100,   0,   0,   0,   0]
        x2     = [-100,-100,   0,   0,   0,   0]
        x3     = [   0,   0, 100,   0,   0,   0]
        print('3 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('4 parallel_axis_1'); parallel_axis(x1=x1, x2=x2, x3=x3, axis=a_axis, ref=a_ref) # len_() == 5 # _nType = 2
        time.sleep(1)
        
        ## ParallelAxis2
        n_vect = [   0,   1,  -1]
        movej([0,0,90,0,90,0], vel=30, acc=30)
        print('5 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('6 parallel_axis_2'); parallel_axis(vect=n_vect, axis=a_axis, ref=a_ref) # len_() == 3 # _nType = 3

        ## AlignAxis1 : operation error occurred (stop issue)
        s_vect = [ 300, 0, 300]
        print('7 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('8 align_axis_1'); align_axis(x1=x1, x2=x2, x3=x3, pos=s_vect, axis=a_axis, ref=a_ref) # len_() == 6 # _nType == 2
        time.sleep(1)

        ## AlignAxis2
        t_vect = n_vect
        movej([0,0,90,0,90,0], vel=30, acc=30)
        print('9 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('10 align_axis_2'); align_axis(vect=t_vect, pos=s_vect, axis=a_axis, ref=a_ref)# len_() == 4 # _nType == 3



    ############# Coordinate example #############
    elif ROBOT_RUN_MODE == 'coord':

        ## Coordinate Check Operation 1: Move along each axis of the specified coordinate system
        def show_coord(id):
            movejx([0,0,0,0,0,0], vel=d_vel+100, acc=d_acc+100, sol=2, ref=id)
            xlist = [
                posx( 0, 0, 0,0,0,0),
                posx(50, 0, 0,0,0,0),
                posx( 0, 0, 0,0,0,0),
                posx( 0,50, 0,0,0,0),
                posx( 0, 0, 0,0,0,0),
                posx( 0, 0,50,0,0,0),
                posx( 0, 0, 0,0,0,0)]
            movesx(xlist, vel=1000, acc=1000, ref=id)

        ## Coordinate Check Operation 2: Move to the origin points of the specified coordinate systems
        def show_all_coord(id_list, txt):
            print(); print('txt')
            for id in id_list:
                print(f'coord : {id}'); movejx(posx(0,0,0,0,0,0), vel=d_vel+100, acc=d_acc+100, sol=2, ref=id)

        c_axis = DR_AXIS_Z
        c_ref  = DR_BASE
    
        ## Default positions (origin points of 4 coordinate systems)
        cooro_1 = [ 400,   0, 300,  90, 135,  90] # 367.5,   0, 311,   0, 180,   0
        cooro_2 = [ 400,   0, 300,   0, 135,   0]
        cooro_3 = [ 400,   0, 300, -90, 135, -90]
        cooro_4 = [ 400,   0, 300, 180, 135, 180]

        # ## Operation for checking default positions
        # movejx(cooro_1, vel=d_vel, acc=d_acc, sol=2)
        # movejx(cooro_2, vel=d_vel, acc=d_acc, sol=2)
        # movejx(cooro_3, vel=d_vel, acc=d_acc, sol=2)
        # movejx(cooro_4, vel=d_vel, acc=d_acc, sol=2)

        ## SetUserCartCoord1
        id1 = set_user_cart_coord(pos=cooro_1, ref=c_ref)
        print(f'SetUserCartCoord1 = {id1}'); show_coord(id1)

        ## SetUserCartCoord2: Operates normally only in real mode
        x1 = [  0,  0,  0,  0,  0,  0]
        x2 = [  0,100,  0,  0,  0,  0]
        x3 = [100,  0,100,  0,  0,  0]
        id2 = set_user_cart_coord(x1=x1, x2=x2, x3=x3, pos=cooro_2, ref=c_ref)
        print(f'SetUserCartCoord2 = {id2}'); show_coord(id2)

        ## SetUserCartCoord3
        u1 = [-1, 0, 0]
        v1 = [ 0, 1,-1]
        id3 = set_user_cart_coord(u1=u1, v1=v1, pos=cooro_3, ref=c_ref)
        print(f'SetUserCartCoord3 = {id3}'); show_coord(id3)

        ## Check all coordinate systems
        show_all_coord([id1, id2, id3], 'all coord')

        ## Overwrite coordinate system
        overwrite_user_cart_coord(id=id2, pos=cooro_4, ref=DR_BASE)

        ## Check all coordinate systems
        show_all_coord([id1, id2, id3], 'all coord (replaced)')



    ############# example : get IO signal #############
    elif ROBOT_RUN_MODE == 'getIO':
        get_all_IO()



    ############# example : Set DIO signal (index = 2, 3) #############
    elif ROBOT_RUN_MODE == 'setDO':

        # set signal (on)
        set_digital_output(index=4, val=1)
        set_tool_digital_output(index=3, val=1)
        time.sleep(1)

        # check signal (on)
        print(f'ControlBox Digital Output : {get_digital_output(index=4)}')
        print(f'      Tool Digital Output : {get_tool_digital_output(index=3)}')
        print()

        # set signal (off)
        set_digital_output(index=4, val=0)
        set_tool_digital_output(index=3, val=0)
        time.sleep(1)

        # check signal (off)
        print(f'ControlBox Digital Output : {get_digital_output(index=4)}')
        print(f'      Tool Digital Output : {get_tool_digital_output(index=3)}')
        print()




    ############################## Basic operation test ##############################



    ############# move_j #############
    elif ROBOT_RUN_MODE == 'move_j':
        p0 = posj(   0, -20, 110,   0,  90,   0)
        p1 = posj(   0,  20,  70,   0,  90,   0)
        p2 = posj(   0,   0,  90,   0,  90,   0)
        i = 0; current_time = 0
        print('move j 1'); movej(p0, vel=d_vel, acc=d_acc)
        print('move j 2'); movej(p1, vel=d_vel, acc=d_acc)
        print('move j 3'); movej(p2, vel=d_vel, acc=d_acc)



    ############# move_jx #############
    elif ROBOT_RUN_MODE == 'move_jx':
        x0 = posx( 300,  150, 300,   0, 180,   0)
        x1 = posx( 300, -150, 300,   0, 180,   0)
        x2 = posx( 500, -150, 300,   0, 180,   0)
        x3 = posx( 500,  150, 300,   0, 180,   0)
        print('move jx 1'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 2'); movejx(x1, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 3'); movejx(x2, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 4'); movejx(x3, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 5'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)



    ############# move_l #############
    elif ROBOT_RUN_MODE == 'move_l':
        x0 = posx( 300,  150, 300,   0, 180,   0)
        x1 = posx( 300, -150, 300,   0, 180,   0)
        x2 = posx( 500, -150, 300,   0, 180,   0)
        x3 = posx( 500,  150, 300,   0, 180,   0)
        print('move l 1'); movel(x0, vel=d_velx, acc=d_accx)
        print('move l 2'); movel(x1, vel=d_velx, acc=d_accx)
        print('move l 3'); movel(x2, vel=d_velx, acc=d_accx)
        print('move l 4'); movel(x3, vel=d_velx, acc=d_accx)
        print('move l 5'); movel(x0, vel=d_velx, acc=d_accx)



    ############# move_c #############
    elif ROBOT_RUN_MODE == 'move_c':
        x0 = posx( 400,   0, 500,   0, 180,   0)
        c1 = posx( 500,-100, 500,   0, 180,   0)
        c2 = posx( 600,   0, 500,   0, 180,   0)
        c3 = posx( 500, 100, 500,   0, 180,   0)
        print('move jx'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('move c 1'); movec(c1, c2, d_velx, d_accx)
        print('move c 2'); movec(c3, x0, d_velx, d_accx)



    ############# move_sj #############
    elif ROBOT_RUN_MODE == 'move_sj':
        q0 = posj(   0,   0, 120,   0,   0,   0)
        q1 = posj(  30,   0, 120,   0,   0,   0)
        q2 = posj(  30,   0,  90,   0,  90,   0)
        q3 = posj(   0,   0,  90,   0,  90,   0)
        qlist = [q0, q1, q2, q3]
        print('move sj'); movesj(qlist, vel=d_vel, acc=d_acc, time=10)



    ############# move_sx #############
    elif ROBOT_RUN_MODE == 'move_sx':
        x0 = posx( 300,  150, 300,   0, 180,   0)
        x1 = posx( 300, -150, 300,   0, 180,   0)
        x2 = posx( 500, -150, 300,   0, 180,   0)
        x3 = posx( 500,  150, 300,   0, 180,   0)
        xlist = [x0, x1, x2, x3, x0]
        print('move sx'); movesx(xlist, vel=300, acc=300)



    ############# move_b #############
    elif ROBOT_RUN_MODE == 'move_b':
        x0 = posx( 600,-100, 500,   0, 180,   0) # position
        l0 = posx( 600, 100, 500,   0, 180,   0)
        l1 = posx( 500, 100, 500,   0, 180,   0)
        c1 = posx( 400, 100, 500,   0, 180,   0)
        c2 = posx( 400,   0, 500,   0, 180,   0)
        l2 = posx( 400,-100, 500,   0, 180,   0)
        l3 = posx( 600,-100, 500,   0, 180,   0)
        b0 = posb(DR_LINE, l0, radius=30) # define
        b1 = posb(DR_LINE, l1, radius=30)
        cc = posb(DR_CIRCLE, c1, c2, radius=30)
        b2 = posb(DR_LINE, l2, radius=30)
        b3 = posb(DR_LINE, l3, radius=30)
        b_list = [b0, b1, cc, b2, b3] # list
        print('move jx'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('move b'); moveb(b_list, vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)



    ############# move_spiral #############
    elif ROBOT_RUN_MODE == 'move_spiral':
        x0 = posx( 500,   0, 500,   0, 180,   0)
        print('move jx'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('move_spiral'); move_spiral(rev=2, rmax=100, lmax=100, vel=[60,60], acc=[60,60], time=10, axis=DR_AXIS_Z, ref=DR_TOOL) # DR_BASE



    ############# move_periodic #############
    elif ROBOT_RUN_MODE == 'move_periodic':
        x0 = posx(367.5,   0, 311,   0, 180,   0)
        print('## BASE'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('X_trans | amp =[30, 0, 0, 0, 0, 0] '); move_periodic(amp =[30, 0, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('X_rot   | amp =[ 0, 0, 0,10, 0, 0] '); move_periodic(amp =[ 0, 0, 0,10, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)



    ############# move_home #############
    elif ROBOT_RUN_MODE == 'move_home':
        set_robot_mode(robot_mode=0) # manual
        print('move_home 1'); move_home(target=0); time.sleep(5); print('1 fin')
        print('move_home 2'); move_home(target=1); time.sleep(5); print('2 fin')



    # 종료
    print(); print('##### Fin (test) #####')
    print('move home'); movej(home, vel=d_vel, acc=d_acc)
    print('######################'); print()

    rclpy.shutdown()


if __name__ == '__main__':
    main()



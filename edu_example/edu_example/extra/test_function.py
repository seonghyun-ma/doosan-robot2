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

logger = get_logger('test_function')

def main(args=None):

    ## 인자 입력
    parser = argparse.ArgumentParser(description='Test Function')
    parser.add_argument('--name',     type=str, default="dsr01",  help='ID of the robot')
    parser.add_argument('--model',    type=str, default="m0609",  help='Model of the robot')
    parser.add_argument('--run_mode', type=str, default="test", help='Run mode of the robot')
    parsed_args = parser.parse_args(args=args)

    ROBOT_ID       = parsed_args.name
    ROBOT_MODEL    = parsed_args.model
    ROBOT_RUN_MODE = parsed_args.run_mode

    ## 두산 로봇 설정 모듈 import
    import DR_init
    DR_init.__dsr__id       = ROBOT_ID
    DR_init.__dsr__model    = ROBOT_MODEL

    ## 초기 설정
    rclpy.init(args=args) # ROS2 클라이언트 초기화
    node = rclpy.create_node('test_function', namespace=ROBOT_ID) # 노드생성
    DR_init.__dsr__node = node # 두산 로봇 설정 모듈에 노드 설정

    ## 두산 로봇 작동 모듈 임포트
    try:
        # # Existing
        # from DSR_ROBOT2 import (
        #     print_ext_result, set_velx, set_accx, set_robot_mode,
        #     movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb,
        #     posj, posx, posb,
        #     DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS,
        #     ROBOT_MODE_AUTONOMOUS)
        # New
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

            posj, posx, posb,
            DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS,
            ROBOT_MODE_MANUAL, ROBOT_MODE_MEASURE, DR_AVOID, DR_TASK_STOP, DR_VAR_VEL, DR_COND_NONE, DR_MV_MOD_REL
        )        
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    ## 로봇 모드 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    # ROBOT_MODE_MANUAL                     = 0
    # ROBOT_MODE_AUTONOMOUS                 = 1
    # ROBOT_MODE_MEASURE                    = 2

    ## 속도, 가속도 지정
    d_vel = 30; d_velx = [200, 200]
    d_acc = 30; d_accx = [200, 200]

    ## 위치 지정
    zero = posj(   0,   0,   0,   0,   0,   0)
    home = posj(   0,   0,  90,   0,  90,   0)
    
    ## DRL 파라미터 설정
    RorV = 0 # 0: Real, 1: Virtual
    cus_code = """\
    movej(posj(   0,   0,  90,   0,  90,   0), vel= 60, acc= 60); 
    movel(posx( 400,   0, 500,   0, 180,   0), vel=120, acc=120); 
    movel(posx( 400, 200, 500,   0, 180,   0), vel=120, acc=120); 
    movel(posx( 400,   0, 500,   0, 180,   0), vel=120, acc=120); 
    movej(posj(   0,   0,  90,   0,  90,   0), vel= 60, acc= 60)
    """

    ## IO show function
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

    ## 작업 공간 메모
    # ############ m0609 ############
    # x1 = posx(400, -200, 500,   0, 180,   0)
    # x2 = posx(700, -200, 500,   0, 180,   0)
    # x3 = posx(700,  200, 500,   0, 180,   0)
    # x4 = posx(400,  200, 500,   0, 180,   0)

    ## 기능 테스트용
    if ROBOT_RUN_MODE == 'test': # 조인트좌표계, 조인트무브 
        print(); print('##### Start (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('########################'); print()

        print('test')

        print(); print('##### Fin (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('######################'); print(); print(); print(); print(); print(); print()

        rclpy.shutdown()
        return
    






    
    if ROBOT_RUN_MODE == 'rpyeul1':

        ############# move_periodic #############
        print(); print('##### Start (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('########################'); print()

        # x0 = posx( 500,   0, 600,   0, 180,   0)
        # movejx(x0, vel=d_vel, acc=d_acc, sol=2)

        # 가상 모드 1 : 실제에서는 이게 맞음
        print('move_periodic 의 기준 회전축 확인'); print()
        print('순서를 [Tx, Ty, Tz, Rx, Ry, Rz] 로 가정하고 진행할 경우'); print()
        print('## BASE'); 
        print('X_trans | amp =[30, 0, 0, 0, 0, 0] '); move_periodic(amp =[30, 0, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('X_rot   | amp =[ 0, 0, 0,10, 0, 0] '); print('Rx 가 아닌 "Rz"로 적용')
        move_periodic(amp =[ 0, 0, 0,10, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('Y_trans | amp =[ 0,30, 0, 0, 0, 0] '); move_periodic(amp =[ 0,30, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('Y_rot   | amp =[ 0, 0, 0, 0,10, 0] '); move_periodic(amp =[ 0, 0, 0, 0,10, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('Z_trans | amp =[ 0, 0,30, 0, 0, 0] '); move_periodic(amp =[ 0, 0,30, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('Z_rot   | amp =[ 0, 0, 0, 0, 0,10] '); print('Rz 가 아닌 "Rx"로 적용')
        move_periodic(amp =[ 0, 0, 0, 0, 0,10], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('## TOOL')
        print('X_trans | amp =[30, 0, 0, 0, 0, 0] '); move_periodic(amp =[30, 0, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('X_rot   | amp =[ 0, 0, 0,10, 0, 0] '); print('Rx 가 아닌 "Rz"로 적용')
        move_periodic(amp =[ 0, 0, 0,10, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()
        print('Y_trans | amp =[ 0,30, 0, 0, 0, 0] '); move_periodic(amp =[ 0,30, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('Y_rot   | amp =[ 0, 0, 0, 0,10, 0] '); move_periodic(amp =[ 0, 0, 0, 0,10, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()
        print('Z_trans | amp =[ 0, 0,30, 0, 0, 0] '); move_periodic(amp =[ 0, 0,30, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('Z_rot   | amp =[ 0, 0, 0, 0, 0,10] '); print('Rz 가 아닌 "Rx"로 적용')
        move_periodic(amp =[ 0, 0, 0, 0, 0,10], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()


        if get_robot_system() == 0: # 실제면
            print('실제 모드 작동 중')
            rclpy.shutdown()
            return
 
        # 가상 모드 2 : 가상에서는 이게 맞음
        print('순서를 [Tx, Ty, Tz, Rz, Ry, Rx] 로 가정하고 진행할 경우 (Rx, Rz 의 순서를 변경)'); print()
        print('## BASE')
        print('X_trans | amp =[30, 0, 0, 0, 0, 0] '); move_periodic(amp =[30, 0, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('X_rot   | amp =[ 0, 0, 0, 0, 0,10] '); move_periodic(amp =[ 0, 0, 0, 0, 0,10], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('Y_trans | amp =[ 0,30, 0, 0, 0, 0] '); move_periodic(amp =[ 0,30, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('Y_rot   | amp =[ 0, 0, 0, 0,10, 0] '); move_periodic(amp =[ 0, 0, 0, 0,10, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('Z_trans | amp =[ 0, 0,30, 0, 0, 0] '); move_periodic(amp =[ 0, 0,30, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('Z_rot   | amp =[ 0, 0, 0,10, 0, 0] '); move_periodic(amp =[ 0, 0, 0,10, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('## TOOL')
        print('X_trans | amp =[30, 0, 0, 0, 0, 0] '); move_periodic(amp =[30, 0, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('X_rot   | amp =[ 0, 0, 0, 0, 0,10] '); move_periodic(amp =[ 0, 0, 0, 0, 0,10], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()
        print('Y_trans | amp =[ 0,30, 0, 0, 0, 0] '); move_periodic(amp =[ 0,30, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('Y_rot   | amp =[ 0, 0, 0, 0,10, 0] '); move_periodic(amp =[ 0, 0, 0, 0,10, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()
        print('Z_trans | amp =[ 0, 0,30, 0, 0, 0] '); move_periodic(amp =[ 0, 0,30, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('Z_rot   | amp =[ 0, 0, 0,10, 0, 0] '); move_periodic(amp =[ 0, 0, 0,10, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()

        print(); print('##### Fin (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('######################'); print(); print(); print(); print(); print(); print()

        rclpy.shutdown()
        return

    if ROBOT_RUN_MODE == 'rpyeul2':

        ############# line jx #############
        print(); print('##### Start (test) #####')
        print('move home'); movej([0,0,90,0,-90,0], vel=d_vel+100, acc=d_acc+100)
        print('########################'); print()

        x0 = posx( 500,   0, 700,   0,   0,   0)
        x1 = posx( 500,   0, 700,  45,   0,   0)
        x2 = posx( 500,   0, 700,   0,  45,   0)
        x3 = posx( 500,   0, 700,   0,   0,  45)
        
        print('##### move line')
        print('move l 1  [ 0,   0,   0]'); movel(x0, vel=[100,100], acc=[100,100])
        print('move l 2  [45,   0,   0]'); movel(x1, vel=[100,100], acc=[100,100])
        print('move l 3  [ 0,  45,   0]'); movel(x2, vel=[100,100], acc=[100,100])
        print('move l 4  [ 0,   0,  45]'); movel(x3, vel=[100,100], acc=[100,100])

        print('##### move jx')
        print('move jx 1 [ 0,   0,   0]'); movejx(x0, vel=d_vel, acc=d_acc, sol=3)
        print('move jx 2 [45,   0,   0]'); movejx(x1, vel=d_vel, acc=d_acc, sol=3)
        print('move jx 3 [ 0,  45,   0]'); movejx(x2, vel=d_vel, acc=d_acc, sol=3)
        print('move jx 4 [ 0,   0,  45]'); movejx(x3, vel=d_vel, acc=d_acc, sol=3)
        

        print(); print('##### Fin (test) #####')
        print('move home'); movej([0,0,90,0,-90,0], vel=d_vel+100, acc=d_acc+100)
        print('######################'); print(); print(); print(); print(); print(); print()

        rclpy.shutdown()
        return


    if ROBOT_RUN_MODE == 'jog':

        def jog_test(): # jog_axis=0 int8
            print(' 0'); jog(jog_axis= 0, ref=0, speed= 100); time.sleep(1); jog(jog_axis= 0, ref=0, speed=0)
            print(' 1'); jog(jog_axis= 1, ref=0, speed=-100); time.sleep(1); jog(jog_axis= 1, ref=0, speed=0)
            print(' 2'); jog(jog_axis= 2, ref=0, speed=-100); time.sleep(1); jog(jog_axis= 2, ref=0, speed=0)
            print(' 3'); jog(jog_axis= 3, ref=0, speed= 100); time.sleep(1); jog(jog_axis= 3, ref=0, speed=0)
            print(' 4'); jog(jog_axis= 4, ref=0, speed=-100); time.sleep(1); jog(jog_axis= 4, ref=0, speed=0)
            print(' 5'); jog(jog_axis= 5, ref=0, speed= 100); time.sleep(1); jog(jog_axis= 5, ref=0, speed=0); time.sleep(1)
            movej(home, vel=d_vel+100, acc=d_acc+100)
            print(' 6'); jog(jog_axis= 6, ref=0, speed=  30); time.sleep(1); jog(jog_axis= 6, ref=0, speed=0)
            print(' 7'); jog(jog_axis= 7, ref=0, speed=  30); time.sleep(1); jog(jog_axis= 7, ref=0, speed=0)
            print(' 8'); jog(jog_axis= 8, ref=0, speed=  30); time.sleep(1); jog(jog_axis= 8, ref=0, speed=0)
            print(' 9'); jog(jog_axis= 9, ref=0, speed=  30); time.sleep(1); jog(jog_axis= 9, ref=0, speed=0)
            print('10'); jog(jog_axis=10, ref=0, speed=  30); time.sleep(1); jog(jog_axis=10, ref=0, speed=0)
            print('11'); jog(jog_axis=11, ref=0, speed=  30); time.sleep(1); jog(jog_axis=11, ref=0, speed=0); time.sleep(1) # 이게 있어야 다음 move_l이 가능

        print(); print('##### Start (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('########################'); print()

        set_robot_mode(robot_mode=0) # 수동
        print(f'get_robot_mode() = {get_robot_mode()} (수동)')
        jog_test(); print('jog_test fin')

        # movej(home, vel=d_vel+100, acc=d_acc+100)

        # set_robot_mode(robot_mode=1) # 자동 # 실제모드로 불가능
        # print(f'get_robot_mode() = {get_robot_mode()} (자동)')
        # jog_test(); print('jog_test fin')

        print(); print('##### Fin (test) #####')
        print('move home'); movej(home, vel=d_vel+100, acc=d_acc+100)
        print('######################'); print(); print(); print(); print(); print(); print()

        rclpy.shutdown()
        return

    if ROBOT_RUN_MODE == 'man':
        set_robot_mode(robot_mode=0) # 수동
        print(f'get_robot_mode() = {get_robot_mode()} (수동)')
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

    if ROBOT_RUN_MODE == 'msg_test':

        print('a')














    ############################## main ##############################

    # 시작
    print(); print('##### Start #####')
    print('move home'); movej(home, vel=d_vel, acc=d_acc)
    print('#################'); print()
    
    
    if ROBOT_RUN_MODE == 'writing': # @
        # set_robot_mode(robot_mode=0) # 수동
        # print(f'get_robot_mode() = {get_robot_mode()} (수동)') # 기본적으로 파이썬을 실행시키면 자동모드가 됨
        i = 1
        while True:
            print(f'{i} posx : {get_current_posx(ref=DR_BASE)}', end = ' | ')
            print(f'DI_1 : {get_digital_input(1)}')
            i+=1

        #### 기타 명령어 :  ------------------------------------------------------------------------------------------
    elif ROBOT_RUN_MODE == 'trans': # 
        ############# trans #############
        pos   = [100,0,0,0,0,0]
        delta = [100,0,0,0,0,0]
        print(f'pos   = {pos}')
        print(f'delta = {delta}')
        print(trans(pos, delta, ref=DR_TOOL, ref_out=DR_BASE))

    elif ROBOT_RUN_MODE == 'ikin': # 
        ############# ikin ############# # task -> joint
        pos = posx(559.0, 34.5, 651.5, 45.0, 180.0, 45.0); sol_space = 2
        print('ikin') # print('조인트 각도로 변환된 값 잘 나오는지 확인')
        # print(f'pos = {pos}')
        print(np.round(ikin(pos, sol_space, ref=DR_BASE), 2) )

    elif ROBOT_RUN_MODE == 'fkin': #
        ############# fkin ############# # joint -> task
        # pos = posj(   0,   0,  90,   0,  90,   0)
        pos = posj(367.38607788, 4.22731638, 322.96060181, 90.14829254, 179.98649597, 90.4562912 )
        print('fkin') # print('테스크 위치로 변환된 값 잘 나오는지 확인')
        print(np.round(fkin(pos, ref=DR_BASE)) )

    elif ROBOT_RUN_MODE == 'set_ref_coord':
        ############# set_ref_coord #############
        print('true 뜨는지 보기') # 둘다 0(success) 가 반환됨
        print(f'set_ref_coord(1)     =     {set_ref_coord(1)}')
        print(f'set_ref_coord(DR_BASE)     =     {set_ref_coord(DR_BASE)}')

    elif ROBOT_RUN_MODE == 'COS':
        ############# change_operation_speed(speed) #############
        x0 = posx( 400, 200, 310,   0, 180,   0)
        x1 = posx( 400,-200, 310,   0, 180,   0)
        print('# inital pose')
        movejx(x1, vel=d_vel, acc=d_acc, sol=2)
        print()
        print('# speed = 50')
        change_operation_speed(50)
        low_speed_1 = time.time()
        movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        movejx(x1, vel=d_vel, acc=d_acc, sol=2) # 12.3s / real : 11.48s
        low_speed_2 = time.time()
        print(f'low_speed_time = {np.round(low_speed_2-low_speed_1, 2)}s')
        print()
        print('# speed = 100')
        change_operation_speed(100)
        high_speed_1 = time.time()
        movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        movejx(x1, vel=d_vel, acc=d_acc, sol=2) # 6.27s / real : 5.94s
        high_speed_2 = time.time()
        print(f'high_speed_time = {np.round(high_speed_2-high_speed_1, 2)}s')
        print()

    elif ROBOT_RUN_MODE == 'alter_motion': # ?
        ############# alter_motion ############# # 어떻게 되는지는 모름
        print('어떤 결과가 나오는지만 보기')
        pos=posx( 100,    0,    0, 100,    0,    0)
        enable_alter_motion(5, 0, ref=DR_BASE, limit_dPOS=[ 50, 90], limit_dPOS_per=[ 50, 50])
        alter_motion(pos)
        disable_alter_motion()


    elif ROBOT_RUN_MODE == 'singular': # 되는건지는 모르겠음, 어떻게 보여줘야 할지?
        set_robot_system(robot_system=1) # 가상
        print(f'get_robot_system() = {get_robot_system()}')

        ############# set_singular_handling #############
        x0 = posx(  80,-200, 500,   0, 180,   0)
        x1 = posx(  80, 200, 500,   0, 180,   0)
        print('특이점 회피 변하는지만 보기')
        time.sleep(1); movel(x1, vel=d_velx, acc=d_accx); print('initial'); time.sleep(1)
        print('DR_AVOID')
        set_singular_handling(DR_AVOID)
        movel(x0, vel=d_velx, acc=d_accx); movel(x1, vel=d_velx, acc=d_accx)
        print('DR_TASK_STOP')
        set_singular_handling(DR_TASK_STOP)
        movel(x0, vel=d_velx, acc=d_accx); movel(x1, vel=d_velx, acc=d_accx)
        print('DR_VAR_VEL')
        set_singular_handling(DR_VAR_VEL)
        movel(x0, vel=d_velx, acc=d_accx); movel(x1, vel=d_velx, acc=d_accx)
        
        set_robot_system(robot_system=0) # 실제
        print(f'get_robot_system() = {get_robot_system()}')

    # elif ROBOT_RUN_MODE == 'angle_test': # 125
    #     # movej([0,0, 90,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}')
    #     # movej([0,0,100,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}')
    #     # movej([0,0,110,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}')
    #     # movej([0,0,120,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}') # 125 까지만
    #     # movej([0,0,130,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}')
    #     # movej([0,0,140,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}')
    #     # movej([0,0,150,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}')
    #     # movej([0,0,160,0,0,0], vel=d_vel, acc=d_acc); print(f'axis_3 : {get_current_posj()[2]}')
    #     print('600'); movel([300, 300, 500,   0, 180,   0], vel=d_velx, acc=d_accx); print(f'axis_3 : {get_current_posj()[2]}')
    #     print('500'); movel([250, 250, 500,   0, 180,   0], vel=d_velx, acc=d_accx); print(f'axis_3 : {get_current_posj()[2]}')
    #     print('400'); movel([200, 200, 500,   0, 180,   0], vel=d_velx, acc=d_accx); print(f'axis_3 : {get_current_posj()[2]}')
    #     print('300'); movel([150, 150, 500,   0, 180,   0], vel=d_velx, acc=d_accx); print(f'axis_3 : {get_current_posj()[2]}')
    #     print('200'); movel([100, 100, 500,   0, 180,   0], vel=d_velx, acc=d_accx); print(f'axis_3 : {get_current_posj()[2]}')
        

    elif ROBOT_RUN_MODE == 'DRL_motion': # 가상 안됨, 실제로도 안됨
        ############# DRL example : run, pause, resume #############
        # 초기
        if   RorV == 0: print('real mode')
        elif RorV == 1: print('virtual mode')
        else: print('None')
        movej(posj(0,0,90,0,90,0), vel=30, acc=30)
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')
        print(); # time.sleep(1)

        # DRL
        drl_script_run(RorV, cus_code) # start
        print('# start')
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')
        print(); # time.sleep(1)
        drl_script_pause() # pause
        print('# pause')
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')
        print(); # time.sleep(1)
        drl_script_resume() # resume
        print('# resume')
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')
        print(); # time.sleep(1)
    
    elif ROBOT_RUN_MODE == 'DRL_stop0':
        ############# DRL example : stop0 #############
        drl_script_run(RorV, cus_code)
        time.sleep(1)
        drl_script_stop(0)
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')
    
    elif ROBOT_RUN_MODE == 'DRL_stop1':
        ############# DRL example : stop1 #############
        drl_script_run(RorV, cus_code)
        time.sleep(1)
        drl_script_stop(1)
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')
    
    elif ROBOT_RUN_MODE == 'DRL_stop2':
        ############# DRL example : stop2 #############
        drl_script_run(RorV, cus_code)
        time.sleep(1)
        drl_script_stop(2)
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')
    
    elif ROBOT_RUN_MODE == 'DRL_stop3':
        ############# DRL example : stop3 #############
        drl_script_run(RorV, cus_code)
        time.sleep(1)
        drl_script_stop(3)
        print(f'DrlState = {get_drl_state()} (0: 작동     / 1: stop  / 2: hold)')
        print(f'Motion   = {check_motion()}  (0: 모션 없음 / 1: 연산중 / 2: 수행중)')

    elif ROBOT_RUN_MODE == 'system':

        def test_move(): # 해당 구조에서는 time.sleep이 있어야 오류가 안남
            time.sleep(1)
            movejx([ 367, 200, 311,   0, 180,   0], vel=d_vel, acc=d_acc, sol=2)
            a = time.time()
            movejx([ 367,-200, 311,   0, 180,   0], vel=d_vel, acc=d_acc, sol=2)
            b = time.time()   
            time.sleep(1)
            print(f'move time : {np.round(b-a, 2)}')
        
        # ############# system #############
        # print('#### robot system ####') # 
        # print('robot_system (0: 실제 / 1: 가상)')
        # set_robot_system(robot_system=1) # 가상
        # print(f'get_robot_system() = {get_robot_system()}'); test_move()
        # print()
        # set_robot_system(robot_system=0) # 실제
        # print(f'get_robot_system() = {get_robot_system()}'); test_move()

        # print('#### robot mode ####') # 정상, 실제에선 속도 차이 없고, LED 잘 바뀜
        # print('robot_mode (0: 수동(조그) / 1: 자동(프로그램))')
        # set_robot_mode(robot_mode=0)
        # test_move()
        # print()
        # print(f'get_robot_mode() = {get_robot_mode()}')
        # set_robot_mode(robot_mode=1)
        # test_move()
        # print()
        # print(f'get_robot_mode() = {get_robot_mode()}')
        # print()

        # print('#### speed mode ####') # 수동이던 자동이던 스피드 모드 변화 없음
        # # set_robot_mode(robot_mode=0) # 수동모드 (택1)
        # # # set_robot_mode(robot_mode=1) # 자동모드 (택1)
        # if   get_robot_mode() == 0: print('수동모드')
        # elif get_robot_mode() == 1: print('자동모드')
        # else: print('None')
        # print('robot_speed_mode (0: 정속 / 1: 감속)')
        # movejx([ 367,-200, 311,   0, 180,   0], vel=d_vel, acc=d_acc, sol=2)
        # set_robot_speed_mode(speed_mode=1)
        # print(f'get_robot_speed_mode() = {get_robot_speed_mode()} (감속)')
        # test_move()
        # set_robot_speed_mode(speed_mode=0)
        # print(f'get_robot_speed_mode() = {get_robot_speed_mode()} (정속)')
        # test_move()
        # print()
        # # 가상
        # # 속도 차이 없음 
        # # 실제
        # # 수동모드 : 속도 차이 없음 
        # # 자동모드 : 속도 차이 없음
        # # 모드 지정 안하면 (자동모드라고 뜨긴 함): 감속 엄청 느림, 정속 빠름) 14.73 / 3.11

        # print('#### current pose ####') # 
        # print(f'get_current_pose(0) (joint) = {np.round(get_current_pose(space_type=0), 2)}')
        # print(f'get_current_pose(1) (task)  = {np.round(get_current_pose(space_type=1), 2)}')
        # print()

        # print('#### ETC ####')
        # print('get_last_alarm() / (level, group, index, param)')
        # print(get_last_alarm())
        # print()
        # print('get_robot_state() / (1: 윤용대기 / 3: 서보오프 / 5: 안전정지 / 6: 비상정지)')
        # print(get_robot_state())
        # print()
        # print('set_safe_stop_reset_type (0: 단순 상태 해제(수동), 프로그램 종료(자동) / 1: 프로그램 재시작(자동))')
        # set_safe_stop_reset_type(reset_type=0)
        # print()

    elif ROBOT_RUN_MODE == 'aux_control':
        ############# aux_control #############
        print('#### aux_control ####')
        print(f'get_current_posj()                        = {get_current_posj()}')
        print(f'get_current_velj()                        = {get_current_velj()}') # 0만 반환 (실제도 동일)
        print(f'get_desired_posj()                        = {get_desired_posj()}') # 0만 반환 (실제도 동일)
        print(f'get_desired_velj()                        = {get_desired_velj()}') # 0만 반환 (실제도 동일)
        print(f'get_current_posx(ref=DR_BASE)             = {np.round(get_current_posx(ref=DR_BASE)[0], 2)}') # 외력 감지시 여기서 멈춤
        print(f'get_current_velx(ref=DR_BASE)             = {get_current_velx(ref=DR_BASE)}') # 0만 반환 (실제도 동일)
        print(f'get_desired_posx(ref=DR_BASE)             = {get_desired_posx(ref=DR_BASE)}')
        print(f'get_desired_velx(ref=DR_BASE)             = {get_desired_velx(ref=DR_BASE)}') # 0만 반환 (실제도 동일)
        print(f'get_current_solution_space()              = {get_current_solution_space()}')
        print(f'get_solution_space([0,0,90,0,90,0])       = {get_solution_space([0,0,90,0,90,0])}')
        print(f'get_current_tool_flange_posx(ref=DR_BASE) = {get_current_tool_flange_posx(ref=DR_BASE)}') # 0만 반환
        xd=[400,0,500,0,180,0]; xc=[400,0,500,0,0,0]; axis=DR_AXIS_X
        print(f'get_orientation_error(xd, xc, axis)       = {get_orientation_error(xd, xc, axis)}') # ?
        print(f'get_control_mode()                        = {get_control_mode()}') # 0만 반환
        print(f'    3: position / 4: Torque')
        print(f'get_control_space()                       = {get_control_space()}') # 0만 반환
        print(f'    1: joint / 2: Task')
        print(f'get_current_rotm(ref=DR_BASE)             = {get_current_rotm(ref=DR_BASE)}') # tool 설정하고 해보기
        print(f'get_joint_torque()                        = {get_joint_torque()}') # 0만 반환 (실제도 동일)
        print(f'get_external_torque()                     = {get_external_torque()}') # 0만 반환 (실제도 동일)
        print(f'get_tool_force(ref=DR_BASE)               = {get_tool_force(ref=DR_BASE)}') # 0만 반환 (실제도 동일)
        print()

    elif ROBOT_RUN_MODE == 'TCPTool':
        ############# TCPTool ############# # 재 연결 했을때 ROS 에서 지정한 것이 남아있는지 보기
        name = 'asd'
        pos = [0,0,0,0,0,0]
        print('get_tcp()'); print(get_tcp()); print()
        print('add_tcp(name, pos)'); add_tcp(name, pos); print(get_tcp()); print()
        print('set_tcp(name)'); set_tcp(name); print(get_tcp()); print()
        print('del_tcp(name)'); del_tcp(name); print(get_tcp()); print()
        name = 'asdasd'
        print('get_tool()'); print(get_tool()); print()
        print('add_tool'); add_tool(name, weight=10, cog=[0,0,0], inertia=[0,0,0,0,0,0]); print(get_tool()); print()
        print('set_tool(name)'); set_tool(name); print(get_tool()); print()
        print('del_tool(name)'); del_tool(name); print(get_tool()); print()
        print('set_tool_shape(name)'); set_tool_shape(name); print(get_tool()); print()

    elif ROBOT_RUN_MODE == 'axis':
        ############# Axis example #############
        # 수동모드로 하면 어느정도 정상 작동함
        # 하지만 parallel_axis_1, align_axis_1 직후 move가 씹힘

        # 수동모드 변환 필수
        set_robot_mode(robot_mode=0) # 수동
        print(f'get_robot_mode() = {get_robot_mode()} (수동)')
        a_axis = DR_AXIS_Z
        a_ref  = DR_BASE

        ## initial pos : show normal vector direction
        up_pos = posx( 300,-100, 400,   0, 180,   0)
        dn_pos = posx( 300,   0, 300,   0, 180,   0)
        print('1 movejx'); movejx(up_pos, vel=d_vel, acc=d_acc, sol=2)
        print('2 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)

        ## ParallelAxis1 : stop issue # 수동모드는 멈추지 않음 # 실제로는 이거 다음 동작이 씹힘
        x1     = [ 100,-100,   0,   0,   0,   0]
        x2     = [-100,-100,   0,   0,   0,   0]
        x3     = [   0,   0, 100,   0,   0,   0]
        print('3 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('4 parallel_axis'); parallel_axis(x1=x1, x2=x2, x3=x3, axis=a_axis, ref=a_ref) # len_() == 5 # _nType = 2
        
        ## ParallelAxis2 : good
        n_vect = [   0,   1,  -1]
        movej([0,0,90,0,90,0], vel=30, acc=30) # 이게 작동하지 않음
        print('5 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('6 parallel_axis'); parallel_axis(vect=n_vect, axis=a_axis, ref=a_ref) # len_() == 3 # _nType = 3

        ## AlignAxis1 : stop issue # 수동모드는 멈추지 않음 # 실제로는 이거 다음 동작이 씹힘
        s_vect = [ 300, 0, 300]
        print('7 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('8 align_axis'); align_axis(x1=x1, x2=x2, x3=x3, pos=s_vect, axis=a_axis, ref=a_ref) # len_() == 6 # _nType == 2

        ## AlignAxis2 : good
        t_vect = n_vect
        movej([0,0,90,0,90,0], vel=30, acc=30) # 이게 작동하지 않음
        print('9 movejx'); movejx(dn_pos, vel=d_vel, acc=d_acc, sol=2)
        print('10 align_axis'); align_axis(vect=t_vect, pos=s_vect, axis=a_axis, ref=a_ref)# len_() == 4 # _nType == 3


    elif ROBOT_RUN_MODE == 'coord':
        ############# Coordinate example #############

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

        def show_all_coord(id_list, txt):
            print(); print('txt')
            for id in id_list:
                print(f'coord : {id}'); movejx(posx(0,0,0,0,0,0), vel=d_vel+100, acc=d_acc+100, sol=2, ref=id)

        c_axis = DR_AXIS_Z
        c_ref  = DR_BASE
    
        ## 기본 위치 (4개의 좌표계 원점 위치)
        cooro_1 = [ 400,   0, 300,  90, 135,  90] # 367.5,   0, 311,   0, 180,   0
        cooro_2 = [ 400,   0, 300,   0, 135,   0]
        cooro_3 = [ 400,   0, 300, -90, 135, -90]
        cooro_4 = [ 400,   0, 300, 180, 135, 180]
        # movejx(cooro_1, vel=d_vel, acc=d_acc, sol=2)
        # movejx(cooro_2, vel=d_vel, acc=d_acc, sol=2)
        # movejx(cooro_3, vel=d_vel, acc=d_acc, sol=2)
        # movejx(cooro_4, vel=d_vel, acc=d_acc, sol=2)

        ## SetUserCartCoord1 : 해당 지점을 원점으로, 서비스, 파이썬, 가상, 실제 전부 정상
        id1 = set_user_cart_coord(pos=cooro_1, ref=c_ref)
        print(f'SetUserCartCoord1 = {id1}'); show_coord(id1)

        ## SetUserCartCoord2 : 오류 발생 / 서비스는 가상모드 안됨, 실제모드 됨 (연결시, 가상모드도 됨) (함수도 동일)
        x1 = [  0,  0,  0,  0,  0,  0]
        x2 = [  0,100,  0,  0,  0,  0]
        x3 = [100,  0,100,  0,  0,  0]
        id2 = set_user_cart_coord(x1=x1, x2=x2, x3=x3, pos=cooro_2, ref=c_ref)
        print(f'SetUserCartCoord2 = {id2}'); show_coord(id2)

        ## SetUserCartCoord3 : 각 벡터, 오른손 법칙, 서비스, 파이썬, 가상, 실제 전부 정상
        u1 = [-1, 0, 0]
        v1 = [ 0, 1,-1]
        id3 = set_user_cart_coord(u1=u1, v1=v1, pos=cooro_3, ref=c_ref)
        print(f'SetUserCartCoord3 = {id3}'); show_coord(id3)

        ## 전체 좌표계 확인
        show_all_coord([id1, id2, id3], 'all coord')

        ## 덮어씌우기
        overwrite_user_cart_coord(id=id2, pos=cooro_4, ref=DR_BASE)

        ## 전체 좌표계 확인
        show_all_coord([id1, id2, id3], 'all coord (replaced)')


    ######### 여기부터 하기 # @@@@@@@@@@@@@@@@@@@@

    elif ROBOT_RUN_MODE == 'comp':
        ############# ComplianceCtrl example ############# # 코드 짰으니, 실제로 테스트 해보기
        # 힘제어 안됨, 이동 후 딜레이

        # 자동모드 변환 필수
        set_robot_mode(robot_mode=1) #자동
        print(f'get_robot_mode() = {get_robot_mode()} (자동)')

        # # 수동모드 변환 필수
        # set_robot_mode(robot_mode=0) #수동
        # print(f'get_robot_mode() = {get_robot_mode()} (수동)')

        # 접촉하므로, TCP, tool 지정
        tcp_name  = 'test_tcp'
        tool_name = 'test_tool'
        print(f'######### TCP and Tool #########')
        print(f'# initial')
        print(f'present TCP  : {get_tcp()}')
        print(f'present Tool : {get_tool()}')
        print(f'# set')
        print(f'add_tcp'); add_tcp(tcp_name, pos=[0,0,0,0,0,0])
        print(f'set_tcp'); set_tcp(tcp_name)
        print(f'add_tool'); add_tool(tool_name, weight=10, cog=[0,0,0], inertia=[0,0,0,0,0,0])
        print(f'set_tool'); set_tool(tool_name)
        print(f'# get')
        print(f'present TCP  : {get_tcp()}')
        print(f'present Tool : {get_tool()}')
        print()

        # fd : 힘 성분 3개(병진), 모멘트 성분 3개(회전) / dir : 1이면 힘 제어, 0이면 순응제어
        cc_stx = [500, 500, 500, 100, 100, 100]
        cc_fd  = [  0,   0, -30,   0,   0,   0]
        cc_dir = [  0,   0,   1,   0,   0,   1]
        # 위치 정의
        init_pos   = posj(   0,   0,  90,   0,  90,   0)
        target_pos = posx( 400,   0, 100,   0, 180,   0) # Very low, so be careful
        rel_pos    = posx(   0,-100,   0,   0,   0,   0) 
        # 예제
        print(f'######### Comfliance control #########')
        print('# movej');                   movej(init_pos, vel=d_vel, acc=d_acc)
        print('# movejx');                  movejx(target_pos, vel=d_vel, acc=d_acc, sol=2)
        print('# task_compliance_ctrl');    print(task_compliance_ctrl(stx=cc_stx, time=0)) # Chech the reference coordinate / when service call, ref: 1
        print('# set_desired_force');       print(set_desired_force(fd=cc_fd, dir=cc_dir, time=1)) # ref: 0, mod: 0}"
        time.sleep(10)
        print('# movel');                   movel(rel_pos, vel=d_velx, acc=d_accx, mod=DR_MV_MOD_REL) # 이러고 멈춤
        print('# release_force');           release_force(time=0.5)
        print('# release_compliance_ctrl'); release_compliance_ctrl()

    elif ROBOT_RUN_MODE == 'CT':
        ############# CoordTransform example ############# # 0만 뜨는데 이유는?
        pose_in_1 = [ 400,   0, 500,   0, 180,   0]
        pose_in_2 = [   0,   0,   0,   0,   0,   0]
        id1 = set_user_cart_coord(pos=[ 400,   0, 500,  90, 135,  90], ref=DR_BASE)
        print(f'base -> tool : {coord_transform(pose_in_1, ref_in=DR_BASE, ref_out=DR_TOOL)}'); # base를 tool로 (맞겠지)
        print(f'tool -> base : {coord_transform(pose_in_2, ref_in=DR_TOOL, ref_out=DR_BASE)}'); # tool을 base로 (현위치 나와야 함) (정상)
        print(f'base -> user : {coord_transform(pose_in_1, ref_in=DR_BASE, ref_out=id1)}');     # base를 user로 (정상)

    elif ROBOT_RUN_MODE == 'workpiece':
        ############# Workpiece ############# # 막줄에서 안넘어가는 현상이 있었는데, 지금은 잘 되네
        print('debug_1'); print(f'get_workpiece_weight() = {get_workpiece_weight()}'); print()
        print('debug_2'); reset_workpiece_weight(); print()
        # print('debug_3'); print(f'reset_workpiece_weight fin'); print()
        # print('debug_4'); print(f'get_workpiece_weight() = {get_workpiece_weight()}'); print() # 리셋 후 여기서 멈춤

    elif ROBOT_RUN_MODE == 'condition':
        ############# ????? ############# # ?
        check_position_condition(axis=2, min=DR_COND_NONE, max=DR_COND_NONE, ref=DR_BASE, mod= DR_MV_MOD_ABS)
        check_force_condition(axis=2, min=DR_COND_NONE, max=DR_COND_NONE, ref=DR_BASE) # axis = 2? DR_AXIS_Z?
        check_orientation_condition(axis, min=None, max=None, ref=None, mod = None, pos=None)
        check_orientation_condition(axis, min=None, max=None, ref=None, mod = None, pos=None)
        is_done_bolt_tightening(m=0, timeout=0, axis=None)
    
    elif ROBOT_RUN_MODE == 'getIO': # 이거 두개는 했음
        ## example : get IO signal
        get_all_IO()
    
    elif ROBOT_RUN_MODE == 'setDO': # 가상에서 안바뀜, 실제로 해보기 # 이거 두개는 했음
        ## example : Set DIO signal (index = 2, 3)
        set_digital_output(index=4, val=1)
        set_tool_digital_output(index=3, val=1)
        time.sleep(1) # 이게 있어야 반영됨
        print(f'ControlBox Digital Output : {get_digital_output(index=4)}')
        print(f'      Tool Digital Output : {get_tool_digital_output(index=3)}')
        print()
        set_digital_output(index=4, val=0)
        set_tool_digital_output(index=3, val=0)
        time.sleep(1) # 이게 있어야 반영됨
        print(f'ControlBox Digital Output : {get_digital_output(index=4)}')
        print(f'      Tool Digital Output : {get_tool_digital_output(index=3)}')
        print()

    elif ROBOT_RUN_MODE == 'setAO': # 가상에서 안바뀜, 실제로 해보기
        ## example : Set Analog Output (Check TP)
        # CH_1 / CURRENT / 15
        print('debug_1'); set_mode_analog_output(ch=1, mod=0); time.sleep(1); set_analog_output(ch=1, val=15)
        # CH_1 / CURRENT / 0
        print('debug_2'); set_mode_analog_output(ch=1, mod=0); time.sleep(1); set_analog_output(ch=1, val=0)
        # CH_1 / VOLTAGE / 5
        print('debug_3'); set_mode_analog_output(ch=1, mod=1); time.sleep(1); set_analog_output(ch=1, val=5)
        # CH_1 / VOLTAGE / 0
        print('debug_4'); set_mode_analog_output(ch=1, mod=1); time.sleep(1); set_analog_output(ch=1, val=0)
        # CH_2 / CURRENT / 15
        print('debug_5'); set_mode_analog_output(ch=2, mod=0); time.sleep(1); set_analog_output(ch=2, val=15)
        # CH_2 / CURRENT / 0
        print('debug_6'); set_mode_analog_output(ch=2, mod=0); time.sleep(1); set_analog_output(ch=2, val=0)
        # CH_2 / VOLTAGE / 5
        print('debug_7'); set_mode_analog_output(ch=2, mod=1); time.sleep(1); set_analog_output(ch=2, val=5)
        # CH_2 / VOLTAGE / 0
        print('debug_8'); set_mode_analog_output(ch=2, mod=1); time.sleep(1); set_analog_output(ch=2, val=0)
    
    elif ROBOT_RUN_MODE == 'setAI':
        ## example : Set Analog Input (Check TP)
        set_mode_analog_input(ch=1, mod=0); (get_analog_input(ch=1))
        set_mode_analog_input(ch=1, mod=1); (get_analog_input(ch=1))
        set_mode_analog_input(ch=2, mod=0); (get_analog_input(ch=2))
        set_mode_analog_input(ch=2, mod=1); (get_analog_input(ch=2))



    # 2주 10명
    # 가천대에서는 어떤 과제를 할지 ROS2 내가 좀 받아서
    # 우리쪽에서 특강을 해줬으면 좋겠다고 함
    # 2,3시간정도
    # 개념 소개 뭘할수있는지 보여
    # 기존 로봇 교육 받았을 듯
    # 언제인지 12월 9월 이후
    # 






    #### 기본 동작 ------------------------------------------------------------------------------------------

    elif ROBOT_RUN_MODE == 'move_j':
        ############# move_j #############
        p0 = posj(   0, -20, 110,   0,  90,   0)
        p1 = posj(   0,  20,  70,   0,  90,   0)
        p2 = posj(   0,   0,  90,   0,  90,   0)
        i = 0; current_time = 0
        print('move j 1'); movej(p0, vel=d_vel, acc=d_acc)
        print('move j 2'); movej(p1, vel=d_vel, acc=d_acc)
        print('move j 3'); movej(p2, vel=d_vel, acc=d_acc)

    elif ROBOT_RUN_MODE == 'move_jx':
        ############# move_jx #############
        x0 = posx( 300,  150, 300,   0, 180,   0)
        x1 = posx( 300, -150, 300,   0, 180,   0)
        x2 = posx( 500, -150, 300,   0, 180,   0)
        x3 = posx( 500,  150, 300,   0, 180,   0)
        print('move jx 1'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 2'); movejx(x1, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 3'); movejx(x2, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 4'); movejx(x3, vel=d_vel, acc=d_acc, sol=2)
        print('move jx 5'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)

    elif ROBOT_RUN_MODE == 'move_l':
        ############# move_l #############
        x0 = posx( 300,  150, 300,   0, 180,   0)
        x1 = posx( 300, -150, 300,   0, 180,   0)
        x2 = posx( 500, -150, 300,   0, 180,   0)
        x3 = posx( 500,  150, 300,   0, 180,   0)
        print('move l 1'); movel(x0, vel=d_velx, acc=d_accx)
        print('move l 2'); movel(x1, vel=d_velx, acc=d_accx)
        print('move l 3'); movel(x2, vel=d_velx, acc=d_accx)
        print('move l 4'); movel(x3, vel=d_velx, acc=d_accx)
        print('move l 5'); movel(x0, vel=d_velx, acc=d_accx)

    elif ROBOT_RUN_MODE == 'move_c':
        ############# move_c #############
        x0 = posx( 400,   0, 500,   0, 180,   0)
        c1 = posx( 500,-100, 500,   0, 180,   0)
        c2 = posx( 600,   0, 500,   0, 180,   0)
        c3 = posx( 500, 100, 500,   0, 180,   0)
        print('move jx'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('move c 1'); movec(c1, c2, d_velx, d_accx)
        print('move c 2'); movec(c3, x0, d_velx, d_accx)

    elif ROBOT_RUN_MODE == 'move_sj':
        ############# move_sj #############
        q0 = posj(   0,   0, 120,   0,   0,   0)
        q1 = posj(  30,   0, 120,   0,   0,   0)
        q2 = posj(  30,   0,  90,   0,  90,   0)
        q3 = posj(   0,   0,  90,   0,  90,   0)
        qlist = [q0, q1, q2, q3]
        print('move sj'); movesj(qlist, vel=d_vel, acc=d_acc, time=10)

    elif ROBOT_RUN_MODE == 'move_sx':
        ############# move_sx #############
        x0 = posx( 300,  150, 300,   0, 180,   0)
        x1 = posx( 300, -150, 300,   0, 180,   0)
        x2 = posx( 500, -150, 300,   0, 180,   0)
        x3 = posx( 500,  150, 300,   0, 180,   0)
        xlist = [x0, x1, x2, x3, x0]
        print('move sx'); movesx(xlist, vel=300, acc=300)

    elif ROBOT_RUN_MODE == 'move_b':
        ############# move_b #############
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

    elif ROBOT_RUN_MODE == 'move_spiral':
        ############# move_spiral ############# # 가상은 m0609도 됨
        x0 = posx( 500,   0, 500,   0, 180,   0)
        print('move jx'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('move_spiral'); move_spiral(rev=2, rmax=100, lmax=100, vel=[60,60], acc=[60,60], time=10, axis=DR_AXIS_Z, ref=DR_TOOL) # DR_BASE

    elif ROBOT_RUN_MODE == 'move_periodic':
        ############# move_periodic #############
        x0 = posx(367.5,   0, 311,   0, 180,   0) # [367.5,   0, 311,   0, 180,   0]
        print('## BASE'); movejx(x0, vel=d_vel, acc=d_acc, sol=2)
        print('X_trans | amp =[30, 0, 0, 0, 0, 0] '); move_periodic(amp =[30, 0, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('X_rot   | amp =[ 0, 0, 0,10, 0, 0] '); move_periodic(amp =[ 0, 0, 0,10, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('Y_trans | amp =[ 0,30, 0, 0, 0, 0] '); move_periodic(amp =[ 0,30, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('Y_rot   | amp =[ 0, 0, 0, 0,10, 0] '); move_periodic(amp =[ 0, 0, 0, 0,10, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('Z_trans | amp =[ 0, 0,30, 0, 0, 0] '); move_periodic(amp =[ 0, 0,30, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print('Z_rot   | amp =[ 0, 0, 0, 0, 0,10] '); move_periodic(amp =[ 0, 0, 0, 0, 0,10], period=2, atime=0.2, repeat=1, ref=DR_BASE)
        print()
        print('## TOOL')#; movejx(x1, vel=d_vel, acc=d_acc, sol=2)
        print('X_trans | amp =[30, 0, 0, 0, 0, 0] '); move_periodic(amp =[30, 0, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('X_rot   | amp =[ 0, 0, 0,10, 0, 0] '); move_periodic(amp =[ 0, 0, 0,10, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()
        print('Y_trans | amp =[ 0,30, 0, 0, 0, 0] '); move_periodic(amp =[ 0,30, 0, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('Y_rot   | amp =[ 0, 0, 0, 0,10, 0] '); move_periodic(amp =[ 0, 0, 0, 0,10, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print()
        print('Z_trans | amp =[ 0, 0,30, 0, 0, 0] '); move_periodic(amp =[ 0, 0,30, 0, 0, 0], period=2, atime=0.2, repeat=1, ref=DR_TOOL)
        print('Z_rot   | amp =[ 0, 0, 0, 0, 0,10] '); move_periodic(amp =[ 0, 0, 0, 0, 0,10], period=2, atime=0.2, repeat=1, ref=DR_TOOL)

    elif ROBOT_RUN_MODE == 'move_home':
        ############# move_home ############# # 수동모드로만 가능, 비동기 인듯
        set_robot_mode(robot_mode=0) # 수동
        print('move_home 1'); move_home(target=0); time.sleep(5); print('1 fin')
        print('move_home 2'); move_home(target=1); time.sleep(5); print('2 fin')












        #### 응용 ------------------------------------------------------------------------------------------

    elif ROBOT_RUN_MODE == 'all':
        ############# all ############# # only virtual
        p0 = posj(-360,-360,-160,-360,-360,-360)
        p1 = posj( 360, 360, 160, 360, 360, 360)
        print('move j 1'); movej(p0, vel=d_vel, acc=d_acc)
        print('move j 2'); movej(p1, vel=d_vel, acc=d_acc)

    elif ROBOT_RUN_MODE == 'loop':
        ############# loop #############
        start_time = time.time()    
        p0 = posj(   0, -20, 110,   0,  90,   0)
        p1 = posj(   0,  20,  70,   0,  90,   0)
        p2 = posj(   0,   0,  90,   0,  90,   0)
        i = 0; current_time = 0
        while current_time - start_time < 600: # 10 min
            i += 1
            print('move j 1'); movej(p0, vel=d_vel, acc=d_acc)
            print('move j 2'); movej(p1, vel=d_vel, acc=d_acc)
            print('move j 3'); movej(p2, vel=d_vel, acc=d_acc)
            current_time = time.time() 
            print(f'time {i} : {current_time - start_time}')

    elif ROBOT_RUN_MODE == 'four': # @
        # ############# four position #############
        # # 관절 각도
        a = np.round(fkin(posj(   0, -20, 110,   0,  90,   0), ref=DR_BASE))
        b = np.round(fkin(posj(   0,  20,  70,   0,  90,   0), ref=DR_BASE))
        c = np.round(fkin(posj(  30,   0,  90,   0,  90,   0), ref=DR_BASE))
        d = np.round(fkin(posj( -30,   0,  90,   0,  90,   0), ref=DR_BASE))
        print()
        print(a) # 조인트 -> 테스크
        print(b)
        print(c)
        print(d)
        print()

        x_min = a[0]
        x_max = b[0]
        y_max = max(abs(c[1]), abs(d[1]))
        y_min = -y_max
        z_avg = round(np.mean([a[2],b[2],c[2],d[2]]))
        
        print('###### base position ######')
        print(f'base_p = [{np.mean([x_min, x_max])},   0, {z_avg},   0, 180,   0]')
        print()
        print('###### 4 position ######')
        print(f'x0 = [{x_min}, {y_max}, {z_avg},   0, 180,   0]')
        print(f'x1 = [{x_min}, {y_min}, {z_avg},   0, 180,   0]')
        print(f'x2 = [{x_max}, {y_min}, {z_avg},   0, 180,   0]')
        print(f'x3 = [{x_max}, {y_max}, {z_avg},   0, 180,   0]')
        print()

    elif ROBOT_RUN_MODE == 'test_show':

        p0 = posj(   0, -20, 110,   0,  90,   0)
        p1 = posj(   0,  20,  70,   0,  90,   0)
        p2 = posj(  30,   0,  90,   0,  90,   0)
        p3 = posj( -30,   0,  90,   0,  90,   0)

        def go_back():
            movel([0,0,100,0,0,0], vel=d_velx, acc=d_accx, mod=1, ref=1)
            # time.sleep(1)
            movel([0,0,-100,0,0,0], vel=d_velx, acc=d_accx, mod=1, ref=1)

        movej(p0, vel=d_vel, acc=d_acc)
        go_back()
        movej(p1, vel=d_vel, acc=d_acc)
        go_back()
        movej(p2, vel=d_vel, acc=d_acc)
        go_back()
        movej(p3, vel=d_vel, acc=d_acc)
        go_back()

    # 종료
    print(); print('##### Fin (test) #####')
    print('move home'); movej(home, vel=d_vel, acc=d_acc)
    print('######################'); print()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

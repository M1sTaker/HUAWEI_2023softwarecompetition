#!/bin/bash
import sys
import time
from navigate import move_to_xy
from avoidCrash import avoid_crash


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    # time.sleep(10)
    read_util_ok()#初始化完成
    finish()#初始化完成输出一个OK
    
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0]) #第一行：帧数 得分
         

        line = sys.stdin.readline().strip().split(' ')
        # 工作台数量
        K = int(line[0])
        
        #下面K行为工作站信息
        work_bench = []
        for i in range(K):
            line = sys.stdin.readline().strip().split(' ')
            work_bench.append({'id':int(line[0]), 'x':float(line[1]), 'y':float(line[2]), 'produce_remain_time':int((line[3])), 'raw_state':int(line[4]), \
                                 'product_state':int(line[5]) })
        
        
        #读取4个机器人信息
        robot_list = []     
        for i in range(4):
            line = sys.stdin.readline().strip().split(' ')
            robot_list.append({'work_bench_id': int(line[0]), 'item_tpye': int(line[1]), 'time_cost': float(line[2]), 'crash_cost': float(line[3]), \
                'angle_speed': float(line[4]), 'line_speed_x':float(line[5]), 'line_speed_y':(float(line[6])), 'face_angle': float(line[7]), \
                    'x': float(line[8]), 'y':float(line[9])})

        read_util_ok()#数据读入完成


        
        sys.stdout.write('%d\n' % frame_id)

        for robot_id in range(4):
            line_speed, angle_speed = move_to_xy(robot_list[robot_id], work_bench[robot_id]['x'], work_bench[robot_id]['y'])
            line_speed, angle_speed = avoid_crash(robot_list[robot_id], i, line_speed, angle_speed, robot_list)
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))

        # sys.stdout.write('%d\n' % frame_id)

        # line_speed, angle_speed = 3, 1.5
        # for robot_id in range(4):
        #     sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
        #     sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        finish()

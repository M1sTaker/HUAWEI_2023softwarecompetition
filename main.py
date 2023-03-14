#!/bin/bash
import sys
import time
from navigate import move_to_xy
from avoidCrash import avoid_crash
from strategy_greedy import strategy_greedy


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    read_util_ok()  # 初始化完成
    finish()  # 初始化完成输出一个OK

    targets_of_robots = [[], [], [], []]
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])  # 第一行：帧数 得分
        print("得分:" + str(parts[1]), file=sys.stderr)

        line = sys.stdin.readline().strip().split(' ')
        # 工作台数量
        K = int(line[0])

        # 下面K行为工作站信息
        work_bench = []
        for i in range(K):
            line = sys.stdin.readline().strip().split(' ')
            work_bench.append(
                {'id': int(line[0]), 'x': float(line[1]), 'y': float(line[2]), 'produce_remain_time': int((line[3])),
                 'raw_state': int(line[4]),
                 'product_state': int(line[5])})

        # 读取4个机器人信息
        robot_list = []
        for i in range(4):
            line = sys.stdin.readline().strip().split(' ')
            robot_list.append({'work_bench_id': int(line[0]), 'item_type': int(line[1]), 'time_cost': float(line[2]),
                               'crash_cost': float(line[3]),
                               'angle_speed': float(line[4]), 'line_speed_x': float(line[5]),
                               'line_speed_y': (float(line[6])), 'face_angle': float(line[7]),
                               'x': float(line[8]), 'y': float(line[9])})

        sys.stdout.write('%d\n' % frame_id)

        for robot in robot_list:
            robot_id = robot_list.index(robot)
            target = targets_of_robots[robot_id]
            # 如果该机器人没有被分配任务或者该机器人没有在任何工作台附近
            if not target or robot['work_bench_id'] == -1:
                continue
            # 否则，判断是否可以进行购买或出售动作
            else:
                if robot['item_type'] == 0 and work_bench[target[0]]['product_state'] == 1 and robot['work_bench_id'] == \
                        target[0]:
                    sys.stdout.write('buy %d \n' % robot_id)
                elif robot['item_type'] != 0 and robot['work_bench_id'] == target[1] and robot['item_type'] & \
                        work_bench[target[1]]['raw_state'] == 0:
                    sys.stdout.write('sell %d \n' % robot_id)
                    targets_of_robots[robot_id] = []

        targets_of_robots = strategy_greedy(work_bench, robot_list, targets_of_robots)
        print(targets_of_robots, file=sys.stderr)
        for robot_id in range(4):
            target = targets_of_robots[robot_id]
            robot = robot_list[robot_id]
            if not target:

                continue

            # 如果机器人手中无货，则前往目标取货台
            if robot['item_type'] == 0:
                line_speed, angle_speed = move_to_xy(robot, work_bench[target[0]]['x'],
                                                     work_bench[target[0]]['y'])
                line_speed, angle_speed = avoid_crash(robot_list[robot_id], robot_id, line_speed, angle_speed, robot_list)
                sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
                sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
            if robot['item_type'] != 0:
                line_speed, angle_speed = move_to_xy(robot, work_bench[target[1]]['x'],
                                                     work_bench[target[1]]['y'])
                line_speed, angle_speed = avoid_crash(robot_list[robot_id], robot_id, line_speed, angle_speed, robot_list)
                sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
                sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))

        # sys.stdout.write('%d\n' % frame_id)

        # line_speed, angle_speed = 3, 1.5
        # for robot_id in range(4):
        #     sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
        #     sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        finish()


        read_util_ok()  # 数据读入完成

#!/bin/bash
import sys
import time
from navigate import move_to_xy
from avoidCrash import avoid_crash
from strategies import strategy_greedy


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    read_util_ok()  # 初始化完成
    finish()  # 初始化完成输出一个OK

    strategies_of_robots = [[], [], [], []]
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])  # 第一行：帧数 得分

        line = sys.stdin.readline().strip().split(' ')
        # 工作台数量
        num_of_work_bench = int(line[0])

        print("帧数:" + str(frame_id) + "; 得分:" + str(parts[1]) + "工作台数量：" + str(num_of_work_bench), file=sys.stderr)
        # 下面K行为工作站信息
        work_bench_list = []
        for i in range(num_of_work_bench):
            line = sys.stdin.readline().strip().split(' ')
            # 工作台类型，整数，1-9
            # 坐标(x,y)，浮点数
            # 剩余生产时间，整数（单位：帧）
            # 原材料格状态，整数，二进制位表描述，例如 48(110000) 表示拥有物品 4 和 5。
            # 产品格状态，整数，0表示无，1表示有
            work_bench_list.append(
                {'id': i, 'type': int(line[0]), 'x': float(line[1]), 'y': float(line[2]),
                 'produce_remain_time': int((line[3])),
                 'material_state': int(line[4]),
                 'product_state': int(line[5])})

        # 读取4个机器人信息
        robot_list = []
        for i in range(4):
            line = sys.stdin.readline().strip().split(' ')
            # 附近工作台编号，整数，-1表示不在任何工作台附近
            # 携带物品类型，0表示未携带物品，1-7表示对应物品
            robot_list.append(
                {'id': i, 'near_work_bench_id': int(line[0]), 'carried_product_type': int(line[1]),
                 'time_cost': float(line[2]),
                 'crash_cost': float(line[3]),
                 'angle_speed': float(line[4]), 'line_speed_x': float(line[5]),
                 'line_speed_y': (float(line[6])), 'face_angle': float(line[7]),
                 'x': float(line[8]), 'y': float(line[9])})

        sys.stdout.write('%d\n' % frame_id)

        strategies_of_robots = strategy_greedy(work_bench_list, robot_list, strategies_of_robots)

        # 看看每个机器人能不能购买或售出物品
        for robot in robot_list:
            strategy = strategies_of_robots[robot['id']]
            print("机器人" + str(robot['id']) + ":" + str(robot), file=sys.stderr)
            print("策略：" + str(strategy), file=sys.stderr)
            # 如果该机器人没有被分配任务或者该机器人没有在任何工作台附近
            if not strategy or robot['near_work_bench_id'] == -1:
                continue
            # 否则，判断是否可以进行购买或出售动作
            else:
                # 如果机器人靠近目标取货工作台且手中没有东西且取货工作台有产品可取，则可以进行购买操作
                if robot['near_work_bench_id'] == strategy[0] and robot['carried_product_type'] == 0 and \
                        work_bench_list[strategy[0]]['product_state'] == 1:
                    sys.stdout.write('buy %d \n' % robot['id'])
                # 如果机器人手中有货且靠近目标销售工作台且目标工作台原料格空着
                elif robot['carried_product_type'] != 0 and robot['near_work_bench_id'] == strategy[1] and robot[
                    'carried_product_type'] & work_bench_list[strategy[1]]['material_state'] == 0:
                    sys.stdout.write('sell %d \n' % robot['id'])
                    strategies_of_robots[robot['id']] = []

        print("\n", file=sys.stderr)

        # 为每个机器人输出操作
        for robot in robot_list:
            strategy = strategies_of_robots[robot['id']]
            if not strategy:
                continue

            # 如果机器人手中无货，则前往目标取货台
            if robot['carried_product_type'] == 0:
                print("机器人" + str(robot['id']) + ":" + str(robot), file=sys.stderr)
                print("策略：" + str(strategy), file=sys.stderr)
                print("目标工作台:" + str(work_bench_list[strategy[0]]), file=sys.stderr)
                line_speed, angle_speed = move_to_xy(robot, work_bench_list[strategy[0]]['x'],
                                                     work_bench_list[strategy[0]]['y'])
                sys.stdout.write('forward %d %d\n' % (robot['id'], line_speed))
                sys.stdout.write('rotate %d %f\n' % (robot['id'], angle_speed))
                print("line_speed:" + str(line_speed), file=sys.stderr)
                print("angle_speed:" + str(angle_speed), file=sys.stderr)
                print("\n", file=sys.stderr)

            if robot['carried_product_type'] != 0:
                print("机器人" + str(robot['id']) + ":" + str(robot), file=sys.stderr)
                print("策略：" + str(strategy), file=sys.stderr)
                print("目标坐标(" + str(work_bench_list[strategy[0]]['x']) + "," + str(
                    work_bench_list[strategy[0]]['y']) + ")",
                      file=sys.stderr)
                line_speed, angle_speed = move_to_xy(robot, work_bench_list[strategy[1]]['x'],
                                                     work_bench_list[strategy[1]]['y'])
                sys.stdout.write('forward %d %d\n' % (robot['id'], line_speed))
                sys.stdout.write('rotate %d %f\n' % (robot['id'], angle_speed))
                print("line_speed:" + str(line_speed), file=sys.stderr)
                print("angle_speed:" + str(angle_speed), file=sys.stderr)
                print("\n", file=sys.stderr)

            # line_speed, angle_speed = 3, 1.5
            # for robot_id in range(4):
            #     sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
            #     sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        finish()

        read_util_ok()

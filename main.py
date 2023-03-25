#!/bin/bash
import sys
import time
import numpy as np
import math
from navigate import move_to_xy

from avoidCrash import crash_detect, avoid_crash_v2
from avoidCrash import countPreVec, avoid_wall

from strategies import strategy_greedy, strategy_greedy_for_map_34, strategy_greedy_for_map_2


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    # time.sleep(10)
    read_util_ok()  # 初始化完成
    finish()  # 初始化完成输出一个OK

    # 4，5，6,7类型工作台产物最近的销售地，key为4，5，6，7类型工作台id，value为最近销售目标工作台id和距离
    nearest_sell_place = {}

    strategies_of_robots = [{}, {}, {}, {}]

    while True:
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])  # 第一行：帧数 得分

        line = sys.stdin.readline().strip().split(' ')
        # 工作台数量
        num_of_work_bench = int(line[0])

        # print("帧数:" + str(frame_id) + "; 得分:" + str(parts[1]) + "工作台数量：" + str(num_of_work_bench), file=sys.stderr)
        # 下面K行为工作站信息
        work_bench_list = []
        work_bench_list_by_type = [[], [], [], [], [], [], [], [], []]
        # 各类型工作台统计信息，key为工作台类型，value为工作台类型统计信息，以字典形式存储，
        # 格式为{num_of_this_type,num_of_products,num_of_producing,num_of_material_type_x}
        work_bench_statistics_by_type = {
            '4': {'num_of_this_type': 0, 'num_of_products': 0, 'num_of_producing': 0, 'num_of_material_type_1': 0,
                  'num_of_material_type_2': 0},
            '5': {'num_of_this_type': 0, 'num_of_products': 0, 'num_of_producing': 0, 'num_of_material_type_1': 0,
                  'num_of_material_type_3': 0},
            '6': {'num_of_this_type': 0, 'num_of_products': 0, 'num_of_producing': 0, 'num_of_material_type_2': 0,
                  'num_of_material_type_3': 0},
            '7': {'num_of_this_type': 0, 'num_of_products': 0, 'num_of_producing': 0, 'num_of_material_type_4': 0,
                  'num_of_material_type_5': 0, 'num_of_material_type_6': 0}
        }
        for i in range(num_of_work_bench):
            line = sys.stdin.readline().strip().split(' ')
            # 工作台类型，整数，1-9
            # 坐标(x,y)，浮点数
            # 剩余生产时间，整数（单位：帧）
            # 原材料格状态，整数，二进制位表描述，例如 48(110000) 表示拥有物品 4 和 5。
            # 产品格状态，整数，0表示无，1表示有
            work_bench = {'id': i, 'type': int(line[0]), 'x': float(line[1]), 'y': float(line[2]),
                          'produce_remain_time': int((line[3])),
                          'material_state': int(line[4]),
                          'product_state': int(line[5])}
            work_bench_list.append(work_bench)
            if frame_id == 1:
                work_bench_list_by_type[work_bench_list[i]['type'] - 1].append(work_bench_list[i])
            # 针对图1的策略
            if num_of_work_bench == 43:
                if work_bench['type'] == 4:
                    work_bench_statistics_by_type['4']['num_of_this_type'] += 1
                    if work_bench['product_state'] == 1:
                        work_bench_statistics_by_type['4']['num_of_products'] += 1
                    if work_bench['produce_remain_time'] >= 0:
                        work_bench_statistics_by_type['4']['num_of_producing'] += 1
                    if work_bench['material_state'] & 2:
                        work_bench_statistics_by_type['4']['num_of_material_type_1'] += 1
                    if work_bench['material_state'] & 4:
                        work_bench_statistics_by_type['4']['num_of_material_type_2'] += 1
                elif work_bench['type'] == 5:
                    work_bench_statistics_by_type['5']['num_of_this_type'] += 1
                    if work_bench['product_state'] == 1:
                        work_bench_statistics_by_type['5']['num_of_products'] += 1
                    if work_bench['produce_remain_time'] >= 0:
                        work_bench_statistics_by_type['5']['num_of_producing'] += 1
                    if work_bench['material_state'] & 2:
                        work_bench_statistics_by_type['5']['num_of_material_type_1'] += 1
                    if work_bench['material_state'] & 8:
                        work_bench_statistics_by_type['5']['num_of_material_type_3'] += 1
                elif work_bench['type'] == 6:
                    work_bench_statistics_by_type['6']['num_of_this_type'] += 1
                    if work_bench['product_state'] == 1:
                        work_bench_statistics_by_type['6']['num_of_products'] += 1
                    if work_bench['produce_remain_time'] >= 0:
                        work_bench_statistics_by_type['6']['num_of_producing'] += 1
                    if work_bench['material_state'] & 4:
                        work_bench_statistics_by_type['6']['num_of_material_type_2'] += 1
                    if work_bench['material_state'] & 8:
                        work_bench_statistics_by_type['6']['num_of_material_type_3'] += 1
                elif work_bench['type'] == 7:
                    work_bench_statistics_by_type['7']['num_of_this_type'] += 1
                    if work_bench['product_state'] == 1:
                        work_bench_statistics_by_type['7']['num_of_products'] += 1
                    if work_bench['produce_remain_time'] >= 0:
                        work_bench_statistics_by_type['7']['num_of_producing'] += 1
                    if work_bench['material_state'] & 16:
                        work_bench_statistics_by_type['7']['num_of_material_type_4'] += 1
                    if work_bench['material_state'] & 32:
                        work_bench_statistics_by_type['7']['num_of_material_type_5'] += 1
                    if work_bench['material_state'] & 64:
                        work_bench_statistics_by_type['7']['num_of_material_type_6'] += 1
        # 只需要在第一帧的时候记录nearest_sell_place就行
        if frame_id == 1:
            for work_bench in work_bench_list:
                if work_bench['type'] == 4:
                    work_bench_xy = np.array([work_bench['x'], work_bench['y']])
                    nearest_sell_place[work_bench['id']] = {'id': -1, 'distance': 9999}
                    for other_work_bench in work_bench_list_by_type[5]:
                        other_work_bench_xy = np.array([other_work_bench['x'], other_work_bench['y']])
                        distance = np.linalg.norm(work_bench_xy - other_work_bench_xy)
                        if distance < nearest_sell_place[work_bench['id']]['distance']:
                            nearest_sell_place[work_bench['id']] = {'id': other_work_bench['id'], 'distance': distance}
                if work_bench['type'] == 5:
                    work_bench_xy = np.array([work_bench['x'], work_bench['y']])
                    nearest_sell_place[work_bench['id']] = {'id': -1, 'distance': 9999}
                    for other_work_bench in work_bench_list_by_type[6]:
                        other_work_bench_xy = np.array([other_work_bench['x'], other_work_bench['y']])
                        distance = np.linalg.norm(work_bench_xy - other_work_bench_xy)
                        if distance < nearest_sell_place[work_bench['id']]['distance']:
                            nearest_sell_place[work_bench['id']] = {'id': other_work_bench['id'], 'distance': distance}
                if work_bench['type'] == 6:
                    work_bench_xy = np.array([work_bench['x'], work_bench['y']])
                    nearest_sell_place[work_bench['id']] = {'id': -1, 'distance': 9999}
                    for other_work_bench in work_bench_list_by_type[7]:
                        other_work_bench_xy = np.array([other_work_bench['x'], other_work_bench['y']])
                        distance = np.linalg.norm(work_bench_xy - other_work_bench_xy)
                        if distance < nearest_sell_place[work_bench['id']]['distance']:
                            nearest_sell_place[work_bench['id']] = {'id': other_work_bench['id'], 'distance': distance}
                if work_bench['type'] == 7:
                    work_bench_xy = np.array([work_bench['x'], work_bench['y']])
                    nearest_sell_place[work_bench['id']] = {'id': -1, 'distance': 9999}
                    for other_work_bench in work_bench_list_by_type[8]:
                        other_work_bench_xy = np.array([other_work_bench['x'], other_work_bench['y']])
                        distance = np.linalg.norm(work_bench_xy - other_work_bench_xy)
                        if distance < nearest_sell_place[work_bench['id']]['distance']:
                            nearest_sell_place[work_bench['id']] = {'id': other_work_bench['id'], 'distance': distance}

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
                 'angle_speed': float(line[4]),
                 'line_speed_x': float(line[5]), 'line_speed_y': (float(line[6])),
                 'line_speed': math.sqrt(math.pow(float(line[5]), 2) + math.pow(float(line[6]), 2)),
                 'face_angle': float(line[7]),
                 'x': float(line[8]), 'y': float(line[9]),
                 'destination': np.array([0, 0]), 'rotate_state': 0.0, 'forward_state': 0.0,
                 'angle_speed_up': 100 / (math.pi * 20 * pow(0.45, 4)) if int(line[1]) == 0 else 100 / (
                         math.pi * 20 * pow(0.53, 4)),
                 'line_speed_up': 250 / (math.pi * 20 * pow(0.45, 2)) if int(line[1]) == 0 else 100 / (
                         math.pi * 20 * pow(0.53, 4))},
            )

        sys.stdout.write('%d\n' % frame_id)

        if num_of_work_bench == 43:
            strategies_of_robots = strategy_greedy(work_bench_list, robot_list, strategies_of_robots, frame_id,
                                                   nearest_sell_place, work_bench_statistics_by_type)
        if num_of_work_bench == 25:
            strategies_of_robots = strategy_greedy_for_map_2(work_bench_list, robot_list, strategies_of_robots,
                                                             frame_id,
                                                             nearest_sell_place)

        else:
            strategies_of_robots = strategy_greedy_for_map_34(work_bench_list, robot_list, strategies_of_robots,
                                                              frame_id,
                                                              nearest_sell_place)

        # 看看每个机器人能不能购买或售出物品
        # print("此处为调试购买或售出物品判断：", file=sys.stderr)
        for robot in robot_list:
            strategy = strategies_of_robots[robot['id']]
            # print("机器人" + str(robot['id']) + ":" + str(robot), file=sys.stderr)
            # print("策略：" + str(strategy), file=sys.stderr)
            # print("\n", file=sys.stderr)
            # 如果该机器人没有被分配任务或者该机器人没有在任何工作台附近
            if not strategy or robot['near_work_bench_id'] == -1:
                continue
            # 否则，判断是否可以进行购买或出售动作
            else:
                # print(robot['carried_product_type'], file=sys.stderr)
                # print(work_bench_list[strategy[1]]['material_state'], file=sys.stderr)
                # print(robot['carried_product_type'] & work_bench_list[strategy[1]]['material_state'], file=sys.stderr)
                # 如果机器人靠近目标取货工作台且手中没有东西且取货工作台有产品可取，则可以进行购买操作
                if robot['near_work_bench_id'] == strategy['departure_work_bench_id'] and robot[
                    'carried_product_type'] == 0 and \
                        work_bench_list[strategy['departure_work_bench_id']]['product_state'] == 1:
                    sys.stdout.write('buy %d \n' % robot['id'])
                    strategies_of_robots[robot['id']]['carried_product_type'] = strategy['product_type']
                # 如果机器人手中有货且靠近目标销售工作台且目标工作台原料格空着
                elif robot['carried_product_type'] != 0 and robot['near_work_bench_id'] == strategy[
                    'destination_work_bench_id'] and 1 << robot[
                    'carried_product_type'] & work_bench_list[strategy['destination_work_bench_id']][
                    'material_state'] == 0:
                    sys.stdout.write('sell %d \n' % robot['id'])
                    strategies_of_robots[robot['id']] = {}

        # 为每个机器人输出操作
        # print("此处为机器人决策信息：", file=sys.stderr)
        for robot in robot_list:
            strategy = strategies_of_robots[robot['id']]
            if not strategy:
                continue

            # 如果机器人手中无货，则前往目标取货台
            if robot['carried_product_type'] == 0:
                # print("机器人" + str(robot['id']) + ":" + str(robot), file=sys.stderr)
                # print("策略：" + str(strategy), file=sys.stderr)
                # print("目标工作台:" + str(work_bench_list[strategy[0]]), file=sys.stderr)
                robot['destination'] = np.array([work_bench_list[strategy['departure_work_bench_id']]['x'],
                                                 work_bench_list[strategy['departure_work_bench_id']]['y']])
                # line_speed, angle_speed = move_to_xy(robot, work_bench_list[strategy['departure_work_bench_id']]['x'],
                #                                      work_bench_list[strategy['departure_work_bench_id']]['y'],
                #                                      robot_list)
                line_speed, angle_speed = move_to_xy(robot, robot['destination'][0], robot['destination'][1],
                                                     robot_list)
                pre_speed = countPreVec(robot, work_bench_list, strategy['departure_work_bench_id'])
                line_speed, angle_speed = avoid_wall(robot, line_speed, angle_speed, pre_speed)
                robot['rotate_state'] = angle_speed
                robot['forward_state'] = line_speed


                # sys.stdout.write('forward %d %d\n' % (robot['id'], line_speed))
                # sys.stdout.write('rotate %d %f\n' % (robot['id'], angle_speed))
                # print("\n", file=sys.stderr)

            # 如果机器人手中有，则前往目标送货台
            if robot['carried_product_type'] != 0:
                # print("机器人" + str(robot['id']) + ":" + str(robot), file=sys.stderr)
                # print("策略：" + str(strategy), file=sys.stderr)
                # print("目标工作台:" + str(work_bench_list[strategy[1]]), file=sys.stderr)
                robot['destination'] = np.array([work_bench_list[strategy['destination_work_bench_id']]['x'],
                                                 work_bench_list[strategy['destination_work_bench_id']]['y']])
                # line_speed, angle_speed = move_to_xy(robot, work_bench_list[strategy['destination_work_bench_id']]['x'],
                #                                      work_bench_list[strategy['destination_work_bench_id']]['y'],
                #                                      robot_list)
                line_speed, angle_speed = move_to_xy(robot, robot['destination'][0], robot['destination'][1],
                                                     robot_list)
                pre_speed = countPreVec(robot, work_bench_list, strategy['destination_work_bench_id'])
                line_speed, angle_speed = avoid_wall(robot, line_speed, angle_speed, pre_speed)
                robot['rotate_state'] = angle_speed
                robot['forward_state'] = line_speed

                # sys.stdout.write('forward %d %d\n' % (robot['id'], line_speed))
                # sys.stdout.write('rotate %d %f\n' % (robot['id'], angle_speed))
                # print("\n", file=sys.stderr)

        # 碰撞检测
        crash_list = crash_detect(robot_list, crash_detect_distance=10)
        if crash_list:
            avoid_crash_v2(robot_list, crash_list, frame_id)

        # if 1215 <= frame_id <= 1220:
        #     print(frame_id, file=sys.stderr)
        #     print(crash_list, file=sys.stderr)
        #     print(robot_list, file=sys.stderr)

        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, robot_list[robot_id]['forward_state']))
            sys.stdout.write('rotate %d %f\n' % (robot_id, robot_list[robot_id]['rotate_state']))
        finish()

        read_util_ok()

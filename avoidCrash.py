import numpy as np
import math
import sys


def outer(arr1, arr2):  # 计算外积,用于判断转向的方向
    return arr1[0] * arr2[1] - arr1[1] * arr2[0]


def countPreVec(robot, work_bench_list, work_bench_id):
    robot_xy = np.array([robot['x'], robot['y']])  # 机器人坐标
    des_xy = np.array([work_bench_list[work_bench_id]['x'],
                       work_bench_list[work_bench_id]['y']])  # 终点坐标
    xy_robot_vec = des_xy - robot_xy  # xy-机器人坐标向量
    xy_robot_vec_angle = math.atan2(xy_robot_vec[1], xy_robot_vec[0])  # xy-机器人向量朝向
    pre_speed = np.array([math.cos(xy_robot_vec_angle) * 6, math.sin(xy_robot_vec_angle) * 6])
    return pre_speed


def avoid_wall(robot, line_speed, angle_speed, pre_speed):
    line_speed_new = line_speed
    angle_speed_new = angle_speed
    # if robot['carried_product_type']:
    #     if (robot['x'] <= 0.6 and robot['y'] <= 0.6 and robot['face_angle'] >= -np.pi - 0.1 and robot[
    #         'face_angle'] <= -np.pi / 2 + 0.1) \
    #             or (robot['x'] <= 0.6 and robot['y'] >= 49.4 and robot['face_angle'] >= np.pi / 2 - 0.1 and robot[
    #         'face_angle'] <= np.pi + 0.1) \
    #             or (robot['x'] >= 49.4 and robot['y'] <= 0.6 and robot['face_angle'] >= -np.pi / 2 - 0.1 and robot[
    #         'face_angle'] <= 0 + 0.1) \
    #             or (robot['x'] >= 49.4 and robot['y'] >= 49.4 and robot['face_angle'] >= 0 - 0.1 and robot[
    #         'face_angle'] <= np.pi / 2 + 0.1):
    #         line_speed_new = 0.2
    #         angle_speed = (angle_speed/line_speed_new)*robot['carried_product_type']*2
    #         return line_speed_new, angle_speed_new
    angle_pre_speed = math.atan2(pre_speed[1], pre_speed[0])
    if robot['x'] <= 0.7 and (
            robot['face_angle'] > -np.pi and robot['face_angle'] < -np.pi / 2 or robot['face_angle'] > np.pi / 2 and
            robot['face_angle'] < np.pi):
        # angle_delta = min(abs(robot['face_angle']+np.pi/2), abs(np.pi/2-robot['face_angle']))

        # if angle_delta == abs(robot['face_angle']+np.pi/2):
        #     x = 1
        # else:
        #     x = -1
        angle_delta = angle_pre_speed - robot['face_angle']
        if angle_delta > 0:
            x = -1
        elif angle_delta < 0:
            x = 1
        else:
            x = 0
        angle_speed_new = (angle_delta / 0.01) * x * 2
        line_speed_new = 0
    if robot['x'] >= 49.3 and robot['face_angle'] > -np.pi / 2 and robot['face_angle'] < np.pi / 2:
        # angle_delta = min(abs(robot['face_angle'] + np.pi / 2), abs(np.pi / 2 - robot['face_angle']))
        # if angle_delta == abs(robot['face_angle'] + np.pi / 2):
        #     x = -1
        # else:
        #     x = 1
        angle_delta = angle_pre_speed - robot['face_angle']
        if angle_delta > 0:
            x = -1
        elif angle_delta < 0:
            x = 1
        else:
            x = 0
        angle_speed_new = (angle_delta / 0.01) * x * 2
        line_speed_new = 0
    if robot['y'] <= 0.7 and robot['face_angle'] > -np.pi and robot['face_angle'] < 0:
        # angle_delta = min(abs(robot['face_angle'] - np.pi), abs( robot['face_angle']))
        # if angle_delta == abs(robot['face_angle'] - np.pi):
        #     x = 1
        # else:
        #     x = -1
        angle_delta = angle_pre_speed - robot['face_angle']
        if angle_delta > 0:
            x = -1
        elif angle_delta < 0:
            x = 1
        else:
            x = 0
        angle_speed_new = (angle_delta / 0.01) * x * 2
        line_speed_new = 0
    if robot['y'] >= 49.3 and robot['face_angle'] > 0 and robot['face_angle'] < np.pi:
        # angle_delta = min(abs(robot['face_angle'] + np.pi ), abs(robot['face_angle']))
        # if angle_delta == abs(robot['face_angle'] + np.pi):
        #     x = -1
        # else:
        #     x = 1
        angle_delta = angle_pre_speed - robot['face_angle']
        if angle_delta > 0:
            x = -1
        elif angle_delta < 0:
            x = 1
        else:
            x = 0
        angle_speed_new = (angle_delta / 0.01) * x * 2
        line_speed_new = 0
    return line_speed_new, angle_speed_new


def avoid_crash(robot, other_robot_list, pre_speed, line_speed, angle_speed):
    d_min = 1.0
    line_speed_new = line_speed
    angle_speed_new = angle_speed
    crash_robots = []
    for other_robot in other_robot_list:
        if (float(other_robot['x']) - float(robot['x'])) ** 2 + (
                float(other_robot['y']) - float(robot['y'])) ** 2 <= d_min ** 2:
            crash_robots.append(other_robot)
    if crash_robots:
        line_speed_new, angle_speed_new = orca(robot, crash_robots, 0.02, 1.3, pre_speed)
    return line_speed_new, angle_speed_new


# 安全范围检测,与crash_detect类似,返回一个可能与当前机器人发生碰撞的列表
def safe_detect(robot, robot_list, safe_radius=2):
    # 碰撞检测阈值，距离小于该值开始检测

    crash_list = []  # 碰撞列表

    for i in range(len(robot_list)):  # 遍历整个机器人列表,robot为待检测
        if i == robot['id']: continue
        robot_2 = robot_list[i]
        # 机器人的坐标
        robot_1_xy = np.array([robot['x'], robot['y']])
        robot_2_xy = np.array([robot_2['x'], robot_2['y']])
        robot_12_vec = robot_2_xy - robot_1_xy  # 从1到2的向量

        if np.linalg.norm(robot_12_vec) > safe_radius: continue  # 距离过远不检测

        # 机器人的速度v向量
        robot_1_v = np.array([robot_1['line_speed_x'], robot_1['line_speed_y']])
        robot_2_v = np.array([robot_2['line_speed_x'], robot_2['line_speed_y']])

        relative_speed = robot_1_v - robot_2_v  # 相对速度

        relative_speed_angle = math.atan2(relative_speed[1], relative_speed[0])  # 相对速度方向
        robot_12_angle = math.atan2(robot_12_vec[1], robot_12_vec[0])  # 两机器人连线方向12

        crash_threshold = 2 * np.arcsin(0.45 / crash_detect_distance) * 1.4  # 计算临界碰撞角度

        if abs(relative_speed_angle - robot_12_angle) < crash_threshold:  # 碰撞情况
            crash_list.append([robot_1['id'], robot_2['id']])

    return crash_list


# 检测碰撞函数，返回一个碰撞列表，表中元素表示会发生碰撞的机器人，如[[0, 1], [2, 3]]
def crash_detect(robot_list, crash_detect_distance=4):
    # 碰撞检测阈值，距离小于该值开始检测

    crash_list = []  # 碰撞列表

    for i in range(len(robot_list)):  # 遍历整个机器人列表,robot为待检测
        for j in range(i + 1, len(robot_list)):
            robot_1 = robot_list[i]
            robot_2 = robot_list[j]
            # 机器人的坐标
            robot_1_xy = np.array([robot_1['x'], robot_1['y']])
            robot_2_xy = np.array([robot_2['x'], robot_2['y']])
            robot_12_vec = robot_2_xy - robot_1_xy  # 从1到2的向量
            robot_12_dis = np.linalg.norm(robot_12_vec)

            if robot_12_dis > crash_detect_distance: continue  # 距离过远不检测

            # 机器人的速度v向量
            robot_1_v = np.array([robot_1['line_speed_x'], robot_1['line_speed_y']])
            robot_2_v = np.array([robot_2['line_speed_x'], robot_2['line_speed_y']])

            relative_speed = robot_1_v - robot_2_v  # 相对速度

            relative_speed_angle = math.atan2(relative_speed[1], relative_speed[0])  # 相对速度方向
            robot_12_angle = math.atan2(robot_12_vec[1], robot_12_vec[0])  # 两机器人连线方向12

            # crash_threshold = 2 * np.arcsin(0.45 / crash_detect_distance) * 1.4  # 计算临界碰撞角度
            crash_threshold = 2 * np.arcsin(0.45 / robot_12_dis) * 1.4  # 计算临界碰撞角度

            if abs(relative_speed_angle - robot_12_angle) < crash_threshold:  # 碰撞情况
                crash_list.append([robot_1['id'], robot_2['id']])

    return crash_list


# 已经碰撞的检测
def already_crash_detect(robot_list, crash_detect_distance=0.45 * 2):
    already_crash_list = []

    for i in range(len(robot_list)):  # 遍历整个机器人列表,robot为待检测
        for j in range(i + 1, len(robot_list)):
            robot_1 = robot_list[i]
            robot_2 = robot_list[j]
            # if not (robot_1['carried_product_type'] == 0) == (robot_2['carried_product_type'] == 0): continue
            # 机器人的坐标
            robot_1_xy = np.array([robot_1['x'], robot_1['y']])
            robot_2_xy = np.array([robot_2['x'], robot_2['y']])
            # 距离小于阈值
            if np.linalg.norm(robot_1_xy - robot_2_xy) <= crash_detect_distance:
                already_crash_list.append([robot_1['id'], robot_2['id']])

    return already_crash_list


# 对于已经碰上的机器人
def move_back(robot_list, already_crash_list):
    backward_list = [0, 0, 0, 0]  # 每个机器人只后退一次
    for crash in already_crash_list:
        robot_0 = robot_list[crash[0]]
        robot_1 = robot_list[crash[1]]
        if np.pi - 2 * np.pi / 6 < abs(robot_0['face_angle'] - robot_1['face_angle']) < np.pi + 2 * np.pi:
            sys.stdout.write('forward %d %f\n' % (crash[0], -0.5))
            sys.stdout.write('forward %d %f\n' % (crash[1], -0.5))
        # sys.stdout.write('forward %d %f\n' % (crash_id, -0.5))


# 检测到碰撞后的转向
def avoid_crash_v2(robot_list, crash_list):
    rotate_list = [0, 0, 0, 0]  # 机器人在防碰撞策略中的转向次数
    slow_list = [0, 0, 0, 0]  # 机器人在防碰撞策略中的减速次数

    for crash in crash_list:

        # 如果一个带货物一个不带货物，不带货物的要避让带货物的
        if (robot_list[crash[0]]['carried_product_type'] == 0 or robot_list[crash[1]]['carried_product_type'] == 0) and \
                robot_list[crash[0]]['carried_product_type'] != robot_list[crash[1]]['carried_product_type']:
            avoid_index = 0 if robot_list[crash[0]]['carried_product_type'] == 0 else 1

            # 1为执行避让的机器人，2为不避让的机器人
            robot_1 = robot_list[avoid_index]
            robot_2 = robot_list[avoid_index == 0]

            # 机器人朝向向量
            robot1_face_vec = np.array([np.cos(robot_1['face_angle']), np.sin(robot_1['face_angle'])])
            robot2_face_vec = np.array([np.cos(robot_2['face_angle']), np.sin(robot_2['face_angle'])])

            # 如果携带货物的机器人正在转向
            if robot_2['rotate_state'] != 0:
                robot_1['rotate_state'] = 4.0 * abs(robot_2['rotate_state']) / robot_2['rotate_state']
            else:  # 如果携带货物的机器人正在直行
                if outer(robot1_face_vec, robot2_face_vec) >= 0:
                    robot_1['rotate_state'] = -4.0
                else:
                    robot_1['rotate_state'] = 4.0

        # 若两个机器人均为直行
        if robot_list[crash[0]]['rotate_state'] == robot_list[crash[1]]['rotate_state'] == 0:
            robot_list[crash[0]]['rotate_state'] = robot_list[crash[0]]['face_angle'] + 1
            robot_list[crash[1]]['rotate_state'] = robot_list[crash[1]]['face_angle'] + 1
            rotate_list[crash[0]] += 1
            rotate_list[crash[1]] += 1

        # 若一个直行一个转向
        elif robot_list[crash[0]]['rotate_state'] == 0.0 or robot_list[crash[1]]['rotate_state'] == 0.0:
            # 记录转向机器人在crash中的下标
            if robot_list[crash[0]]['rotate_state'] == 0.0:
                avoid_index = 1
            else:
                avoid_index = 0
            # 不直行的那个要转向或减速
            # 若转向的机器人转向速度已到极限，则减速
            if np.abs(robot_list[crash[avoid_index]]['rotate_state']) == 4.0:
                old_line_speed = np.linalg.norm(
                    np.array([robot_list[crash[avoid_index]]['line_speed_x'],
                              robot_list[crash[avoid_index]]['line_speed_y']]))
                # old_line_speed = robot_list[crash[avoid_index]]['forward_state']
                # sys.stdout.write('forward %d %f\n' % (crash[avoid_index], old_line_speed))
                robot_list[crash[avoid_index]]['forward_state'] = old_line_speed
                # robot_list[crash[avoid_index]]['rotate_state'] = -robot_list[crash[avoid_index]]['rotate_state']
                slow_list[crash[avoid_index]] += 1
            else:  # 若转向机器人转向速度未到极限
                # sys.stdout.write('rotate %d %f\n' % (crash[avoid_index], 4.0 *
                #                                      np.abs(robot_list[crash[avoid_index]]['rotate_state']) /
                #                                      robot_list[crash[avoid_index]]['rotate_state']))
                robot_list[crash[avoid_index]]['rotate_state'] = np.abs(
                    robot_list[crash[avoid_index]]['rotate_state']) / robot_list[crash[avoid_index]]['rotate_state']
                rotate_list[crash[avoid_index]] += 1

            # if robot_list[crash[avoid_index]]['rotate_state'] != robot_list[crash[avoid_index]]['face_angle']:


        # 若两个都转向
        else:
            # 计算两个机器人的避让次数，次数低的优先避让
            if rotate_list[crash[0]] + slow_list[crash[0]] <= rotate_list[crash[1]] + slow_list[crash[1]] and \
                    robot_list[crash[0]]['forward_state'] != 0:  # 避让的那个机器人速度不能为0
                avoid_index = 0
            else:
                avoid_index = 1
            old_line_speed = np.linalg.norm(
                np.array(
                    [robot_list[crash[avoid_index]]['line_speed_x'], robot_list[crash[avoid_index]]['line_speed_y']]))
            old_line_speed = robot_list[crash[avoid_index]]['forward_state']
            robot_list[crash[avoid_index]]['forward_state'] = old_line_speed
            robot_list[crash[avoid_index]]['rotate_state'] = -robot_list[crash[avoid_index]]['rotate_state']

            # robot_list[crash[0]]['rotate_state'] = -robot_list[crash[0]]['rotate_state']
            # robot_list[crash[1]]['rotate_state'] = -robot_list[crash[1]]['rotate_state']

            rotate_list[crash[avoid_index]] += 1

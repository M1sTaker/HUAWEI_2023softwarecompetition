import numpy as np
import math


def outer(arr1, arr2):  # 计算外积,用于判断转向的方向
    return arr1[0] * arr2[1] - arr1[1] * arr2[0]


def move_to_xy(robot, des_x, des_y, robot_list):
    robot_face_angle = robot['face_angle']  # 机器人朝向
    robot_face_vec = np.array([np.cos(robot_face_angle), np.sin(robot_face_angle)])  # 机器人朝向向量

    robot_xy = np.array([robot['x'], robot['y']])  # 机器人坐标
    des_xy = np.array([des_x, des_y])  # 终点坐标

    xy_robot_vec = des_xy - robot_xy  # xy-机器人坐标向量
    xy_robot_vec_angle = math.atan2(xy_robot_vec[1], xy_robot_vec[0])  # xy-机器人向量朝向

    distance = np.linalg.norm(robot_xy - des_xy)  # 计算距离

    # 朝向：弧度[-pi，pi]  速度：m/s [-2,6]  旋转速度：弧度/s [-pi,pi]
    if (np.dot(robot_face_vec, xy_robot_vec) >= 0):  # 机器人面向向量与(工作台-机器人)向量小于90°
        if (abs(robot_face_angle - xy_robot_vec_angle) <= 0.2):  # 夹角小于阈值时不改变方向
            for other_robot in robot_list:
                if robot['id'] == other_robot['id']:
                    continue
                other_robot_xy = np.array([robot['x'], robot['y']])
                if abs(other_robot['face_angle'] - robot['face_angle']) < math.pi - 0.1 and np.linalg.norm(
                        robot_xy - other_robot_xy) < 8:
                    return 6.0, -1
            return 6.0, 0
        line_speed = 6.0
        if (outer(robot_face_vec, xy_robot_vec) <= 0):  # 机器人向右转(顺时针)
            angle_speed = -4
        else:  # 机器人向左转(逆时针)
            angle_speed = 4
    else:# 机器人面向向量与(工作台-机器人)向量大于90°
        line_speed = 2.0
        if (outer(robot_face_vec, xy_robot_vec) <= 0):  # 机器人向右转(顺时针)
            angle_speed = -9.0
        else:  # 机器人向左转(逆时针)
            angle_speed = 9.0

    if (distance < 1.5):  # 快到终点时减速
        line_speed = 2.0
    return line_speed, angle_speed

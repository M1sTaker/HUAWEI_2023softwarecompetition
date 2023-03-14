import numpy as np
import math

def move_to_xy(robot, des_x, des_y):
    robot_face_angle = robot['face_angle']#机器人朝向

    robot_xy = np.array([robot['x'], robot['y']])#机器人坐标
    des_xy = np.array([des_x, des_y])#终点坐标

    xy_robot_vec = des_xy - robot_xy# xy-机器人坐标向量
    xy_robot_vec_angle = math.atan2(xy_robot_vec[1] , xy_robot_vec[0])# xy-机器人向量朝向

    distance = np.linalg.norm(robot_xy - des_xy)#计算距离


    # 机器人面向向量与(工作台-机器人)向量小于90°
    if (np.abs(robot_face_angle - xy_robot_vec_angle) <= np.pi/2):
        if(robot_face_angle == xy_robot_vec_angle):# 夹角为0不改变方向
            return 9999.0, 0
        line_speed = 9999.0
        if(robot_face_angle > xy_robot_vec_angle):# 机器人向右转(顺时针)
            angle_speed = -3.0
        else:
            angle_speed = 3.0
    else:# 机器人面向向量与(工作台-机器人)向量大于90°
        line_speed = 9999
        if (robot_face_angle > xy_robot_vec_angle):  # 机器人向右转(顺时针)
            angle_speed = -3.0
        else:
            angle_speed = 3.0

    if(distance < 0.8): #快到终点时减速
        line_speed = 1

    return line_speed, angle_speed

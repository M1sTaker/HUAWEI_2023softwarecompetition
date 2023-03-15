import numpy as np
import math

def move_to_xy(robot, des_x, des_y):
    robot_face_angle = robot['face_angle']#机器人朝向

    robot_xy = np.array([robot['x'], robot['y']])#机器人坐标
    des_xy = np.array([des_x, des_y])#终点坐标

    xy_robot_vec = des_xy - robot_xy# xy-机器人坐标向量
    xy_robot_vec_angle = math.atan2(xy_robot_vec[1] , xy_robot_vec[0])# xy-机器人向量朝向

    distance = np.linalg.norm(robot_xy - des_xy)#计算距离

    #朝向：弧度[-pi，pi]  速度：m/s [-2,6]  旋转速度：弧度/s [-pi,pi]
    # 机器人面向向量与(工作台-机器人)向量小于90°
    if (np.abs(robot_face_angle - xy_robot_vec_angle) <= np.pi/2):
        if(robot_face_angle == xy_robot_vec_angle):# 夹角为0不改变方向
            return 6.0, 0
        line_speed = 5.0
        if(robot_face_angle > xy_robot_vec_angle):# 机器人向右转(顺时针)
            angle_speed = -2.0
        else:# 机器人向左转(逆时针)
            angle_speed = 2.0
    else:# 机器人面向向量与(工作台-机器人)向量大于90°
        line_speed = 3.0
        if (abs(robot_face_angle) + abs(xy_robot_vec_angle) < np.pi):  # 机器人向右转(顺时针)
            angle_speed = -2.0
        else:
            angle_speed = 2.0

    if(distance < 0.5): #快到终点时减速
        line_speed = 0.1
        if(distance < 0.2): #距离小于一定值时停止
            line_speed = 0
            angle_speed = 0

    return line_speed, angle_speed

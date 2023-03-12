import numpy as np

def move_to_xy(robot, des_x, des_y):
    robot_face_angle = robot['face_angle']

    robot_xy = np.array(robot['x'], robot['y'])#机器人坐标
    des_xy = np.array(des_x, des_y)#终点坐标

    face_vec = np.array(robot['line_speed_x'], robot['line_speed_y']) # 机器人面向向量
    xy_robot_vec = des_xy - robot_xy# xy-机器人坐标向量

    distance = np.linalg.norm(robot_xy - des_xy)#计算距离

    if(np.dot(face_vec, xy_robot_vec.T) >= 0):# 机器人面向向量与(工作台-机器人)向量小于90°
        if(robot_face_angle == np.arctan(xy_robot_vec[1] / xy_robot_vec[0])):# 夹角为0不改变方向
            return 9999, 0
        line_speed = 9999
        if(robot_face_angle > np.arctan(xy_robot_vec[1] / xy_robot_vec[0])):# 机器人向右转(顺时针)
            angle_speed = -np.pi
        else:
            angle_speed = np.pi
    else:# 机器人面向向量与(工作台-机器人)向量大于90°
        line_speed = 0.1
        if (robot_face_angle > np.arctan(xy_robot_vec[1] / xy_robot_vec[0])):  # 机器人向右转(顺时针)
            angle_speed = -np.pi
        else:
            angle_speed = np.pi

    if(distance < 0.8): #快到终点时减速
        line_speed = 0.1

    return line_speed, angle_speed

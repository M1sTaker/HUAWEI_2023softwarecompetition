import numpy as np
import math
def avoid_crash(robot, line_speed, angle_speed,all_robot):
    line_speed_new = line_speed
    angle_speed_new = angle_speed
    d_min = 0.45
    for i in range(4):
        if i == robot['id']:
            continue
        if (float(all_robot[i]['x'])-float(robot['x']))**2+(float(all_robot[i]['y'])-float(robot['y']))**2 <= d_min**2:
            AB = np.array([float(all_robot[i]['x'])-float(robot['x']), float(all_robot[i]['y'])-float(robot['y'])]) #本机器人指向碰撞机器人的向量，简称L1
            BA = np.array([float(robot['x'])-float(all_robot[i]['x']), float(robot['y'])-float(all_robot[i]['y'])]) #碰撞机器人指向本机器人的向量，简称L2
            theta1 = robot['face_angle'] #本机器人的朝向角
            D1 = np.array([math.cos(theta1), math.sin(theta1)]) #本机器人的朝向向量
            theta2 = all_robot[i]['face_angle']  #碰撞机器人的朝向角
            D2 = np.array([math.cos(theta2),math.sin(theta2)]) #碰撞机器人的碰撞向量
            theta = robot['face_angle']-all_robot[i]['face_angle']  #两个机器人朝向的夹角
            cos_A1 = np.dot(AB, D1)/(np.linalg.norm(AB)*np.linalg.norm(D1))
            cos_B2 = np.dot(BA, D2)/(np.linalg.norm(BA)*np.linalg.norm(D2))
            cos_theta = math.cos(theta)
            if cos_A1==1 & cos_B2==1:
                line_speed_new = 0.5
                angle_speed_new = 1
                return line_speed_new, angle_speed_new
            if cos_A1>0:
                cos_A1 = 1
            if cos_A1<0:
                cos_A1 = -1
            if cos_B2>0:
                cos_B2 = 1
            if cos_B2<0:
                cos_B2 = -1
            if cos_theta > 0:
                cos_theta = 1
            if cos_theta < 0:
                cos_theta = -1
            if cos_A1+cos_B2+cos_theta>=2:
                line_speed_new = 0.5
                angle_speed_new = 0
                return line_speed_new, angle_speed_new
    return line_speed_new, angle_speed_new
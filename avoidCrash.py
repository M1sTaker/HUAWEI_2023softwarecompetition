import numpy as np
import math
import sys
import random
def avoid_crash(robot, robot_id, line_speed, angle_speed,all_robot):
    line_speed_new = line_speed
    angle_speed_new = angle_speed
    line_speed_new_list = [2,4,6]
    # angle_speed_new_list = [-2,0,1,2]
    d_min = 2.5

    if robot['carried_product_type']:
        if (robot['x'] <= 0.5 and robot['y'] <= 0.5 and  robot['face_angle'] >= -np.pi-0.1 and robot['face_angle'] <= -np.pi/2+0.1)\
                or (robot['x'] <= 0.5 and robot['y'] >=49.5 and  robot['face_angle'] >= np.pi/2-0.1 and robot['face_angle'] <= np.pi+0.1) \
                or (robot['x'] >=49.5 and robot['y'] <= 0.5 and  robot['face_angle'] >= -np.pi/2-0.1 and robot['face_angle'] <= 0+0.1) \
                or (robot['x'] >=49.5 and robot['y'] >=49.5 and  robot['face_angle'] >= 0-0.1 and robot['face_angle'] <= np.pi/2+0.1):
            line_speed_new = 1
            # if robot['face_angle'] >= 0:
            #     angle_speed_new = -3
            # else:
            #     angle_speed_new = 3
            return line_speed_new, angle_speed_new
        if robot['x']<=0.5 and (robot['face_angle']>-np.pi/2 and robot['face_angle']<0 or robot['face_angle']>np.pi/2 and robot['face_angle']<np.pi)\
                or robot['x'] >=49.5 and robot['face_angle']>-np.pi/2 and robot['face_angle']<np.pi/2\
                or robot['y']<=0.5 and robot['face_angle']>-np.pi and robot['face_angle']<0\
                or robot['y'] >= 49.5 and robot['face_angle']>0 and robot['face_angle']<np.pi:
            line_speed_new = 1.5
            if robot['face_angle']>=0:
                angle_speed_new = 2+angle_speed
            else:
                angle_speed_new = -2+angle_speed
            return line_speed_new, angle_speed_new
    for i in range(4):
        if i == robot_id:
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
            AB_len = math.sqrt((float(all_robot[i]['x'])-float(robot['x']))**2+(float(all_robot[i]['y'])-float(robot['y']))**2)
            D1_m = (abs(BA[0]*D2[1]-BA[1]*D2[0])/(np.linalg.norm(BA)*np.linalg.norm(D2)))*(math.sin(theta)/AB_len)
            D2_m = (abs(AB[0]*D1[1]-AB[1]*D1[0])/(np.linalg.norm(AB)*np.linalg.norm(D1)))*(math.sin(theta)/AB_len)
            if cos_A1>=0.99 and cos_B2>=0.99:
                if robot['face_angle'] >= 0:
                    angle_speed_new = 2+angle_speed-all_robot[i]['angle_speed']
                else:
                    angle_speed_new = -2+angle_speed+all_robot[i]['angle_speed']
                if (robot['carried_product_type'] or all_robot[i]['carried_product_type']) and line_speed>=0:
                    line_speed_new = all_robot[i]['line_speed_x']/math.cos(all_robot[i]['face_angle'])-2
                else:
                    line_speed_new = all_robot[i]['line_speed_x'] / math.cos(all_robot[i]['face_angle']) + 2
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
            #if cos_A1+cos_B2+cos_theta>=2 and robot['line_speed']:
            if cos_A1 + cos_B2 + cos_theta >= 2 \
                     and D1_m/(robot['line_speed_x']/math.cos(robot['face_angle']))-D2_m/(all_robot[i]['line_speed_x']/math.cos(all_robot[i]['face_angle']))<=0.5:
                line_speed_new = line_speed
                if robot['carried_product_type'] and (float(all_robot[i]['x'])-float(robot['x']))**2+(float(all_robot[i]['y'])-float(robot['y']))**2<=1**2:
                    line_speed_new = line_speed-2
                if robot['face_angle'] >= 0:
                    angle_speed_new = 2 + angle_speed
                else:
                    angle_speed_new = -2 + angle_speed
                return line_speed_new, angle_speed_new
    return line_speed_new, angle_speed_new

#检测碰撞函数，返回一个碰撞列表，表中元素表示会发生碰撞的机器人，如[[0, 1], [2, 3]]
def crash_detect(robot_list, crash_detect_distance = 2):
    #碰撞检测阈值，距离小于该值开始检测

    crash_list = [] #碰撞列表

    for i in range(len(robot_list)): #遍历整个机器人列表,robot为待检测
        for j in range(i+1 ,len(robot_list)):
            robot_1 = robot_list[i]
            robot_2 = robot_list[j]
            #机器人的坐标
            robot_1_xy = np.array([robot_1['x'], robot_1['y']])
            robot_2_xy = np.array([robot_2['x'], robot_2['y']])
            robot_12_vec = robot_2_xy - robot_1_xy #从1到2的向量

            if np.linalg.norm(robot_12_vec) > crash_detect_distance: continue  # 距离过远不检测

            # 机器人的速度v向量
            robot_1_v = np.array([robot_1['line_speed_x'], robot_1['line_speed_y']])
            robot_2_v = np.array([robot_2['line_speed_x'], robot_2['line_speed_y']])

            relative_speed =robot_1_v - robot_2_v #相对速度

            relative_speed_angle = math.atan2(relative_speed[1], relative_speed[0]) #相对速度方向
            robot_12_angle = math.atan2(robot_12_vec[1], robot_12_vec[0]) #两机器人连线方向12

            crash_threshold = 2 * np.arcsin(0.45 / crash_detect_distance) #计算临界碰撞角度

            if abs(relative_speed_angle - robot_12_angle) < crash_threshold: #碰撞情况
                crash_list.append([robot_1['id'], robot_2['id']])

    return crash_list

#检测到碰撞后的转向
def avoid_crash_v2( crash_list):
    rotate_list = [0, 0, 0, 0] #一个id的机器人只能转一次

    for crash in crash_list:
        for crash_id in crash:
            if rotate_list[crash_id]: continue
            else:
                sys.stdout.write('forward %d %d\n' % (crash_id, 2))
                sys.stdout.write('rotate %d %f\n' % (crash_id, 2))

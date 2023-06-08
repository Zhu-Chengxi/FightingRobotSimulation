import pybullet as p
import time
import math
import cv2
import threading
import pybullet_data
import socket
import numpy as np
import csv
import random
from collections import namedtuple
from attrdict import AttrDict

# degree/s   a=31.8 degree/4ms/4ms = 0.555 rad/4ms/4ms  max_speed = 20rad/s

max_speed = 20
next_pos = [0, 0, 0, 0, 0, 0]
next_v = [0, 0, 0, 0, 0, 0]

hitpoint = [53, 40.1, 103]
hit_time = -1
hit_point_vx = -1

desire_position = [0, 400, 800]
desire_posture = [0., 0.5, 0.5, 0.5]

# MotoControl Algorithm
def Jointcontrol(curpos, curv, tarpos, tarv, acc, i):  # acc>0
    if math.fabs(tarpos - curpos) > math.pi / 180.0:
        if (tarpos - curpos) * curv < 0:
            if curv > 0:
                next_v[i] = curv - acc
            if curv < 0:
                next_v[i] = curv + acc
        else:
            s = curv * curv / 2.0 / 137.5
            if math.fabs(tarpos - curpos) > s:
                if tarpos > curpos and math.fabs(curv) < tarv:
                    next_v[i] = curv + acc
                if tarpos < curpos and math.fabs(curv) < tarv:
                    next_v[i] = curv - acc
            else:
                if curv > 0:
                    next_v[i] = curv - acc
                if curv < 0:
                    next_v[i] = curv + acc
        dis = curv * 4 / 1000.0
        next_pos[i] = curpos + dis
    else:
        next_pos[i] = tarpos
        next_v[i] = 0.0
    return next_pos[i], next_v[i]


def armthread(args):
    zero = [0 * math.pi / 180.0, 0 * math.pi / 180.0, 0 * math.pi / 180.0,
            90 * math.pi / 180.0, 90 * math.pi / 180.0, 0 * math.pi / 180.0, 0 * math.pi / 180.0]
    joint_target = [0, 0 * math.pi / 180.0, 0 * math.pi / 180.0, 0 * math.pi / 180.0, 0 * math.pi / 180.0,
                    0 * math.pi / 180.0, 0 * math.pi / 180.0]
    # 1th: [-50, 50]  2th: [0, -65]  3th: [0, -65]   4th: [-30, 30]   5th: [-180, 180]  6th: [-50, 50]
    # 5th    0-(-90)  80-(-170)   100-(170)   -100-(10)    -80-(-10)
    count = 0
    while True:
        count = count + 1
        t = time.time()
        position1, velocity1, force1, torque1 = p.getJointState(robotid, 0)
        position2, velocity2, force2, torque2 = p.getJointState(robotid, 1)
        position3, velocity3, force3, torque3 = p.getJointState(robotid, 2)
        position4, velocity4, force4, torque4 = p.getJointState(robotid, 3)
        position5, velocity5, force5, torque5 = p.getJointState(robotid, 4)
        position6, velocity6, force6, torque6 = p.getJointState(robotid, 5)
        position7, velocity7, force7, torque7 = p.getJointState(robotid, 6)

        if math.fabs((position1 - zero[0]) * 180 / math.pi - joint_target[0] * 180 / math.pi) < 1 and math.fabs(
                (position2 - zero[1]) * 180 / math.pi - joint_target[1] * 180 / math.pi) < 1 and math.fabs(
            (position3 - zero[2]) * 180 / math.pi - joint_target[2] * 180 / math.pi) < 2 and math.fabs(
            (position4 - zero[3]) * 180 / math.pi - joint_target[3] * 180 / math.pi) < 1 and math.fabs(
            (position5 - zero[4]) * 180 / math.pi - joint_target[4] * 180 / math.pi) < 1 and math.fabs(
            (position6 - zero[5]) * 180 / math.pi - joint_target[5] * 180 / math.pi) < 1 and math.fabs(
            (position7 - zero[6]) * 180 / math.pi - joint_target[6] * 180 / math.pi) < 1:
            joint_target[0] = round(math.pi / 180.0 * random.uniform(-50, 50), 2)
            thate2 = random.uniform(-50, 0)
            joint_target[1] = round(math.pi / 180.0 * thate2, 2)
            joint_target[2] = round(math.pi / 180.0 * random.uniform(0, thate2), 2)
            joint_target[3] = joint_target[1] - joint_target[2]  # round(math.pi/180.0*random.uniform(-20,8), 2)
            joint_target[4] = round(math.pi / 180.0 * random.uniform(-180, 180), 2)
            joint_target[5] = round(math.pi / 180.0 * random.uniform(-30, 30), 2)
            joint_target[6] = round(math.pi / 180.0 * random.uniform(-90, 90), 2)
        #		for j in range(0,6):
        #			pos, speed = Jointcontrol(next_pos[j], next_v[j], joint_target[j], 20, 0.555, j)
        #			p.setJointMotorControl2(robotid,j,p.POSITION_CONTROL, zero[j]+pos, speed)
        p.setJointMotorControl2(robotid, 0, p.POSITION_CONTROL, zero[0] + joint_target[0], 0)
        p.setJointMotorControl2(robotid, 1, p.POSITION_CONTROL, zero[1] + joint_target[1], 0)
        p.setJointMotorControl2(robotid, 2, p.POSITION_CONTROL, zero[2] + joint_target[2], 0)
        p.setJointMotorControl2(robotid, 3, p.POSITION_CONTROL, zero[3] + joint_target[3], 0)
        p.setJointMotorControl2(robotid, 4, p.POSITION_CONTROL, zero[4] + joint_target[4], 0)
        p.setJointMotorControl2(robotid, 5, p.POSITION_CONTROL, zero[5] + joint_target[5], 0)
        p.setJointMotorControl2(robotid, 6, p.POSITION_CONTROL, zero[6] + joint_target[6], 0)

        time.sleep(1. / 40.)
        fiveth_pos = p.getLinkState(robotid, 6)[0]
        fiveth_Orn = p.getLinkState(robotid, 6)[1]
        fiveth_Orn_temp = np.array(fiveth_Orn)
        fiveth_gripperOrn = p.getEulerFromQuaternion(fiveth_Orn_temp)  # roll pitch yaw
        fiveth_gripperOrn = np.array(fiveth_gripperOrn)

        fourth_pos = p.getLinkState(robotid, 5)[0]
        fourth_Orn = p.getLinkState(robotid, 5)[1]
        fourth_Orn_temp = np.array(fourth_Orn)
        fourth_gripperOrn = p.getEulerFromQuaternion(fourth_Orn_temp)  # roll pitch yaw
        fourth_gripperOrn = np.array(fourth_gripperOrn)

        position1, velocity1, force1, torque1 = p.getJointState(robotid, 0)
        position2, velocity2, force2, torque2 = p.getJointState(robotid, 1)
        position3, velocity3, force3, torque3 = p.getJointState(robotid, 2)
        position4, velocity4, force4, torque4 = p.getJointState(robotid, 3)
        position5, velocity5, force5, torque5 = p.getJointState(robotid, 4)
        position6, velocity6, force6, torque6 = p.getJointState(robotid, 5)
        position7, velocity7, force7, torque7 = p.getJointState(robotid, 6)
        print('hit state: ', (position1 - zero[0]) * 180 / math.pi, (position2 - zero[1]) * 180 / math.pi,
              (position3 - zero[2]) * 180 / math.pi,
              (position4 - zero[3]) * 180 / math.pi, (position5 - zero[4]) * 180 / math.pi,
              (position6 - zero[5]) * 180 / math.pi, (position7 - zero[6]) * 180 / math.pi)
        print('hit stata: ', (joint_target[0]) * 180 / math.pi, (joint_target[1]) * 180 / math.pi,
              (joint_target[2]) * 180 / math.pi,
              (joint_target[3]) * 180 / math.pi, (joint_target[4]) * 180 / math.pi, (joint_target[5]) * 180 / math.pi,
              (joint_target[6]) * 180 / math.pi)
        print(t);
        #		print('hit stataaaa: ', round(fiveth_pos[0]*100, 2), round(fiveth_pos[1]*100, 2), round(fiveth_pos[2]*100, 2), round(gripperOrn[0]*180.0/math.pi, 2), round(gripperOrn[1]*180.0/math.pi, 2), round(gripperOrn[2]*180.0/math.pi, 2))
        #		if count == 4:
        #			count = 0
        #		write.writerow({'pos_x': round(fiveth_pos[0]*100, 2), 'pos_y': round(fiveth_pos[1]*100, 2), 'pos_z': round(fiveth_pos[2]*100, 2), 'roll': round(fiveth_gripperOrn[0]*180.0/math.pi, 2), 'pitch': round(fiveth_gripperOrn[1]*180.0/math.pi, 2), 'yaw': round(fiveth_gripperOrn[2]*180.0/math.pi, 2), 'th1': round((position1-zero[0])*180/math.pi, 2), 'th2': round((position2-zero[1])*180/math.pi, 2), 'th3': round((position3-zero[2])*180/math.pi, 2), 'th4': round((position4-zero[3])*180/math.pi, 2), 'th5': round((position5-zero[4])*180/math.pi, 2), 'th6': round((position6-zero[5])*180/math.pi, 2)})
        write.writerow({'6pos_x': round(fiveth_pos[0] * 100, 2), '6pos_y': round(fiveth_pos[1] * 100, 2),
                        '6pos_z': round(fiveth_pos[2] * 100, 2),
                        '6roll': round(fiveth_gripperOrn[0] * 180.0 / math.pi, 2),
                        '6pitch': round(fiveth_gripperOrn[1] * 180.0 / math.pi, 2),
                        '6yaw': round(fiveth_gripperOrn[2] * 180.0 / math.pi, 2),
                        '5pos_x': round(fourth_pos[0] * 100, 2), '5pos_y': round(fourth_pos[1] * 100, 2),
                        '5pos_z': round(fourth_pos[2] * 100, 2),
                        '5roll': round(fourth_gripperOrn[0] * 180.0 / math.pi, 2),
                        '5pitch': round(fourth_gripperOrn[1] * 180.0 / math.pi, 2),
                        '5yaw': round(fourth_gripperOrn[2] * 180.0 / math.pi, 2),
                        'th6': round((position6 - zero[5]) * 180 / math.pi, 2)})

        time.sleep(1. / 40.)


def inverse_kinematics(input_matrix, output_list):
    if input_matrix[1][2] >= 0:
        output_list[5] = math.acos(input_matrix[1][2])  # theta6
    else:
        output_list[5] = math.acos(input_matrix[1][2]) - math.pi
    print("theta6 = ", output_list[5])
    output_list[6] = math.atan2(-input_matrix[1][1], input_matrix[1][0])  # theta7
    print("theta7 = ", output_list[6])
    output_list[4] = math.atan2(input_matrix[2][2], input_matrix[0][2])  # theta5
    print("theta5 = ", output_list[4])
    temp = round(
        float(0.55 * math.sin(output_list[4]) * math.sin(output_list[5]) + 0.2 * math.cos(output_list[4]) + 0.6 -
              input_matrix[2][3]) / 0.3, 5)
    output_list[1] = math.asin(temp)
    if output_list[1] >= 0:
        output_list[1] -= math.pi
    print("theta2 = ", output_list[1])
    if math.cos(output_list[1]) >= 1:
        temp = 0
    else:
        temp = round(input_matrix[1][3] - 0.55 * math.cos(output_list[5]) - 0.1) / (
                0.3 * (1 - math.cos(output_list[1])))
    output_list[0] = math.asin(temp)  # theta1
    print("theta1 = ", output_list[0])
    output_list[2] = -output_list[1] - math.pi
    output_list[3] = -output_list[0] - math.pi


if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(numSolverIterations=10)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0, cameraPitch=-40,
                                 cameraTargetPosition=[0.5, -0.9, 0.5])  # 转变视角

    # p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(1)

    scale = [1, 1, 1]

    plane_id = p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
    # 添加.urdf文件路径
    robotid = p.loadURDF("C:/Users/Zhu/Desktop/FightingRobotSimulation/MultifunctionalFightingRobot/urdf/MultifunctionalFightingRobot.urdf", useFixedBase=True,
                         basePosition=[0, 0, 0])

    p.resetJointState(robotid, 0, 0)
    p.resetJointState(robotid, 1, 0)
    p.resetJointState(robotid, 2, 0)
    p.resetJointState(robotid, 3, 0)
    p.resetJointState(robotid, 4, 0)

    # 添加.csv输出文件路径
    csvfile = open('C:/Users/Zhu/Desktop/FightingRobotSimulation/data.csv', mode='w', newline='')

    # 标题列表
    fieldnames = ['6pos_x', '6pos_y', '6pos_z', '6roll', '6pitch', '6yaw', '5pos_x', '5pos_y', '5pos_z', '5roll',
                  '5pitch', '5yaw', 'th6']
    # 创建 DictWriter 对象
    write = csv.DictWriter(csvfile, fieldnames=fieldnames)
    # 写入表头
    write.writeheader()
    # 写入数据

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # 添加渲染
    #	fd = 0.5*0.45*0.001256*1.21*5*5
    p.changeDynamics(robotid, 5, restitution=1.)

    # 记录各个节点类型的列表
    jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
    # 得到机器人的节点个数
    numJoints = p.getNumJoints(robotid)
    # 属于提前创造存储节点信息的数据结构
    jointInfo = namedtuple("jointInfo",
                           ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity"])
    # 另一个数据结构
    joints = AttrDict()

    # thread1 = threading.Thread(target=armthread, args=(1,))
    # thread1.start()

    print(joints)

    for i in range(numJoints):
        # 得到节点的信息
        info = p.getJointInfo(robotid, i)
        # 将节点的各个信息提取出来
        jointID = info[0]
        jointName = info[1].decode('utf-8')
        jointType = jointTypeList[info[2]]
        jointLowerLimit = info[8]
        jointUpperLimit = info[9]
        jointMaxForce = info[10]
        jointMaxVelocity = info[11]
        singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce,
                               jointMaxVelocity)
        joints[singleInfo.name] = singleInfo
    print(joints)

    # 创建空列表
    position_control_group = []
    # 将创建的参数存到空列表中
    original_joint0 = 0
    original_joint1 = 0
    original_joint2 = 0
    original_joint3 = 0
    original_joint4 = 0


    # 参数：["节点名字","可滑动最小的角度","可滑动最大的角度","节点初始角度"]
    position_control_group.append(
        p.addUserDebugParameter('joint0', -90 * math.pi / 180, 90 * math.pi / 180, original_joint0))
    position_control_group.append(
        p.addUserDebugParameter('joint1', 0 * math.pi / 180, 90 * math.pi / 180, original_joint1))
    position_control_group.append(
        p.addUserDebugParameter('joint2', -90 * math.pi / 180, 0 * math.pi / 180, original_joint2))
    position_control_group.append(
        p.addUserDebugParameter('joint3', -180 * math.pi / 180, 180 * math.pi / 180, original_joint3))
    position_control_group.append(
        p.addUserDebugParameter('joint4', -90 * math.pi / 180, 90 * math.pi / 180, original_joint4))

    position_control_joint_name = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5"]
    print("position_control_group:", position_control_group)

    # kinematics_para = np.array([[1., 0., 0., 0.3], [0., 0., 1., 0.7], [0., -1., 0., 0.8]])
    joints_para = [0., 0., 0., 0., 0.]
    # inverse_kinematics(kinematics_para, joints_para)

    # print(joints_para)
    #
    joints_para[0] = joints_para[0]
    joints_para[1] = joints_para[1]
    joints_para[2] = joints_para[2]
    joints_para[3] = joints_para[3] + math.pi / 2
    joints_para[4] = joints_para[4]

    p.setRealTimeSimulation(1)
    while True:
        time.sleep(0.01)
        parameter = []
        # 将添加的参数“读”出来
        for i in range(5):
            parameter.append(p.readUserDebugParameter(position_control_group[i]))
            # parameter.append(joints_para[i])
        # print(parameter)
        num = 0
        # print(joints_para)
        for jointName in joints:
            if jointName in position_control_joint_name:
                joint = joints[jointName]
                parameter_sim = parameter[num]
                p.setJointMotorControl2(robotid, joint.id, p.POSITION_CONTROL, targetPosition=parameter_sim,
                                        force=joint.maxForce, maxVelocity=joint.maxVelocity)
                num = num + 1
        # for i in range(5):
        #     p.setJointMotorControl2(robotid, i, p.POSITION_CONTROL, targetPosition=joints_para[i], force=750,
        #                             maxVelocity=1)

        # first_pos = p.getLinkState(robotid, 0)[0]
        # second_pos = p.getLinkState(robotid, 1)[0]
        # third_pos = p.getLinkState(robotid, 2)[0]
        # base_pos = p.getBasePositionAndOrientation(robotid)[0]
        # base_orn = p.getBasePositionAndOrientation(robotid)[1]
        # end_pos = p.getLinkState(robotid, 6)[0]
        # end_orn = p.getLinkState(robotid, 6)[1]
        # base_pos = np.array(base_pos) - [0, 0, 0.075]
        # print(np.array(second_pos) - np.array(first_pos))
        # print(np.array(first_pos) - np.array(base_pos), np.array(second_pos) - np.array(base_pos), np.array(third_pos) - np.array(base_pos))
        # print(np.array(end_pos) - np.array(base_pos), end_orn)
        # print(base_pos, base_orn)
        p.stepSimulation()

    # posture = np.array(p.getLinkState(robotid, 6)[5], dtype=float).reshape(4, 1)
    # position = np.array(p.getLinkState(robotid, 6)[4], dtype=float).reshape(3, 1)
    # n = 100
    # while (n):
    #     p.stepSimulation()
    #     n = n - 1
    # position = np.array(p.getLinkState(robotid, 6)[4], dtype=float).reshape(3, 1)
    # print(position[0])
    # time.sleep(0.5)
    # p.stepSimulation()

    p.disconnect()

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
import keyboard

gen_homogeneousMatrix = np.zeros((4, 4))
end_homogeneousMatrix = np.zeros((4, 4))
joint_list = np.zeros(5)

def generateHomogeneousMatrix(roll, pitch, yaw, pos_x, pos_y, pos_z):  # 根据位姿生成齐次式 roll-z pitch-y yaw-x
    gen_homogeneousMatrix[0][0] = math.cos(roll) * math.cos(pitch)
    gen_homogeneousMatrix[0][1] = math.cos(roll) * math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw)
    gen_homogeneousMatrix[0][2] = math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)
    gen_homogeneousMatrix[0][3] = pos_x

    gen_homogeneousMatrix[1][0] = math.sin(roll) * math.cos(pitch)
    gen_homogeneousMatrix[1][1] = math.sin(roll) * math.sin(pitch) * math.sin(yaw) + math.cos(roll) * math.cos(yaw)
    gen_homogeneousMatrix[1][2] = math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)
    gen_homogeneousMatrix[1][3] = pos_y

    gen_homogeneousMatrix[2][0] = -math.sin(pitch)
    gen_homogeneousMatrix[2][1] = math.cos(pitch) * math.sin(yaw)
    gen_homogeneousMatrix[2][2] = math.cos(pitch) * math.cos(yaw)
    gen_homogeneousMatrix[2][3] = pos_z

    gen_homogeneousMatrix[3][0] = 0.
    gen_homogeneousMatrix[3][1] = 0.
    gen_homogeneousMatrix[3][2] = 0.
    gen_homogeneousMatrix[3][3] = 1.


def forward_kinematics(joint_para):  # 正运动学 根据关节角度求出末端位姿
    theta1 = joint_para[0]
    theta2 = joint_para[1]
    theta3 = joint_para[2]
    theta4 = joint_para[3]
    theta5 = joint_para[4]

    end_homogeneousMatrix[0][0] = math.cos(theta1) * math.cos(theta2 + theta3) * math.cos(theta4) * math.cos(
        theta5) + math.sin(theta1) * math.sin(theta4) * math.cos(theta5) - math.cos(theta1) * math.sin(
        theta2 + theta3) * math.sin(theta5)
    end_homogeneousMatrix[0][1] = -math.cos(theta1) * math.cos(theta2 + theta3) * math.cos(theta4) * math.sin(
        theta5) - math.sin(theta1) * math.sin(theta4) * math.sin(theta5) - math.cos(theta1) * math.sin(
        theta2 + theta3) * math.cos(theta5)
    end_homogeneousMatrix[0][2] = -math.cos(theta1) * math.cos(theta2 + theta3) * math.sin(theta4) + math.sin(
        theta1) * math.cos(theta4)
    end_homogeneousMatrix[0][3] = 0.68 * math.cos(theta1) * math.cos(theta2 + theta3) * math.sin(
        theta4) - 0.68 * math.sin(theta1) * math.cos(theta4) - 0.2 * math.cos(theta1) * math.sin(
        theta2 + theta3) + 0.6 * math.cos(theta1) * math.cos(theta2)

    end_homogeneousMatrix[1][0] = math.sin(theta1) * math.cos(theta2 + theta3) * math.cos(theta4) * math.cos(
        theta5) - math.cos(theta1) * math.sin(theta4) * math.cos(theta5) - math.sin(theta1) * math.sin(
        theta2 + theta3) * math.sin(theta5)
    end_homogeneousMatrix[1][1] = -math.sin(theta1) * math.cos(theta2 + theta3) * math.cos(theta4) * math.sin(
        theta5) + math.cos(theta1) * math.sin(theta4) * math.sin(theta5) - math.sin(theta1) * math.sin(
        theta2 + theta3) * math.cos(theta5)
    end_homogeneousMatrix[1][2] = -math.sin(theta1) * math.cos(theta2 + theta3) * math.sin(theta4) - math.cos(
        theta1) * math.cos(theta4)
    end_homogeneousMatrix[1][3] = 0.68 * math.sin(theta1) * math.cos(theta2 + theta3) * math.sin(
        theta4) + 0.68 * math.cos(theta1) * math.cos(theta4) - 0.2 * math.sin(theta1) * math.sin(
        theta2 + theta3) + 0.6 * math.sin(theta1) * math.cos(theta2)

    end_homogeneousMatrix[2][0] = math.sin(theta2 + theta3) * math.cos(theta4) * math.cos(theta5) + math.cos(
        theta2 + theta3) * math.sin(theta5)
    end_homogeneousMatrix[2][1] = -math.sin(theta2 + theta3) * math.cos(theta4) * math.sin(theta5) + math.cos(
        theta2 + theta3) * math.cos(theta5)
    end_homogeneousMatrix[2][2] = -math.sin(theta2 + theta3) * math.sin(theta4)
    end_homogeneousMatrix[2][3] = 0.8 * math.sin(theta2 + theta3) * math.sin(theta4) + 0.2 * math.cos(
        theta2 + theta3) + 0.6 * math.sin(theta2) + 0.6

    end_homogeneousMatrix[3][0] = 0.
    end_homogeneousMatrix[3][1] = 0.
    end_homogeneousMatrix[3][2] = 0.
    end_homogeneousMatrix[3][3] = 1.

def inverse_kinematics(end_matrix):  # 逆运动学 根据4*4的末端齐次坐标求出关节角
    # theta1
    joint_list[0] = math.atan2(0.68 * end_matrix[1][2] + end_matrix[1][3], 0.68 * end_matrix[0][2] + end_matrix[0][3])
    if joint_list[0] > math.pi / 2:
        joint_list[0] -= math.pi
    if joint_list[0] < -math.pi / 2:
        joint_list[0] += math.pi
    theta1 = joint_list[0]
    # theta4
    joint_list[3] = math.acos((-math.sin(theta1) * end_matrix[0][3] + math.cos(theta1) * end_matrix[1][3]) / 0.68)
    if joint_list[3] >= 0:
        joint_list[3] = -joint_list[3]
    if joint_list[3] <= -math.pi:
        joint_list[3] += math.pi
    theta4 = joint_list[3]
    # theta5
    joint_list[4] = math.atan2(-math.sin(theta1) * end_matrix[0][1] + math.cos(theta1) * end_matrix[1][1],
                               math.sin(theta1) * end_matrix[0][0] - math.cos(theta1) * end_matrix[1][0])
    if joint_list[4] > math.pi / 2:
        joint_list[4] -= math.pi
    if joint_list[4] < -math.pi / 2:
        joint_list[4] += math.pi
    theta5 = joint_list[4]
    # theta2 + theta3
    theta23 = math.atan2(end_matrix[2][2], math.cos(theta1) * end_matrix[0][2] + math.sin(theta1) * end_matrix[1][2])
    if theta23 > math.pi / 2:
        theta23 -= math.pi
    if theta23 < -math.pi / 2:
        theta23 += math.pi
    # theta2
    # print(joint_list)
    joint_list[1] = math.acos((math.cos(theta1) * end_matrix[0][3] + math.sin(theta1) * end_matrix[1][
        3] - 0.68 * math.cos(theta23) * math.sin(theta4) + 0.2 * math.sin(theta23)) / 0.6)
    theta2 = joint_list[1]
    # theta 3
    joint_list[2] = theta23 - theta2
    theta3 = joint_list[2]

    print("calculateJointPos", joint_list)


def generateRandomJointPos(jointPos):
    jointPos[0] = random.uniform(-math.pi / 2, math.pi / 2)
    jointPos[1] = random.uniform(0, math.pi / 2)
    jointPos[2] = random.uniform(-math.pi / 2, 0)
    jointPos[3] = random.uniform(-math.pi, 0)
    jointPos[4] = random.uniform(-math.pi / 2, math.pi / 2)


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
    robotid = p.loadURDF(
        "C:/Users/Zhu/Desktop/FightingRobotSimulation/MultifunctionalFightingRobot/urdf/MultifunctionalFightingRobot.urdf",
        useFixedBase=True,
        basePosition=[0, 0, 0])

    p.resetJointState(robotid, 0, 0)
    p.resetJointState(robotid, 1, 0)
    p.resetJointState(robotid, 2, 0)
    p.resetJointState(robotid, 3, 0)
    p.resetJointState(robotid, 4, 0)

    # 添加.csv输出文件路径
    csvfile = open('C:/Users/Zhu/Desktop/FightingRobotSimulation/data.csv', mode='w', newline='')

    # 标题列表
    fieldnames = ['yaw', 'pitch', 'row', 'pos_x', 'pos_y', 'pos_z']
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

    # 此处修改齐次坐标，然后进行逆运动学求解
    jointRandom = np.zeros(5)
    generateRandomJointPos(jointRandom)
    print("targetJointPos", jointRandom)
    joint_temp = [jointRandom[0], jointRandom[1], jointRandom[2], jointRandom[3], jointRandom[4]]
    # joint_temp = [-math.pi / 3, math.pi / 3, 0, -math.pi / 2, math.pi / 6]
    # joint_temp = [0, 0, 0, -math.pi / 2, 0]
    forward_kinematics(joint_temp)
    # kinematics_para = np.array([[5.00000000e-01, 8.66025404e-01, -2.24125920e-17, - 1.00000000e-01],
    #                             [-8.66025404e-01, 5.00000000e-01, -8.36449319e-17, 1.73205081e-01],
    #                             [-6.12323400e-17, 6.12323400e-17, 1.00000000e+00, 4.00000000e-01],
    #                             [0., 0., 0., 1.]])
    print(end_homogeneousMatrix[0][3], end_homogeneousMatrix[1][3], end_homogeneousMatrix[2][3])
    inverse_kinematics(end_homogeneousMatrix)

    joint_list[3] = joint_list[3] + math.pi / 2

    p.setRealTimeSimulation(1)
    while True:
        time.sleep(0.01)
        for i in range(5):
            p.setJointMotorControl2(robotid, i, p.POSITION_CONTROL, targetPosition=joint_list[i], force=750,
                                    maxVelocity=1)
        end_pos = p.getLinkState(robotid, 4)[0]
        end_orn = p.getLinkState(robotid, 4)[1]
        print(end_pos, end_orn)
        # first_pos = p.getLinkState(robotid, 0)[0]
        # second_pos = p.getLinkState(robotid, 1)[0]
        # third_pos = p.getLinkState(robotid, 2)[0]
        # base_pos = p.getBasePositionAndOrientation(robotid)[0]
        # base_orn = p.getBasePositionAndOrientation(robotid)[1]

        # base_pos = np.array(base_pos) - [0, 0, 0.075]
        # print(np.array(second_pos) - np.array(first_pos))
        # print(np.array(first_pos) - np.array(base_pos), np.array(second_pos) - np.array(base_pos), np.array(third_pos) - np.array(base_pos))
        # print(np.array(end_pos) - np.array(base_pos), end_orn)
        # print(base_pos, base_orn)
        p.stepSimulation()
        if keyboard.is_pressed('esc'):
            break

    p.disconnect()

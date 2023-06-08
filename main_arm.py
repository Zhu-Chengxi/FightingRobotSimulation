import pybullet as p
import time
import math
import cv2
import threading
import pybullet_data
import numpy as np
import csv
import random
from collections import namedtuple
from attrdict import AttrDict
import keyboard

if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(numSolverIterations=10)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
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
    fieldnames = ['pos_x', 'pos_y', 'pos_z', 'roll', 'pitch', 'yaw']
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

    p.setRealTimeSimulation(1)
    while True:
        time.sleep(0.01)
        parameter = []
        # 将添加的参数“读”出来
        for i in range(5):
            if i == 3:
                parameter.append(p.readUserDebugParameter(position_control_group[i]) + math.pi/2)
            else:
                parameter.append(p.readUserDebugParameter(position_control_group[i]))
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
        p.stepSimulation()
        if keyboard.is_pressed('esc'):
            break


    p.disconnect()

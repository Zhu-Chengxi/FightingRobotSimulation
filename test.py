import numpy as np
import math
import random

HomogeneousMatrix = np.zeros((4, 4))
end_homogeneousMatrix = np.zeros((4, 4))
joint_list = np.zeros(5)


def generateHomogeneousMatrix(roll, pitch, yaw, pos_x, pos_y, pos_z):  # 根据位姿生成齐次式 roll-z pitch-y yaw-x 弧度
    HomogeneousMatrix[0][0] = math.cos(pitch) * math.cos(roll)
    HomogeneousMatrix[0][1] = -math.cos(pitch) * math.sin(roll)
    HomogeneousMatrix[0][2] = math.sin(pitch)
    HomogeneousMatrix[0][3] = pos_x

    HomogeneousMatrix[1][0] = math.sin(yaw) * math.sin(pitch) * math.cos(roll) + math.cos(yaw) * math.sin(roll)
    HomogeneousMatrix[1][1] = -math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll)
    HomogeneousMatrix[1][2] = -math.sin(yaw) * math.cos(pitch)
    HomogeneousMatrix[1][3] = pos_y

    HomogeneousMatrix[2][0] = -math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)
    HomogeneousMatrix[2][1] = math.cos(yaw) * math.sin(pitch) * math.sin(roll) + math.sin(yaw) * math.cos(roll)
    HomogeneousMatrix[2][2] = math.cos(pitch) * math.cos(yaw)
    HomogeneousMatrix[2][3] = pos_z

    HomogeneousMatrix[3][0] = 0.
    HomogeneousMatrix[3][1] = 0.
    HomogeneousMatrix[3][2] = 0.
    HomogeneousMatrix[3][3] = 1.


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
    print(joint_list[3])
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
    theta23 = math.atan2(round(end_matrix[2][2], 6),round(math.cos(theta1) * end_matrix[0][2] + math.sin(theta1) * end_matrix[1][2], 6))
    if theta23 > math.pi / 2:
        theta23 -= math.pi
    if theta23 < -math.pi / 2:
        theta23 += math.pi
    # theta2
    print(joint_list)
    print(theta23)
    joint_list[1] = math.acos((math.cos(theta1) * end_matrix[0][3] + math.sin(theta1) * end_matrix[1][
        3] - 0.68 * math.cos(theta23) * math.sin(theta4) + 0.2 * math.sin(theta23)) / 0.6)
    theta2 = joint_list[1]
    # theta 3
    joint_list[2] = theta23 - theta2
    theta3 = joint_list[2]

    print("calculateJointPos", joint_list)


if __name__ == '__main__':
    generateHomogeneousMatrix(math.pi/4, 0, math.pi/2, 0.6, 0.68, 0.8)
    print(HomogeneousMatrix)
    joint = [ 0, 0, 0, 0, math.pi/4]
    print(joint)
    forward_kinematics(joint)
    print(end_homogeneousMatrix)
    inverse_kinematics(HomogeneousMatrix)
    # for i in range(5):
    #     joint_list[i] = joint_list[i] * 180 / math.pi
    # print(joint_list)

import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# 角度转弧度
def d2r(degree):
    return degree * np.pi / 180.0

# 弧度转角度
def r2d(radian):
    return radian * 180.0 / np.pi

# 正运动学, 输入关节角度, 输出末端位姿
def Forward_Kinematics(d, a, alpha, theta):
    theta = d2r(theta)
    T = []
    for i in range(6):
        T_i = np.eye(4)
        T_i[0, 0] = np.cos(theta[i])
        T_i[0, 1] = -np.sin(theta[i]) * np.cos(alpha[i])
        T_i[0, 2] = np.sin(theta[i]) * np.sin(alpha[i])
        T_i[0, 3] = a[i] * np.cos(theta[i])
        T_i[1, 0] = np.sin(theta[i])
        T_i[1, 1] = np.cos(theta[i]) * np.cos(alpha[i])
        T_i[1, 2] = -np.cos(theta[i]) * np.sin(alpha[i])
        T_i[1, 3] = a[i] * np.sin(theta[i])
        T_i[2, 0] = 0
        T_i[2, 1] = np.sin(alpha[i])
        T_i[2, 2] = np.cos(alpha[i])
        T_i[2, 3] = d[i]
        T.append(T_i)

    T06 = np.eye(4)
    for t in T:
        T06 = np.dot(T06, t)

    return T06

# 逆运动学, 输入末端位姿, 输出关节角度
def Inverse_Kinematics(d, a, T06):

    theta = np.zeros((8, 6))

    # theta1 两个解
    A = d[5] * T06[1, 2] - T06[1, 3]
    B = d[5] * T06[0, 2] - T06[0, 3]
    C = d[3]
    # 第一个解，赋给一到四组
    theta[0, 0] = math.atan2(A, B) - math.atan2(C, math.sqrt(A * A + B * B - C * C))
    theta[1, 0] = theta[0, 0]
    theta[2, 0] = theta[0, 0]
    theta[3, 0] = theta[0, 0]
    # 第二个解，赋给五到八组
    theta[4, 0] = math.atan2(A, B) - math.atan2(C, -math.sqrt(A * A + B * B - C * C))
    theta[5, 0] = theta[4, 0]
    theta[6, 0] = theta[4, 0]
    theta[7, 0] = theta[4, 0]

    # theta5 四个解
    # 由theta[1, 1]产生的第一个解，赋给一到二组
    A = np.sin(theta[0, 0]) * T06[0, 2] - np.cos(theta[0, 0]) * T06[1, 2]
    theta[0, 4] = math.acos(A)
    theta[1, 4] = theta[0, 4]
    # 由theta[1, 1]产生的第二个解，赋给三到四组
    theta[2, 4] = -math.acos(A)
    theta[3, 4] = theta[2, 4]
    # 由theta[5, 1]产生的第一个解，赋给五到六组
    A = np.sin(theta[4, 0]) * T06[0, 2] - np.cos(theta[4, 0]) * T06[1, 2]
    theta[4, 4] = math.acos(A)
    theta[5, 4] = theta[4, 4]
    # 由theta[5, 1]产生的第二个解，赋给七到八组
    theta[6, 4] = -math.acos(A)
    theta[7, 4] = theta[6, 4]

    # theta6 四个解
    for i in range(0, 8, 2):
        A = (np.sin(theta[i, 0]) * T06[0, 0] - np.cos(theta[i, 0]) * T06[1, 0])
        B = (np.sin(theta[i, 0]) * T06[0, 1] - np.cos(theta[i, 0]) * T06[1, 1])
        C = np.sin(theta[i, 4])
        D = A * A + B * B - C * C

        if C <= -0.00001 or C >= 0.000001:
            theta[i, 5] = math.atan2(A, B) - math.atan2(C, 0.00)
            theta[i + 1, 5] = math.atan2(A, B) - math.atan2(C, 0.00)
        else:
            theta[i, 5] = 0
            theta[i + 1, 5] = 0

    # theta3 8组解
    for i in range(0, 8, 2):
        C = np.cos(theta[i, 0]) * T06[0, 0] + np.sin(theta[i, 0]) * T06[1, 0]
        D = np.cos(theta[i, 0]) * T06[0, 1] + np.sin(theta[i, 0]) * T06[1, 1]
        E = np.cos(theta[i, 0]) * T06[0, 2] + np.sin(theta[i, 0]) * T06[1, 2]
        F = np.cos(theta[i, 0]) * T06[0, 3] + np.sin(theta[i, 0]) * T06[1, 3]
        G = np.cos(theta[i, 5]) * T06[2, 1] + np.sin(theta[i, 5]) * T06[2, 0]
        A = d[4] * (np.sin(theta[i, 5]) * C + np.cos(theta[i, 5]) * D) - d[5] * E + F
        B = T06[2, 3] - d[0] - T06[2, 2] * d[5] + d[4] * G

        # theta3
        if A * A + B * B <= (a[1] + a[2]) * (a[1] + a[2]):
            theta[i, 2] = math.acos((A * A + B * B - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]))
            theta[i + 1, 2] = -theta[i, 2]
        else:
            theta[i, 2] = 0
            theta[i + 1, 2] = 0

    # theta2 theta4
    for i in range(8):
        C = np.cos(theta[i, 0]) * T06[0, 0] + np.sin(theta[i, 0]) * T06[1, 0]
        D = np.cos(theta[i, 0]) * T06[0, 1] + np.sin(theta[i, 0]) * T06[1, 1]
        E = np.cos(theta[i, 0]) * T06[0, 2] + np.sin(theta[i, 0]) * T06[1, 2]
        F = np.cos(theta[i, 0]) * T06[0, 3] + np.sin(theta[i, 0]) * T06[1, 3]
        G = np.cos(theta[i, 5]) * T06[2, 1] + np.sin(theta[i, 5]) * T06[2, 0]
        A = d[4] * (np.sin(theta[i, 5]) * C + np.cos(theta[i, 5]) * D) - d[5] * E + F
        B = T06[2, 3] - d[0] - T06[2, 2] * d[5] + d[4] * G

        M = ((a[2] * np.cos(theta[i, 2]) + a[1]) * B - a[2] * np.sin(theta[i, 2]) * A) / (a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * np.cos(theta[i, 2]))
        N = (A + a[2] * np.sin(theta[i, 2]) * M) / (a[2] * np.cos(theta[i, 2]) + a[1])
        theta[i, 1] = math.atan2(M, N)

        # theta4
        th = math.atan2((-np.sin(theta[i, 5]) * C - np.cos(theta[i, 5]) * D), G) - theta[i, 1] - theta[i, 2]
        if th > np.pi:
            th -= 2 * np.pi
        if th < -np.pi:
            th += 2 * np.pi
        theta[i, 3] = th

        for j in range(6):
            th = theta[i, j]
            # 角度判断,如果不在[-180,180]调整
            if th > np.pi:
                th -= 2 * np.pi
            if th < -np.pi:
                th += 2 * np.pi
       
    return r2d(theta)

# 选择最接近的角度
def choose_right_angle(theta_previous, theta_current, curve_points):

    cumulative_error = np.zeros((1, 8))
    weights = np.array([60, 50, 40, 3, 2, 1])
    for i in range(8):
        err_theta = abs(theta_current[i] - theta_previous[0])
        err_T = np.linalg.norm(curve_points - Forward_Kinematics(d, a, alpha, theta_current[i])[:3, -1])
        cumulative_error[0, i] = np.dot(err_theta, weights)

    min_error_index = np.argmin(cumulative_error)

    return min_error_index

if __name__ == "__main__":
    
    #DH参数
    d = np.array([0.162, 0.00000, 0.00000, 0.133, 0.1, 0.1])
    a = np.array([0.000000, -0.4250, -0.392, 0.000000, 0.000000, 0.000000])
    alpha = np.array([np.pi/2, 0.00000, 0.00000, np.pi/2, -np.pi/2, 0.000000])


    joint = np.array([0, 0, 0, 0, 0, 0])
    theta_previous = joint
    T06_current = Forward_Kinematics(d, a, alpha, joint)
        
    flag = 0
    while flag == 0:
        T06_current[:3, -1] = curve_points[i]
        theta_current = Inverse_Kinematics(d, a, T06_current) 
        min_error_index = choose_right_angle(theta_previous, theta_current, curve_points[i])
        joint = theta_current[min_error_index]
        theta_previous = joint
        pos = Forward_Kinematics(d, a, alpha, joint)
        print(pos)
    
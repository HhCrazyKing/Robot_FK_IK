import numpy as np
import math
from math import sqrt as sqrt

'''保留小数点后七位，并且在第七位后进行四舍五入'''
def atan2(first, second):
    return round(math.atan2(first,second), 7)

def sin(radians_angle):
    return round(math.sin(radians_angle), 7)

def cos(radians_angle):
    return round(math.cos(radians_angle), 7)

def acos(value):
    return round(math.acos(value), 7)

def round_value(value):
    return round(value, 7)

def Forward_Kinematics(th1,th2,th3,th4,th5,th6):
    T1 = np.array([[cos(th1) , 0 , sin(th1) , 0],
                   [sin(th1) , 0 , -cos(th1) , 0],
                   [0 , 1 , 0 , d1],
                   [0 , 0 , 0 , 1]])
    T2 = np.array([[cos(th2) , -sin(th2) , 0 , a2*cos(th2)],
                   [sin(th2) , cos(th2) ,  0 , a2*sin(th2)],
                   [0 , 0 , 1 , 0],
                   [0 , 0 , 0 , 1]])
    T3 = np.array([[cos(th3), -sin(th3), 0, a3 * cos(th3)],
                   [sin(th3), cos(th3), 0, a3 * sin(th3)],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    T4 = np.array([[cos(th4), 0, sin(th4), 0],
                   [sin(th4), 0, -cos(th4), 0],
                   [0, 1, 0, d4],
                   [0, 0, 0, 1]])
    T5 = np.array([[cos(th5), 0, -sin(th5), 0],
                   [sin(th5), 0, cos(th5), 0],
                   [0, -1, 0, d5],
                   [0, 0, 0, 1]])
    T6 = np.array([[cos(th6), -sin(th6), 0, 0],
                   [sin(th6), cos(th6), 0, 0],
                   [0, 0, 1, d6],
                   [0, 0, 0, 1]])
    T1 = np.mat(T1)
    T2 = np.mat(T2)
    T3 = np.mat(T3)
    T4 = np.mat(T4)
    T5 = np.mat(T5)
    T6 = np.mat(T6)
    T60 = T1*T2*T3*T4*T5*T6
    #以下代码开始求机械臂的位姿
    gama = 0
    beta = 0
    Alpha = 0
    if T60[2, 0] != -1:  # beta != 90°
        beta = atan2(-T60[2, 0], sqrt(T60[0, 0] ** 2 + T60[1, 0] ** 2))
        Alpha = atan2(T60[1, 0] / cos(beta), T60[0, 0] / cos(beta))
        gama = atan2(T60[2, 1] / cos(beta), T60[2, 2] / cos(beta))
    elif T60[2, 0] == -1:
        beta = math.radians(90)
        Alpha = 0
        gama = atan2(T60[0, 1], T60[1, 1])
    elif T60[2, 0] == 1:
        beta = math.radians(-90)
        Alpha = 0
        gama = -atan2(T60[0, 1], T60[1, 1])
    X = T60[0,3]
    Y = T60[1,3]
    Z = T60[2,3]

    return [X,Y,Z,gama,beta,Alpha]

#运动学逆解函数
def Inverse_Kinematics(X, Y, Z, gama, beta, alpha):  # 需要给一个三维坐标以及位姿  绕x轴、y轴、z轴旋转
    #求RPY
    T_goat = np.array([[(cos(alpha) * cos(beta)), (cos(alpha) * sin(beta) * sin(gama)) - (sin(alpha) * cos(gama)),(cos(alpha) * sin(beta) * cos(gama)) + (sin(alpha) * sin(gama)), X],
                       [sin(alpha) * cos(beta), (sin(alpha) * sin(beta) * sin(gama)) + (cos(alpha) * cos(gama)),(sin(alpha) * sin(beta) * cos(gama)) - (cos(alpha) * sin(gama)), Y],
                       [-sin(beta), cos(beta) * sin(gama), cos(beta) * cos(gama), Z],
                       [0, 0, 0, 1]])
    nx = T_goat[0, 0]
    ny = T_goat[1, 0]
    nz = T_goat[2, 0]
    ox = T_goat[0, 1]
    oy = T_goat[1, 1]
    oz = T_goat[2, 1]
    ax = T_goat[0, 2]
    ay = T_goat[1, 2]
    az = T_goat[2, 2]
    px = T_goat[0, 3]
    py = T_goat[1, 3]
    pz = T_goat[2, 3]

    nx = round_value(nx)
    ny = round_value(ny)
    nz = round_value(nz)
    ox = round_value(ox)
    oy = round_value(oy)
    oz = round_value(oz)
    ax = round_value(ax)
    ay = round_value(ay)
    az = round_value(az)
    px = round_value(px)
    py = round_value(py)
    pz = round_value(pz)

    m = d6 * ay - py
    n = ax * d6 - px

    m = round_value(m)
    n = round_value(n)

    tempa = m**2 + n**2 - d4**2

    tempa = round_value(tempa)

    if tempa < 0:
        print("很抱歉，输入的参数角度有问题!",m**2 + n**2 - d4**2)
        return 0
    else:
        #计算theta1(两个解)
        theta1_1 = atan2(m,n) - atan2(d4, sqrt(tempa))
        theta1_2 = atan2(m,n) - atan2(d4,-sqrt(tempa))

        theta1_1 = round_value(theta1_1)
        theta1_2 = round_value(theta1_2)
        #计算theta5(四个解)
        theta5_1 = acos(ax * sin(theta1_1) - ay * cos(theta1_1))
        theta5_2 = -acos(ax * sin(theta1_1) - ay * cos(theta1_1))
        theta5_3 = acos(ax * sin(theta1_2) - ay * cos(theta1_2))
        theta5_4 = -acos(ax * sin(theta1_2) - ay * cos(theta1_2))

        theta5_1 = round_value(theta5_1)
        theta5_2 = round_value(theta5_2)
        theta5_3 = round_value(theta5_3)
        theta5_4 = round_value(theta5_4)

        #计算theta6(四个解)
        mm_1 = nx * sin(theta1_1) - ny * cos(theta1_1)
        nn_1 = ox * sin(theta1_1) - oy * cos(theta1_1)
        mm_2 = nx * sin(theta1_2) - ny * cos(theta1_2)
        nn_2 = ox * sin(theta1_2) - oy * cos(theta1_2)

        mm_1 = round_value(mm_1)
        nn_1 = round_value(nn_1)
        mm_2 = round_value(mm_2)
        nn_2 = round_value(nn_2)

        theta6_1 = atan2(mm_1, nn_1) - atan2(sin(theta5_1), 0)
        theta6_2 = atan2(mm_1, nn_1) - atan2(sin(theta5_2), 0)
        theta6_3 = atan2(mm_2, nn_2) - atan2(sin(theta5_3), 0)
        theta6_4 = atan2(mm_2, nn_2) - atan2(sin(theta5_4), 0)

        theta6_1 = round_value(theta6_1)
        theta6_2 = round_value(theta6_2)
        theta6_3 = round_value(theta6_3)
        theta6_4 = round_value(theta6_4)

        #计算theta3(八个解)
        mmm_1 = d5 * (sin(theta6_1) * (nx * cos(theta1_1) + ny * sin(theta1_1)) + cos(theta6_1) * (ox * cos(theta1_1) + oy * sin(theta1_1))) - d6 * (ax * cos(theta1_1) + ay * sin(theta1_1)) + px * cos(theta1_1) + py * sin(theta1_1)
        mmm_2 = d5 * (sin(theta6_2) * (nx * cos(theta1_1) + ny * sin(theta1_1)) + cos(theta6_2) * (ox * cos(theta1_1) + oy * sin(theta1_1))) - d6 * (ax * cos(theta1_1) + ay * sin(theta1_1)) + px * cos(theta1_1) + py * sin(theta1_1)
        mmm_3 = d5 * (sin(theta6_3) * (nx * cos(theta1_2) + ny * sin(theta1_2)) + cos(theta6_3) * (ox * cos(theta1_2) + oy * sin(theta1_2))) - d6 * (ax * cos(theta1_2) + ay * sin(theta1_2)) + px * cos(theta1_2) + py * sin(theta1_2)
        mmm_4 = d5 * (sin(theta6_4) * (nx * cos(theta1_2) + ny * sin(theta1_2)) + cos(theta6_4) * (ox * cos(theta1_2) + oy * sin(theta1_2))) - d6 * (ax * cos(theta1_2) + ay * sin(theta1_2)) + px * cos(theta1_2) + py * sin(theta1_2)

        mmm_1 = round_value(mmm_1)
        mmm_2 = round_value(mmm_2)
        mmm_3 = round_value(mmm_3)
        mmm_4 = round_value(mmm_4)

        nnn_1 = d5 * (oz * cos(theta6_1) + nz * sin(theta6_1)) + pz - d1 - az * d6
        nnn_2 = d5 * (oz * cos(theta6_2) + nz * sin(theta6_2)) + pz - d1 - az * d6
        nnn_3 = d5 * (oz * cos(theta6_3) + nz * sin(theta6_3)) + pz - d1 - az * d6
        nnn_4 = d5 * (oz * cos(theta6_4) + nz * sin(theta6_4)) + pz - d1 - az * d6

        nnn_1 = round_value(nnn_1)
        nnn_2 = round_value(nnn_2)
        nnn_3 = round_value(nnn_3)
        nnn_4 = round_value(nnn_4)

        one = (mmm_1 ** 2 + nnn_1 ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
        two = (mmm_2 ** 2 + nnn_2 ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
        three = (mmm_3 ** 2 + nnn_3 ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
        four = (mmm_4 ** 2 + nnn_4 ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)

        one = round_value(one)
        two = round_value(two)
        three = round_value(three)
        four = round_value(four)

        #有可能会超过范围，需要限制一下,因为acos函数的自变量取值范围为：-1~1
        if one > 1:
            one = 1
        if two > 1:
            two = 1
        if three > 1:
            three = 1
        if four > 1:
            four = 1
        if one < -1:
            one = -1
        if two < -1:
            two = -1
        if three < -1:
            three = -1
        if four < -1:
            four = -1

        theta3_1 = acos(one)
        theta3_2 = -acos(one)
        theta3_3 = acos(two)
        theta3_4 = -acos(two)
        theta3_5 = acos(three)
        theta3_6 = -acos(three)
        theta3_7 = acos(four)
        theta3_8 = -acos(four)

        theta3_1 = round_value(theta3_1)
        theta3_2 = round_value(theta3_2)
        theta3_3 = round_value(theta3_3)
        theta3_4 = round_value(theta3_4)
        theta3_5 = round_value(theta3_5)
        theta3_6 = round_value(theta3_6)
        theta3_7 = round_value(theta3_7)
        theta3_8 = round_value(theta3_8)

        #计算theta2(八个解)
        s2_1 = ((a3 * cos(theta3_1) + a2) * nnn_1 - a3 * sin(theta3_1) * mmm_1) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_1))
        s2_2 = ((a3 * cos(theta3_2) + a2) * nnn_1 - a3 * sin(theta3_2) * mmm_1) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_2))
        s2_3 = ((a3 * cos(theta3_3) + a2) * nnn_2 - a3 * sin(theta3_3) * mmm_2) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_3))
        s2_4 = ((a3 * cos(theta3_4) + a2) * nnn_2 - a3 * sin(theta3_4) * mmm_2) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_4))
        s2_5 = ((a3 * cos(theta3_5) + a2) * nnn_3 - a3 * sin(theta3_5) * mmm_3) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_5))
        s2_6 = ((a3 * cos(theta3_6) + a2) * nnn_3 - a3 * sin(theta3_6) * mmm_3) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_6))
        s2_7 = ((a3 * cos(theta3_7) + a2) * nnn_4 - a3 * sin(theta3_7) * mmm_4) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_7))
        s2_8 = ((a3 * cos(theta3_8) + a2) * nnn_4 - a3 * sin(theta3_8) * mmm_4) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * cos(theta3_8))

        s2_1 = round_value(s2_1)
        s2_2 = round_value(s2_2)
        s2_3 = round_value(s2_3)
        s2_4 = round_value(s2_4)
        s2_5 = round_value(s2_5)
        s2_6 = round_value(s2_6)
        s2_7 = round_value(s2_7)
        s2_8 = round_value(s2_8)

        c2_1 = (mmm_1 + a3 * sin(theta3_1) * s2_1) / (a3 * cos(theta3_1) + a2)
        c2_2 = (mmm_1 + a3 * sin(theta3_2) * s2_2) / (a3 * cos(theta3_2) + a2)
        c2_3 = (mmm_2 + a3 * sin(theta3_3) * s2_3) / (a3 * cos(theta3_3) + a2)
        c2_4 = (mmm_2 + a3 * sin(theta3_4) * s2_4) / (a3 * cos(theta3_4) + a2)
        c2_5 = (mmm_3 + a3 * sin(theta3_5) * s2_5) / (a3 * cos(theta3_5) + a2)
        c2_6 = (mmm_3 + a3 * sin(theta3_6) * s2_6) / (a3 * cos(theta3_6) + a2)
        c2_7 = (mmm_4 + a3 * sin(theta3_7) * s2_7) / (a3 * cos(theta3_7) + a2)
        c2_8 = (mmm_4 + a3 * sin(theta3_8) * s2_8) / (a3 * cos(theta3_8) + a2)

        c2_1 = round_value(c2_1)
        c2_2 = round_value(c2_2)
        c2_3 = round_value(c2_3)
        c2_4 = round_value(c2_4)
        c2_5 = round_value(c2_5)
        c2_6 = round_value(c2_6)
        c2_7 = round_value(c2_7)
        c2_8 = round_value(c2_8)

        theta2_1 = atan2(s2_1, c2_1)
        theta2_2 = atan2(s2_2, c2_2)
        theta2_3 = atan2(s2_3, c2_3)
        theta2_4 = atan2(s2_4, c2_4)
        theta2_5 = atan2(s2_5, c2_5)
        theta2_6 = atan2(s2_6, c2_6)
        theta2_7 = atan2(s2_7, c2_7)
        theta2_8 = atan2(s2_8, c2_8)

        theta2_1 = round_value(theta2_1)
        theta2_2 = round_value(theta2_2)
        theta2_3 = round_value(theta2_3)
        theta2_4 = round_value(theta2_4)
        theta2_5 = round_value(theta2_5)
        theta2_6 = round_value(theta2_6)
        theta2_7 = round_value(theta2_7)
        theta2_8 = round_value(theta2_8)

        #计算theta4
        theta4_1 = atan2(-sin(theta6_1) * (nx * cos(theta1_1) + ny * sin(theta1_1)) - cos(theta6_1) * (ox * cos(theta1_1) + oy * sin(theta1_1)), oz * cos(theta6_1) + nz * sin(theta6_1)) - theta2_1 - theta3_1
        theta4_2 = atan2(-sin(theta6_1) * (nx * cos(theta1_1) + ny * sin(theta1_1)) - cos(theta6_1) * (ox * cos(theta1_1) + oy * sin(theta1_1)), oz * cos(theta6_1) + nz * sin(theta6_1)) - theta2_2 - theta3_2
        theta4_3 = atan2(-sin(theta6_2) * (nx * cos(theta1_1) + ny * sin(theta1_1)) - cos(theta6_2) * (ox * cos(theta1_1) + oy * sin(theta1_1)), oz * cos(theta6_2) + nz * sin(theta6_2)) - theta2_3 - theta3_3
        theta4_4 = atan2(-sin(theta6_2) * (nx * cos(theta1_1) + ny * sin(theta1_1)) - cos(theta6_2) * (ox * cos(theta1_1) + oy * sin(theta1_1)), oz * cos(theta6_2) + nz * sin(theta6_2)) - theta2_4 - theta3_4
        theta4_5 = atan2(-sin(theta6_3) * (nx * cos(theta1_2) + ny * sin(theta1_2)) - cos(theta6_3) * (ox * cos(theta1_2) + oy * sin(theta1_2)), oz * cos(theta6_3) + nz * sin(theta6_3)) - theta2_5 - theta3_5
        theta4_6 = atan2(-sin(theta6_3) * (nx * cos(theta1_2) + ny * sin(theta1_2)) - cos(theta6_3) * (ox * cos(theta1_2) + oy * sin(theta1_2)), oz * cos(theta6_3) + nz * sin(theta6_3)) - theta2_6 - theta3_6
        theta4_7 = atan2(-sin(theta6_4) * (nx * cos(theta1_2) + ny * sin(theta1_2)) - cos(theta6_4) * (ox * cos(theta1_2) + oy * sin(theta1_2)), oz * cos(theta6_4) + nz * sin(theta6_4)) - theta2_7 - theta3_7
        theta4_8 = atan2(-sin(theta6_4) * (nx * cos(theta1_2) + ny * sin(theta1_2)) - cos(theta6_4) * (ox * cos(theta1_2) + oy * sin(theta1_2)), oz * cos(theta6_4) + nz * sin(theta6_4)) - theta2_8 - theta3_8

        theta4_1 = round_value(theta4_1)
        theta4_2 = round_value(theta4_2)
        theta4_3 = round_value(theta4_3)
        theta4_4 = round_value(theta4_4)
        theta4_5 = round_value(theta4_5)
        theta4_6 = round_value(theta4_6)
        theta4_7 = round_value(theta4_7)
        theta4_8 = round_value(theta4_8)

        return [[theta1_1, theta2_1, theta3_1, theta4_1, theta5_1, theta6_1],
                [theta1_1, theta2_2, theta3_2, theta4_2, theta5_1, theta6_1],
                [theta1_1, theta2_3, theta3_3, theta4_3, theta5_2, theta6_2],
                [theta1_1, theta2_4, theta3_4, theta4_4, theta5_2, theta6_2],
                [theta1_2, theta2_5, theta3_5, theta4_5, theta5_3, theta6_3],
                [theta1_2, theta2_6, theta3_6, theta4_6, theta5_3, theta6_3],
                [theta1_2, theta2_7, theta3_7, theta4_7, theta5_4, theta6_4],
                [theta1_2, theta2_8, theta3_8, theta4_8, theta5_4, theta6_4]]

'''
    gama : 绕x轴旋转; beta : 绕y轴旋转; alpha : 绕z轴旋转
    这个也可以用来计算末端执行器位姿
    print("gama:",atan2(T60[2,1], T60[2,2]) )  
    print("beta:",atan2(-T60[2,0], sqrt(T60[2,1] ** 2 + T60[2,2] ** 2)))  
    print("alpha:",atan2(T60[1,0], T60[0,0]))  
'''

PI = math.pi
alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = PI/2 , 0 , 0 , PI/2 , -PI/2 , 0
a1,a2,a3,a4,a5,a6 = 0 , -425 , -392 , 0 , 0 , 0
d1,d2,d3,d4,d5,d6 = 162 , 0 , 0 , 133 , 100 , 100

theta1,theta2,theta3,theta4,theta5,theta6 = math.radians(80),math.radians(-90),math.radians(-30),math.radians(-90),math.radians(40),math.radians(61)
pos = Forward_Kinematics(theta1,theta2,theta3,theta4,theta5,theta6)
print("通过正运动学得到得X,Y,Z,gama,beta,alpha值: ",pos)
angel = Inverse_Kinematics(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])
if angel:
    for m in range(8):
        for n in range(6):
            if angel[m][n] < -PI:
                angel[m][n] = angel[m][n] + 2*PI
            elif angel[m][n] > PI:
                angel[m][n] = angel[m][n] - 2*PI
    print("通过运动学逆解获得的八组角度解分别为：")
    for i in range(8):
        for j in range(6):
             print(math.degrees(angel[i][j]),end='\t')
             #print(angel[i][j], end='\t')
        print()

'''
#输入theta1~theta2得值后，得到的运动学正解值会传入运动学逆解中，并且得到的几组逆解角度值会再次带入正解中判断是否是有效解，最后输出的是有效解;
pos = Forward_Kinematics(theta1,theta2,theta3,theta4,theta5,theta6)
angel = Inverse_Kinematics(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])
print("通过正运动学得到得X,Y,Z,gama,beta,alpha值: ",pos)
if angel:
    for m in range(8):
        for n in range(6):
            if angel[m][n] < -PI:
                angel[m][n] = angel[m][n] + 2*PI
            elif angel[m][n] > PI:
                angel[m][n] = angel[m][n] - 2*PI
    print("===============================得到的八组角度解分别为（单位为度）===============================")
    for i in range(8):
        for col in range(6):
            print(math.degrees(angel[i][col]), end='\t')
        print()

    num = 0
    list = [0,0,0,0,0,0,0,0]
    for i in range(8):
        position = Forward_Kinematics(angel[i][0],angel[i][1],angel[i][2],angel[i][3],angel[i][4],angel[i][5])
        print("第%d组解为：" % (i + 1))
        print(position)
        if ((abs(position[0] - pos[0]) < 0.1 and abs(position[1] - pos[1]) < 0.1 and abs(position[2] - pos[2]) < 0.1 and abs(position[3] - pos[3]) < 0.1 and abs(position[4] - pos[4]) < 0.1) and ((abs(position[5] - pos[5]) < 0.1) or abs(position[5] + pos[5]) < 0.1)):
            list[num] = i
            num = num + 1
            print("第%d组解有效"%(i+1))
    if num == 0:
        print("八组解均无效!")
    else:
        print("一共有%d组解有效！分别是："%num)
        #转换成角度值输出
        print("===============================最终有效得角度值为：（单位为度）===============================")
        for i in range(num):
            for col in range(6):
                print(math.degrees(angel[list[i]][col]),end='\t')
            print()
else:
    print("运动学逆解计算不出来")
'''


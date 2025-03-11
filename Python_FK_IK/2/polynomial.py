import FK_IK_Test
import math
"""
    description     :       五次多项式求解机械臂运动轨迹，point to point
    param           :       @param_one  :   [[角位移1,角位移2,角位移3,角位移4,角位移5,角位移6]，速度，加速度，时间]
                            @param_two  :   [[角位移1,角位移2,角位移3,角位移4,角位移5,角位移6]，速度，加速度，时间]
                            @num_seg    :   把线段平分的段数
    ret             :       返回的是六个角度值，每个角度值被分成num_seg段的角位移列表,类型：[[],[],[],[],[],[]]
    example:
    #调用函数，检查轨迹生成函数是否正确
    param1 = [[0,0,0,0,0,0], 0, 0, 0]
    param2 = [[10,20,30,40,50,60], 10, 20, 3]
    num = 100
    angle_a = five_polynomial(param1, param2, num)

    for m in range(len(angle_a)):
        for n in range(len(angle_a[0])):
            print(angle_a[m][n])
"""
def five_polynomial(param_one,param_two,num_seg):
    #机械臂六个角位移的角度值
    angle = []

    #角位移、角速度、角加速度、时间差
    t = param_two[3] - param_one[3]
    v_s, a_s, t_s = param_one[1], param_one[2], param_one[3]
    v_e, a_e = param_two[1], param_two[2]

    # 把point to point整个轨迹的时间分成num_seg份
    delta = t / num_seg

    for i in range(len(param_one[0])):   #param_one[0]:为六个角度值,长度为6
        #每个角度的角位移不同
        th_s = param_one[0][i]
        th_e = param_two[0][i]
        #求六个未知数
        a0 = th_s
        a1 = v_s
        a2 = 0.5*a_s
        a3 = (20*(th_e-th_s)-(8*v_e+12*v_s)*t+(a_e-3*a_s)*t*t)/(2*pow(t,3))
        a4 = (-30*(th_e-th_s)+(14*v_e+16*v_s)*t-(2*a_e-3*a_s)*t*t)/(2*pow(t,4))
        a5 = (12*(th_e-th_s)-(6*v_e+6*v_s)*t+(a_e-a_s)*t*t)/(2*pow(t,5))

        tx = t_s
        angle_mid = []   #收集的是每次循环的角度值
        #print(i+1)
        for j in range(num_seg+1):
            ty = a0 + a1*(tx-t_s) + a2*pow((tx-t_s),2) + a3*pow((tx-t_s),3) + a4*pow((tx-t_s),4) + a5*pow((tx-t_s),5)
            tx = tx+delta
            angle_mid.append(ty)
        #收集六组角度值，每组角度值被分为num_seg段
        angle.append(angle_mid)

    return angle

"""
    description        ：       关节路径规划,传入起始点坐标和目标点坐标,进行point to point路径规划
    param              :        @point_one   :  起始点坐标及位姿[x,y,z,gama,beta,alpha]
                                @point_two   :  目标点坐标及位姿[x,y,z,gama,beta,alpha]
                                @param_one   :  [角速度，角加速度，point_one的时间]
                                @param_two   :  [角速度，角加速度，point_two的时间]
                                @last_angle  :  上次的有效组解，便于此次筛选有效组解
    ret                :        返回0，则运动学逆解求解不出来输入的两个坐标,否则返回六个角度值，每个角度值被分成num_seg段的角位移列表,类型：[[],[],[],[],[],[]]
    example:
    one = [0.05892516056903008, -100.58999278055012, 504.00000556965523, -1.5707964, -0.0, -3.1412437]
    two = [0.05892516056903008, -100.58999278055012, 504.00000556965523-100.1, -1.5707964, -0.0, -3.1412437]
    param1 = [0, 0, 0]
    param2 = [0.01, 0.01, 3]
    last_ang = [[0, 0, 0, 0, 0, 0]]
    
    ag = Point_to_Point(one, two, param1, param2, last_ang)
    for m in range(len(ag)):
        print("第{}个角度规划为: ".format(m + 1))
        print("长度: ", len(ag[m]))
        for n in range(len(ag[0])):
            print(ag[m][n])
"""
def Point_to_Point(point_one, point_two, param_one, param_two, last_angle):
    #把两个坐标值转换成角度值
    ik_one_angle = FK_IK_Test.Inverse_Kinematics(point_one[0], point_one[1], point_one[2], point_one[3], point_one[4], point_one[5])
    ik_two_angle = FK_IK_Test.Inverse_Kinematics(point_two[0], point_two[1], point_two[2], point_two[3], point_two[4], point_two[5])

    # 判断运动学逆解是否解出结果
    if ik_one_angle == 0 or ik_two_angle == 0:
        return 0

    #整理角度
    for m in range(len(ik_one_angle)):
        for n in range(6):
            if ik_one_angle[m][n] < -math.pi:
                ik_one_angle[m][n] = ik_one_angle[m][n] + 2 * math.pi
            elif ik_one_angle[m][n] > math.pi:
                ik_one_angle[m][n] = ik_one_angle[m][n] - 2 * math.pi
    for m in range(len(ik_two_angle)):
        for n in range(6):
            if ik_two_angle[m][n] < -math.pi:
                ik_two_angle[m][n] = ik_two_angle[m][n] + 2 * math.pi
            elif ik_two_angle[m][n] > math.pi:
                ik_two_angle[m][n] = ik_two_angle[m][n] - 2 * math.pi

    one_angle, two_angle = [], []
    bias_one_angle, bias_two_angle = [], []
    min_weight = [9999, -1]  # 寻找bias_angle中的最小偏差值   第一个参数存储的是最小值，第二个参数存储的是最小值所在的索引

    #进行运动学正解，筛选出有效组解
    for i in range(len(ik_one_angle)):
        position = FK_IK_Test.Forward_Kinematics(ik_one_angle[i][0], ik_one_angle[i][1], ik_one_angle[i][2], ik_one_angle[i][3], ik_one_angle[i][4], ik_one_angle[i][5])
        if abs(position[0] - point_one[0]) < 0.1 and abs(position[1] - point_one[1]) < 0.1 and abs(position[2] - point_one[2]) < 0.1 and \
           abs(position[3] - point_one[3]) < 0.1 and abs(position[4] - point_one[4]) < 0.1 and abs(position[5] - point_one[5]) < 0.1:
            one_angle.append(ik_one_angle[i])
    for i in range(len(ik_two_angle)):
        position = FK_IK_Test.Forward_Kinematics(ik_two_angle[i][0], ik_two_angle[i][1], ik_two_angle[i][2], ik_two_angle[i][3], ik_two_angle[i][4], ik_two_angle[i][5])
        if abs(position[0] - point_two[0]) < 0.1 and abs(position[1] - point_two[1]) < 0.1 and abs(position[2] - point_two[2]) < 0.1 and \
           abs(position[3] - point_two[3]) < 0.1 and abs(position[4] - point_two[4]) < 0.1 and abs(position[5] - point_two[5]) < 0.1:
            two_angle.append(ik_two_angle[i])

    #判断是否为空
    if len(one_angle) == 0:
        return 0
    if len(two_angle) == 0:
        return 0

    #矫正角度
    for i in range(len(one_angle)):
        one_angle[i][1] = one_angle[i][1] + math.radians(90)  # 用于初始位姿矫正
        one_angle[i][3] = one_angle[i][3] + math.radians(90)  # 用于初始位姿矫正
    for i in range(len(two_angle)):
        two_angle[i][1] = two_angle[i][1] + math.radians(90)  # 用于初始位姿矫正
        two_angle[i][3] = two_angle[i][3] + math.radians(90)  # 用于初始位姿矫正

    #整理角度
    for m in range(len(one_angle)):
        for n in range(6):
            if one_angle[m][n] < -math.pi:
                one_angle[m][n] = one_angle[m][n] + 2 * math.pi
            elif one_angle[m][n] > math.pi:
                one_angle[m][n] = one_angle[m][n] - 2 * math.pi
    for m in range(len(two_angle)):
        for n in range(6):
            if two_angle[m][n] < -math.pi:
                two_angle[m][n] = two_angle[m][n] + 2 * math.pi
            elif two_angle[m][n] > math.pi:
                two_angle[m][n] = two_angle[m][n] - 2 * math.pi

    #筛选出转动幅度最小的有效组解,先对第一个位置进行筛选
    for i in range(len(one_angle)):  #有效组解依次和上一次机械臂转动的各关节角度进行求偏差，再取权重累加,最后累加得到的值存入bias_angle列表中的
        bias_one_angle.append(0)
        for j in range(6):
            if j < 3:
                bias_one_angle[i] = bias_one_angle[i] + abs(one_angle[i][j] - last_angle[0][j]) * 0.7  # bias_angle:存储len(angle_result)数量的组的六个角度偏差之和
            elif j >= 3:
                bias_one_angle[i] = bias_one_angle[i] + abs(one_angle[i][j] - last_angle[0][j]) * 0.3  # bias_angle:存储len(angle_result)数量的组的六个角度偏差之和
    # 从bias_angle列表中找出最小值，并记录其索引，其索引指向的位置，就是这个机械臂运动到这个坐标点最终的角度值
    for i in range(len(bias_one_angle)):
        if bias_one_angle[i] < min_weight[0]:
            min_weight[0] = bias_one_angle[i]
            min_weight[1] = i
    if min_weight[1] == -1:
        return 0
    else:
        val_one_angle = one_angle[min_weight[1]]  #保存第一个坐标计算出的有效值

    # 筛选出转动幅度最小的有效组解,再对第二个位置进行筛选
    min_weight = [9999, -1]     #重新初始化，开始查找第二个坐标的有效角度组解值
    last_angle[0] = val_one_angle  #更新一下上一次的有效组角度值
    for i in range(len(two_angle)):  # 有效组解依次和上一次机械臂转动的各关节角度进行求偏差，再取权重累加,最后累加得到的值存入bias_angle列表中的
        bias_two_angle.append(0)
        for j in range(6):
            if j < 3:
                bias_two_angle[i] = bias_two_angle[i] + abs(two_angle[i][j] - last_angle[0][j]) * 0.7  # bias_angle:存储len(angle_result)数量的组的六个角度偏差之和
            elif j >= 3:
                bias_two_angle[i] = bias_two_angle[i] + abs(two_angle[i][j] - last_angle[0][j]) * 0.3  # bias_angle:存储len(angle_result)数量的组的六个角度偏差之和
        # 从bias_angle列表中找出最小值，并记录其索引，其索引指向的位置，就是这个机械臂运动到这个坐标点最终的角度值
    for i in range(len(bias_two_angle)):
        if bias_two_angle[i] < min_weight[0]:
            min_weight[0] = bias_two_angle[i]
            min_weight[1] = i
    if min_weight == -1:
        return 0
    else:
        val_two_angle = two_angle[min_weight[1]]  # 保存第一个坐标计算出的有效值
    last_angle[0] = val_two_angle

    #开始进行五次多项式的关节路径规划
    param_one = [val_one_angle, param_one[0], param_one[1], param_one[2]]    #六个角度值,角速度，角加速度，第一个坐标点的时间
    param_two = [val_two_angle, param_two[0], param_two[1], param_two[2]]    #六个角度值,角速度，角加速度，第二个坐标点的时间
    num_seg = 100
    pl_angle = five_polynomial(param_one, param_two, num_seg)

    return pl_angle

"""
    description        ：       关节路径规划,多个点之间的路径规划
    param              :        @point_list   :  格式: [[],[].....,[]]; 起始点坐标及位姿[x,y,z,gama,beta,alpha],中间点1、2.......坐标及位姿[x,y,z,gama,beta,alpha],终止点坐标及位姿[x,y,z,gama,beta,alpha]
                                @param_list   :  格式：[[],[].....,[]];  与point_list对应索引坐标的,角速度、角加速度、时间; 
    ret                :        @multi_angle  :  格式：[[[],[],[],[],[],[]],[[],[],[],[],[],[]].......,[[],[],[],[],[],[]]]; len(point_list)组（每组六个角度值），每个角度值101个元素
                                返回0，则表示@point_list的长度为0
    example:
    multi_point = [[0.05892516056903008, -100.58999278055012, 504.00000556965523, -1.5707964, -0.0, -3.1412437],
               [0.05892516056903008, -100.58999278055012, 504.00000556965523 - 100.1, -1.5707964, -0.0, -3.1412437],
               [0.05892516056903008, -100.58999278055012, 504.00000556965523, -1.5707964, -0.0, -3.1412437]]
    multi_param = [[0, 0, 0],
                   [0.01, 0.01, 3],
                   [0.02, 0.02, 6]]
    
    multi_ag = Multipoint_Interpolation(multi_point, multi_param)
    
    # 打印出来看一下
    for m in range(len(multi_ag)):  # 除原点（起始点）以外，坐标的个数，其实就是机械臂运动的路段数
        print("第{}段路程为 ：".format(m + 1))
        print("=============================================================================================")
        for n in range(len(multi_ag[m])):
            print("第{}个角度规划为: ".format(n + 1))
            for k in range(len(multi_ag[m][0])):
                print(multi_ag[m][n][k])                     
"""
def Multipoint_Interpolation(point_list, param_list):
    if len(point_list) == 0:
        return 0
    #初始化原点的last_ag
    last_ag = [[0, 0, 0, 0, 0, 0]]

    multi_angle = []
    #对所有的点进行遍历
    for i in range(len(point_list)-1):
        multi_angle.append(Point_to_Point(point_list[i], point_list[i+1], param_list[i], param_list[i+1], last_ag))

    return multi_angle


#测试代码
multi_point = [[-0.014399, -100.590004, 504.000000, -1.570796, 0.000349, -3.141244],
               [-0.014399, -100.590004, 504.000000-150, -1.570796, 0.000349, -3.141244],
               [-0.014399, -100.590004, 504.000000, -1.570796, 0.000349, -3.141244]]
multi_param = [[0, 0, 0],
               [0.01, 0.01, 3],
               [0.02, 0.02, 6]]

multi_ag = Multipoint_Interpolation(multi_point, multi_param)

for m in range(len(multi_ag)):  # 除原点（起始点）以外，坐标的个数，其实就是机械臂运动的路段数
        print("第{}段路程为 ：".format(m + 1))
        print("=============================================================================================")
        for n in range(len(multi_ag[m])):
            print("第{}个角度规划为: ".format(n + 1))
            for k in range(len(multi_ag[m][0])):
                print(multi_ag[m][n][k])

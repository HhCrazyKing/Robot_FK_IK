import numpy as np

# DH 参数表
dh_params = [
    [0, 0.162, 0     , np.pi / 2],    # shoulder
    [0, 0    , -0.425, 0        ],           # upper arm
    [0, 0    , -0.392, 0        ],          # forearm
    [0, 0.133, 0     , np.pi / 2],   # wrist 1
    [0, 0.1  , 0     , -np.pi / 2],  # wrist 2
    [0, 0.1  , 0     , 0],       # wrist 3
]

# 构造 DH 变换矩阵
def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# 正向运动学
def forward_kinematics(joint_angles):
    T = np.eye(4)  # 初始为单位矩阵
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        T_i = dh_transform(joint_angles[i] + theta, d, a, alpha)
        T = np.dot(T, T_i)  # 累乘每个关节的变换矩阵
    return T

# 示例关节角度（弧度）
joint_angles = [0, 0, 0, 0, np.pi/4, 0]
T_total = forward_kinematics(joint_angles)
print("末端执行器的变换矩阵：\n", T_total)

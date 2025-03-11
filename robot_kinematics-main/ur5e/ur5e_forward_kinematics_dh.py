import numpy as np

def dh_to_htm(alpha=None, a=None, d=None, phi=None):

    T = np.array([[np.cos(phi), -np.sin(phi)*np.cos(alpha), np.sin(phi)*np.sin(alpha), a*np.cos(phi)],
                  [np.sin(phi), np.cos(phi)*np.cos(alpha), -np.cos(phi) * np.sin(alpha), a * np.sin(phi)],
                  [0, np.sin(alpha), np.cos(alpha), d ],
                  [0, 0, 0, 1]])
    return T

def near_zero(z):
    """
    Determines whether a scalar is zero
    :param z: scalar
    :return: bool
    """
    return abs(z) < 1e-6


def forward_kinematics_dh(dh_table):

    fk = np.eye(4)

    for joint in dh_table:
        alpha, a, d, phi = joint
        tj = dh_to_htm(alpha, a, d, phi)
        fk = np.matmul(fk, tj)

    res = np.where(near_zero(fk), 0, np.round(fk, 3))
    return res


if __name__ == "__main__":

    # theta = [0.7854, -0.7854, 0.7854, -1.5708, -1.5708, 0.7854]
    theta = [0, 0, 0, 0, 0, 0]

    # UR5e specifications
    # W1 = 0.109
    # W2 = 0.082
    # L1 = 0.425
    # L2 = 0.392
    # H1 = 0.089
    # H2 = 0.095
    W1 = 0.133
    W2 = 0.100
    L1 = 0.425
    L2 = 0.392
    H1 = 0.162
    H2 = 0.100

    # DH Parameters Table, alpha, a, d, phi
    dh_table = np.array([[np.pi / 2  , 0   , H1 , theta[0]],
                         [0          , -L1 , 0  , theta[1]],
                         [0          , -L2 , 0  , theta[2]],
                         [np.pi / 2  , 0   , W1 , theta[3]],
                         [-np.pi / 2 , 0   , H2 , theta[4]],
                         [0          , 0   , W2 , theta[5]]])

    # FORWARD KINEMATICS applying DH
    fk_dh = forward_kinematics_dh(dh_table)

    print("Example: UR5e 6dof robot arm")
    print(f"\nDh Parameters: \n{dh_table}")
    print(f"\nForward Kinematics T0{dh_table.shape[0]} applying DH for the configuration {theta}: \n{fk_dh}")



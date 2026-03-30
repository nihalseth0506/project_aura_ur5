import numpy as np

def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])


def forward_chain_ur5(theta):
    d1 = 0.089159
    a2 = -0.425
    a3 = -0.39225
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823

    A1 = dh_transform(theta[0], d1, 0, np.pi/2)
    A2 = dh_transform(theta[1], 0, a2, 0)
    A3 = dh_transform(theta[2], 0, a3, 0)
    A4 = dh_transform(theta[3], d4, 0, np.pi/2)
    A5 = dh_transform(theta[4], d5, 0, -np.pi/2)
    A6 = dh_transform(theta[5], d6, 0, 0)

    T01 = A1
    T02 = T01 @ A2
    T03 = T02 @ A3
    T04 = T03 @ A4
    T05 = T04 @ A5
    T06 = T05 @ A6

    return T01, T02, T03, T04, T05, T06


def jacobian_ur5(theta):
    T01, T02, T03, T04, T05, T06 = forward_chain_ur5(theta)

    o0 = np.array([0,0,0])
    o1 = T01[:3,3]
    o2 = T02[:3,3]
    o3 = T03[:3,3]
    o4 = T04[:3,3]
    o5 = T05[:3,3]
    o6 = T06[:3,3]

    z0 = np.array([0,0,1])
    z1 = T01[:3,2]
    z2 = T02[:3,2]
    z3 = T03[:3,2]
    z4 = T04[:3,2]
    z5 = T05[:3,2]

    Jv1 = np.cross(z0, (o6 - o0))
    Jv2 = np.cross(z1, (o6 - o1))
    Jv3 = np.cross(z2, (o6 - o2))
    Jv4 = np.cross(z3, (o6 - o3))
    Jv5 = np.cross(z4, (o6 - o4))
    Jv6 = np.cross(z5, (o6 - o5))

    Jw1 = z0
    Jw2 = z1
    Jw3 = z2
    Jw4 = z3
    Jw5 = z4
    Jw6 = z5

    J = np.vstack([
        np.column_stack([Jv1, Jv2, Jv3, Jv4, Jv5, Jv6]),
        np.column_stack([Jw1, Jw2, Jw3, Jw4, Jw5, Jw6])
    ])

    return J

def forward_kinematics_ur5(theta):
    _, _, _, _, _, T06 = forward_chain_ur5(theta)
    position = T06[:3, 3]
    return position
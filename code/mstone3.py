from modern_robotics import *
import numpy as np


M0e = np.array([
    [1, 0, 0, 0.033],
    [0, 1, 0, 0],
    [0, 0, 1, 0.6546],
    [0, 0, 0, 1]    
])

Blist = np.array([
    [0, 0, 1, 0, 0.033, 0],
    [0, -1, 0, -0.5076, 0, 0],
    [0, -1, 0, -0.3526, 0, 0],
    [0, -1, 0, -0.2176, 0, 0],
    [0, 0, 1, 0, 0, 0]
]).T

theta_list= np.array([0, 0, 0.2, -1.6, 0]) 

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt):
    """Calculate the kinematic task-space feedforward plus feedback control law

    Args:
        x (_type_): current actual end-effector configuration
        X_d (_type_):  current end-effector reference configuration
        Xd_next (_type_):  end-effector reference configuration at the next timestep in the reference trajectory at a time Δt later. 
        Kp (_type_): p PI gain matrix
        Ki (_type_): i PI gain matrix
        dt (float): timestep Δt between reference trajectory configurations

    Returns:
        _type_: commanded end-effector twist in the end-effector frame
    """
    Xd_inv = TransInv(Xd)
    Vd_skew = (1/dt) * MatrixLog6(Xd_inv@Xd_next)

    Vd = np.array([Vd_skew[2,1],
                Vd_skew[0,2],
                Vd_skew[1,0],
                Vd_skew[0,3],
                Vd_skew[1,3],
                Vd_skew[2,3]
                ])
    print("X",X)
    print("Xd", Xd)
    Xerr_skew = MatrixLog6(TransInv(X)@Xd)
    Xerr = np.array([Xerr_skew[2,1],
            Xerr_skew[0,2],
            Xerr_skew[1,0],
            Xerr_skew[0,3],
            Xerr_skew [1,3],
            Xerr_skew[2,3]
            ])
    #theta_list = IKinBody(Blist, M0e, )
    Tb0 = np.array([
        [1, 0, 0, 0.1662],
        [0, 1, 0, 0],
        [0, 0, 1, 0.0026],
        [0, 0, 0, 1]
    ])
    Je = JacobianBody(Blist, theta_list) 
    Je_inv = np.linalg.pinv(Je)
    V = Adjoint(TransInv(X)@Xd)@Vd #+ Kp@Xerr + Ki
    u_th_dot = Je_inv@V
    T_0e = FKinBody(M0e, Blist, theta_list)
    Je_arm = JacobianBody(Blist, theta_list) 
    l = 0.47 / 2
    w = 0.3 / 2
    r = 0.0475
    tmp_arg = 1 / (l+w)
    Frog6 = (0.25*r) * np.array([
    [0,0,0,0],
    [0,0,0,0],
    [-tmp_arg, tmp_arg, tmp_arg, -tmp_arg],
    [1, 1, 1, 1],
    [-1, 1, -1, 1],
    [0,0,0,0]
    ])

    J_base = Adjoint(TransInv(T_0e)@TransInv(Tb0))@Frog6
    Je_tot = np.hstack((J_base,Je_arm))
    Je_inv = np.linalg.pinv(Je_tot)

    T_0e = FKinBody(M0e, Blist, theta_list)
    Je_arm = JacobianBody(Blist, theta_list) 


    Frog6 = (0.25*r) * np.array([
    [0,0,0,0],
    [0,0,0,0],
    [-tmp_arg, tmp_arg, tmp_arg, -tmp_arg],
    [1, 1, 1, 1],
    [-1, 1, -1, 1],
    [0,0,0,0]
    ])

    J_base = Adjoint(TransInv(T_0e)@TransInv(Tb0))@Frog6
    Je_tot = np.hstack((J_base,Je_arm))
    Je_inv = np.linalg.pinv(Je_tot)

    u_th_dot = Je_inv@V
    
    print("Vd:\n", Vd)
    print("\nV:\n", V)
    print("\nXerr:\n", Xerr)
    print("\nXerr\n", Xerr)
    print("u_th_dot\n", u_th_dot)

    return u_th_dot

dt = 0.01
Kp = 0
Ki = 0
Xd = np.array([
    [0, 0, 1, 0.5],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.5],
    [0, 0, 0, 1]
])
Xd_next = np.array([
    [0, 0, 1, 0.6],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.3],
    [0, 0, 0, 1]
])
X = np.array([
              [0.17, 0, 0.985, 0.387],
              [0, 1, 0, 0],
              [-0.985, 0, 0.17, 0.57],
              [0, 0, 0, 1]
              ])

FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)
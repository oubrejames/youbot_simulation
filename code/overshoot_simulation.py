from modern_robotics import *
import matplotlib.pyplot as plt

def NextState(current_config, velocities, dt, max_vel):
    #print("current config: ", current_config)
    old_chassis_config = []
    old_arm_joint_angles = []
    old_wheel_angles = []
    for i, o in enumerate(current_config): # i < 3: 
        if i < 3:
            old_chassis_config.append(o)
        elif i < 8:
            old_arm_joint_angles.append(o)
        else:
            old_wheel_angles.append(o)

    joint_speeds = []
    wheel_speeds = []
    for i, o in enumerate(velocities): # i < 3: 
        if i < 4:
            wheel_speeds.append(o)
        else:
            joint_speeds.append(o)

    old_chassis_config = np.array(old_chassis_config)
    old_arm_joint_angles = np.array(old_arm_joint_angles)
    joint_speeds = np.array(joint_speeds)
    old_wheel_angles = np.array(old_wheel_angles)
    wheel_speeds = np.array(wheel_speeds)

    new_arm_joint_angles = old_arm_joint_angles + joint_speeds*dt

    new_wheel_angles = old_wheel_angles + wheel_speeds*dt

    Vb = force@wheel_speeds.T
    wbz = Vb[0]
    if wbz == 0:
        d_qb = Vb
    else:
        d_qb = np.array([
            Vb[0],
            (Vb[1]*np.sin(Vb[0])+Vb[2]*(np.cos(Vb[0])-1))/Vb[0],
            (Vb[2]*np.sin(Vb[0])+Vb[1]*(1-np.cos(Vb[0])))/Vb[0]
        ])

    phi_k = old_chassis_config[0]

    d_q = np.array([
        [1, 0, 0],
        [0, np.cos(phi_k), -np.sin(phi_k)],
        [0, np.sin(phi_k), np.cos(phi_k)]
    ])@d_qb

    new_chassis_config = old_chassis_config + d_q*dt

    output = []
    for i in range(len(new_chassis_config)):
        output.append(new_chassis_config[i])

    for i in range(len(new_arm_joint_angles)):
        output.append(new_arm_joint_angles[i])

    for i in range(len(new_wheel_angles)):
        output.append(new_wheel_angles[i])

    return output

def TrajectoryGenerator(Tse_in, Tsc_in, Tsc_f, Tce_g, Tce_so, k):
    """Create a trajectory from an end-effector's starting position to a given goal configuration.

    Args:
    ----
        Tse_in: Initial configuration of the end-effector in the reference trajectory
        Tsc_in: Cube's initial configuration
        Tsc_f: cube's final desired position
        Tce_g: end-effector's configuration relative to the cube when grasping
        Tce_so: end-effector's standoff configuration above the cube, before and after grasping
                        relative to the cube
        k: number of trajectory reference configurations per 0.01 seconds

    Returns:
    -------
        matrix of complete trajectory to simulate

    Example of how to run code and generate a csv:

    Tsc_in = np.array([[1, 0, 0, 1],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.025],
                   [0, 0, 0, 1]
                   ])

    Tsc_f = np.array([[0, 1, 0, 0],
                    [-1, 0, 0, -1],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]
                    ])

    Tse_in = np.array([[0, 0, 1, 0],
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0.5],
                    [0, 0, 0, 1]
                    ])

    ang = 3*np.pi / 4
    Tce_so = np.array([[np.cos(ang), 0, np.sin(ang), 0],
                    [0, 1, 0, 0],
                    [-np.sin(ang), 0, np.cos(ang), 0.2],
                    [0, 0, 0, 1]
                    ])

    Tce_g = np.array([[np.cos(ang), 0, np.sin(ang), 0],
                    [0, 1, 0, 0],
                    [-np.sin(ang), 0, np.cos(ang), 0.0],
                    [0, 0, 0, 1]
                    ])

    k = 1 

    jectory = TrajectoryGenerator(Tse_in, Tsc_in, Tsc_f, Tce_g, Tce_so, k)
    np.savetxt("m2_traj.csv", jectory, delimiter = ",") 

    """
    ###################### Go to stand off ######################
    ex_time_so = 3
    N = int(ex_time_so/0.01)
    traj_2_so = np.zeros((N,13))
    
    # Compute the transformation matrix from world to stand off position
    Tse_so = np.matmul(Tsc_in, Tce_so)
    # Calculate screw trajectory from the initial position to the standoff 
    smol_traj_2_so = ScrewTrajectory(Tse_in, Tse_so, ex_time_so, N, 5)
    config_want = []
    # Create entries for CSV and add them to matrix
    for n in range(N):
        # Get rotation parts
        entry = []
        for k in range(3):
            for i in range(3):
                entry.append(smol_traj_2_so[n][k][i])
        # Add points
        entry.append(smol_traj_2_so[n][0][3])
        entry.append(smol_traj_2_so[n][1][3])
        entry.append(smol_traj_2_so[n][2][3])
        entry.append(0) # Gripper open
        traj_2_so[n] = entry
        config_want.append(smol_traj_2_so[n])


    #Go to grasp position
    current_Tse = smol_traj_2_so[-1]
    ex_time_2_cube_in = 1
    N = int(ex_time_2_cube_in/0.01)
    traj_2_cube_in = np.zeros((N,13))

    # Compute the transformation matrix from standoff to grasping position in world frame
    Tse_g = np.matmul(Tsc_in, Tce_g)

    # Calculate cartesian trajectory from the strandoff position to the grasping position
    smol_traj_2_cube = ScrewTrajectory(current_Tse, Tse_g, ex_time_2_cube_in, N, 5)

    # Create entries for CSV and add them to matrix
    for n in range(N):
        # Get rotation parts
        entry2 = []
        for k in range(3):
            for i in range(3):
                entry2.append(smol_traj_2_cube[n][k][i])
        # Add points
        entry2.append(smol_traj_2_cube[n][0][3])
        entry2.append(smol_traj_2_cube[n][1][3])
        entry2.append(smol_traj_2_cube[n][2][3])
        entry2.append(0)  # Gripper open
        traj_2_cube_in[n] = entry2
        config_want.append(smol_traj_2_cube[n])

    # Update total trajectory
    total_traj = np.vstack((traj_2_so, traj_2_cube_in))
    
    #Grasp 
    grasp = traj_2_cube_in[-1]
    grasp[-1] = 1
    for i in range(6):
        total_traj = np.vstack((total_traj, grasp))

    #Go back to stand off
    current_Tse = smol_traj_2_cube[-1]
    ex_time_so = 3
    N = int(ex_time_so/0.01)
    traj_back_2_so = np.zeros((N,13))

    # Compute the transformation matrix from grasping position back to standoff position
    Tse_so = np.matmul(Tsc_in, Tce_so)

    # Calculate cartesian trajectory from the grasping position to the standoff position
    smol_traj_back_2_so = CartesianTrajectory(Tse_g, Tse_so, ex_time_so, N, 5)

    # Create entries for CSV and add them to matrix
    for n in range(N):
        # Get rotation parts
        entry = []
        for k in range(3):
            #print(smol_traj[n][k][:3])
            for i in range(3):
                entry.append(smol_traj_back_2_so[n][k][i])
        # Add points
        entry.append(smol_traj_back_2_so[n][0][3])
        entry.append(smol_traj_back_2_so[n][1][3])
        entry.append(smol_traj_back_2_so[n][2][3])
        entry.append(0) # Gripper
        traj_back_2_so[n] = entry
        traj_back_2_so[n][-1] = 1 # Close gripper
        config_want.append(smol_traj_back_2_so[n])

    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_back_2_so))
    
    #Move to final stand off
    current_Tse = smol_traj_back_2_so[-1]
    ex_time_g_so = 5
    N = int(ex_time_so/0.01)
    traj_2_goal_so = np.zeros((N,13))

    # Compute the transformation matrix from standoff of initial cube pose to final standoff
    Tse_goal_so = np.matmul(Tsc_f, Tce_so)

    # Calculate cartesian trajectory from fstandoff of initial cube pose to final standoff
    smol_traj_2_g_so = ScrewTrajectory(current_Tse, Tse_goal_so, ex_time_g_so, N, 5)

    # Create entries for CSV and add them to matrix
    for n in range(N):
        # Get rotation parts
        entry = []
        for k in range(3):
            for i in range(3):
                entry.append(smol_traj_2_g_so[n][k][i])
        # Add points
        entry.append(smol_traj_2_g_so[n][0][3])
        entry.append(smol_traj_2_g_so[n][1][3])
        entry.append(smol_traj_2_g_so[n][2][3])
        entry.append(1) # Gripper closed
        traj_2_goal_so[n] = entry
        config_want.append(smol_traj_2_g_so[n])
    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_2_goal_so))
    
    #Move to final configuration of object
    current_Tse = smol_traj_2_g_so[-1]
    ex_time_2_cube_final = 1
    N = int(ex_time_2_cube_final/0.01)
    traj_2_cube_f = np.zeros((N,13))

    # Compute the transformation matrix from final standoff to release position 
    Tse_g = np.matmul(Tsc_f, Tce_g)

    # Calculate cartesian trajectory from final standoff to release position 
    smol_traj_2_cube_f = CartesianTrajectory(current_Tse, Tse_g, ex_time_2_cube_final, N, 5)

    # Create entries for CSV and add them to matrix
    for n in range(N):
        # Get rotation parts
        entry2 = []
        for k in range(3):
            for i in range(3):
                entry2.append(smol_traj_2_cube_f[n][k][i])
        # Add points
        entry2.append(smol_traj_2_cube_f[n][0][3])
        entry2.append(smol_traj_2_cube_f[n][1][3])
        entry2.append(smol_traj_2_cube_f[n][2][3])
        entry2.append(1)  # Gripper closed
        traj_2_cube_f[n] = entry2
        config_want.append(smol_traj_2_cube_f[n])

    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_2_cube_f))
    
    # Open gripper 
    release = traj_2_cube_f[-1]
    release[-1] = 0
    for i in range(6):
        total_traj = np.vstack((total_traj, release))
        
    #Go back to stand off 
    current_Tse = smol_traj_2_cube_f[-1]
    ex_time_so = 3
    N = int(ex_time_so/0.01)
    traj_back_2_f_so = np.zeros((N,13))

    # Compute the transformation matrix from release position back to standoff
    Tse_so = np.matmul(Tsc_f, Tce_so)

    # Calculate cartesian trajectory from release position back to standoff
    smol_traj_back_2_f_so = CartesianTrajectory(current_Tse, Tse_so, ex_time_so, N, 5)

    # Create entries for CSV and add them to matrix
    for n in range(N):
        # Get rotation parts
        entry = []
        for k in range(3):
            for i in range(3):
                entry.append(smol_traj_back_2_f_so[n][k][i])
        # Add points
        entry.append(smol_traj_back_2_f_so[n][0][3])
        entry.append(smol_traj_back_2_f_so[n][1][3])
        entry.append(smol_traj_back_2_f_so[n][2][3])
        entry.append(0)  # Gripper open
        traj_back_2_f_so[n] = entry
        config_want.append(smol_traj_back_2_f_so[n])
    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_back_2_f_so))

    return total_traj, config_want

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt):
    """Calculate the kinematic task-space feedforward plus feedback control law

    Args:
        x (np.array): current actual end-effector configuration
        X_d (np.array):  current end-effector reference configuration
        Xd_next (np.array):  end-effector reference configuration at the next timestep in the reference trajectory at a time Δt later. 
        Kp (np.array): p PI gain matrix
        Ki (np.array): i PI gain matrix
        dt (float): timestep Δt between reference trajectory configurations

    Returns:
        Commanded end-effector twist in the end-effector frame
    """

    # Calculate twist to command end effector
    Xd_inv = TransInv(Xd)
    Vd_skew = (1/dt) * MatrixLog6(Xd_inv@Xd_next)

    Vd = np.array([Vd_skew[2,1],
                Vd_skew[0,2],
                Vd_skew[1,0],
                Vd_skew[0,3],
                Vd_skew[1,3],
                Vd_skew[2,3]
                ])

    # Calculate error of system 
    Xerr_skew = MatrixLog6(TransInv(X)@Xd)
    
    Xerr = np.array([Xerr_skew[2,1],
            Xerr_skew[0,2],
            Xerr_skew[1,0],
            Xerr_skew[0,3],
            Xerr_skew [1,3],
            Xerr_skew[2,3]
            ])
    ad = Adjoint(TransInv(X)@Xd)
    Vv = (ad@Vd) + (Kp@Xerr) + (Ki@(Xerr*dt)) + Kp@(Xerr*dt)

    return Vv, Xerr

############################################

# Other task cube position
# Tsc_in = np.array([[1, 0, 0, 0.5],
#                    [0, 1, 0, -0.5],
#                    [0, 0, 1, 0.025],
#                    [0, 0, 0, 1]
#                    ])

# Tsc_f = np.array([[0, 1, 0, 2],
#                   [-1, 0, 0, 1],
#                   [0, 0, 1, 0.025],
#                   [0, 0, 0, 1]
#                   ])

# Default cube position 
Tsc_in = np.array([[1, 0, 0, 1],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.025],
                   [0, 0, 0, 1]
                   ])

Tsc_f = np.array([[0, 1, 0, 0],
                  [-1, 0, 0, -1],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]
                  ])

Tse_in = np.array([[0, 0, 1, 0],
                   [0, 1, 0, 0],
                   [-1, 0, 0, 0.5],
                   [0, 0, 0, 1]
                   ])

ang = 3*np.pi / 4
Tce_so = np.array([[np.cos(ang), 0, np.sin(ang), 0],
                   [0, 1, 0, 0],
                   [-np.sin(ang), 0, np.cos(ang), 0.3],
                   [0, 0, 0, 1]
                   ])

Tce_g = np.array([[np.cos(ang), 0, np.sin(ang), 0],
                   [0, 1, 0, 0],
                   [-np.sin(ang), 0, np.cos(ang), 0.01],
                   [0, 0, 0, 1]
                   ])

Blist = np.array([
        [0, 0, 1, 0, 0.033, 0],
        [0, -1, 0, -0.5076, 0, 0],
        [0, -1, 0, -0.3526, 0, 0],
        [0, -1, 0, -0.2176, 0, 0],
        [0, 0, 1, 0, 0, 0]
    ]).T

M0e = np.array([
    [1, 0, 0, 0.033],
    [0, 1, 0, 0],
    [0, 0, 1, 0.6546],
    [0, 0, 0, 1]    
])
Tb0 = np.array([
    [1, 0, 0, 0.1662],
    [0, 1, 0, 0],
    [0, 0, 1, 0.0026],
    [0, 0, 0, 1]
])

l = 0.47 / 2
w = 0.3 / 2
r = 0.0475
tmp_arg = 1 / (l+w)

force = (0.25*r) * np.array([
    [-tmp_arg, tmp_arg, tmp_arg, -tmp_arg],
    [1, 1, 1, 1],
    [-1, 1, -1, 1]
])
force6 = (0.25*r) * np.array([
    [0,0,0,0],
    [0,0,0,0],
    [-tmp_arg, tmp_arg, tmp_arg, -tmp_arg],
    [1, 1, 1, 1],
    [-1, 1, -1, 1],
    [0,0,0,0]
    ])

k = 1 

# Obtain desired trajectory
des_traj, desired_config = TrajectoryGenerator(Tse_in, Tsc_in, Tsc_f, Tce_g, Tce_so, k)

# No gain
# Kp = 0*np.identity(6)
# Ki = 0*np.identity(6)

# Best gain
# Kp = 4*np.identity(6)
# Ki =0.01*np.identity(6)

# Overshoot gain
Kp = 20*np.identity(6)
Ki =6*np.identity(6)

dt = 0.01
matrix_for_csv = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
current_config = [0, -0.3, np.pi/4, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# current_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# current_config = [-0.2, -0.5, 0.4, 0.6, 0.2, 0.2, -0.6, 0, 0, 0, 0, 0]

error_plt = np.zeros((len(desired_config),6))

for i in range(len(desired_config)):

    theta_list = np.array([current_config[3],current_config[4],current_config[5],current_config[6],current_config[7]])

    phi_current = current_config[0]
    x_current = current_config[1]
    y_current = current_config[2]
    Tsb = np.array([
        [np.cos(phi_current), -np.sin(phi_current), 0, x_current],
        [np.sin(phi_current), np.cos(phi_current), 0, y_current],
        [0,          0,           1, 0.0963],
        [0,          0,          0, 1]
    ], dtype=object)

    Tse = Tsb@Tb0@M0e

    # Perform forward kinematics of current position
    X = FKinBody(Tse,Blist, theta_list)

    try:
        # Next desired position
        Xd_next = des_traj[i+1]
        Xd_next = np.array([
            [Xd_next[0], Xd_next[1], Xd_next[2], Xd_next[9]],
            [Xd_next[3], Xd_next[4], Xd_next[5], Xd_next[10]],
            [Xd_next[6], Xd_next[7], Xd_next[8], Xd_next[11]],
            [0, 0, 0, 1]
        ])
    except:
        pass


    Xd = des_traj[i]
    # Store grip value for later
    grip = Xd[12]

    # Desired position
    Xd = np.array([
            [Xd[0], Xd[1], Xd[2], Xd[9]],
            [Xd[3], Xd[4], Xd[5], Xd[10]],
            [Xd[6], Xd[7], Xd[8], Xd[11]],
            [0, 0, 0, 1]
        ])

    # Obtain the twist and error to the desired position from current position
    V_calc, error = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)
    error_plt[i] =error
    T_0e = FKinBody(M0e, Blist, theta_list)

    # Calculate Jacobians
    Je_arm = JacobianBody(Blist, theta_list) 

    J_base = Adjoint(TransInv(T_0e)@TransInv(Tb0))@force6
    Je_tot = np.hstack((J_base,Je_arm))
    Je_inv = np.linalg.pinv(Je_tot)

    # Calculate the control velocities
    u_th_dot = Je_inv@V_calc

    # Obtaine the next state from the current positiion and velocities
    next_state = NextState(current_config, u_th_dot, dt, 10)
    current_config = next_state

    # Append to matrix to create CSV
    matrix_for_csv = np.vstack((matrix_for_csv,  np.append(next_state,grip)))

# Make CSV
np.savetxt("overshoot.csv", matrix_for_csv, delimiter = ",") 
np.savetxt("overshoot_err.csv", error_plt, delimiter = ",") 

print(error.shape)

# Plot everything
fig, axis = plt.subplots(2,3)
axis[0,0].plot(error_plt[:,0])
axis[0,0].set_title('Roll Error')
axis[0,1].plot(error_plt[:,1])
axis[0,1].set_title('Pitch Error')
axis[0,2].plot(error_plt[:,2])
axis[0,2].set_title('Yaw Error')

axis[1,0].plot(error_plt[:,3])
axis[1,0].set_title('X Error')
axis[1,1].plot(error_plt[:,4])
axis[1,1].set_title('Y Error')
axis[1,2].plot(error_plt[:,5])
axis[1,2].set_title('Z Error')

axis[0,0].set(xlabel="Time (ms)")
axis[0,1].set(xlabel="Time (ms)")
axis[0,2].set(xlabel="Time (ms)")
axis[1,0].set(xlabel="Time (ms)")
axis[1,1].set(xlabel="Time (ms)")
axis[1,2].set(xlabel="Time (ms)")

axis[0,0].set(ylabel="Error (rads)")
axis[0,1].set(ylabel="Error (rads)")
axis[0,2].set(ylabel="Error (rads)")
axis[1,0].set(ylabel="Error (meters)")
axis[1,1].set(ylabel="Error (meters)")
axis[1,2].set(ylabel="Error (meters)")

fig.tight_layout()
plt.show()
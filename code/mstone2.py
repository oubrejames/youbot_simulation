from modern_robotics import *
import numpy as np

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


    ###################### Go to grasp position ######################
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

    # Update total trajectory
    total_traj = np.vstack((traj_2_so, traj_2_cube_in))
    
    ###################### Grasp ######################
    grasp = traj_2_cube_in[-1]
    grasp[-1] = 1
    for i in range(6):
        total_traj = np.vstack((total_traj, grasp))

    ###################### Go back to stand off ######################
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

    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_back_2_so))
    
    ###################### Move to final stand off ######################
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

    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_2_goal_so))
    
    ###################### Move to final configuration of object  ######################
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

    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_2_cube_f))
    
    ###################### Open gripper ######################
    release = traj_2_cube_f[-1]
    release[-1] = 0
    for i in range(6):
        total_traj = np.vstack((total_traj, release))
        
    ###################### Go back to stand off ######################
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

    # Update total trajectory
    total_traj = np.vstack((total_traj, traj_back_2_f_so))

    return total_traj

########################## Milestone 2 ##########################

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

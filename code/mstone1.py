from modern_robotics import *

def NextState(current_config, velocities, dt, max_vel):
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

    old_arm_joint_angles = np.array(old_arm_joint_angles)

    joint_speeds = np.array(joint_speeds)
    old_wheel_angles = np.array(old_wheel_angles)
    wheel_speeds = np.array(wheel_speeds)

    new_arm_joint_angles = old_arm_joint_angles + joint_speeds*dt

    new_wheel_angles = old_wheel_angles + wheel_speeds*dt

    l = 0.47 / 2
    w = 0.3 / 2
    r = 0.0475
    tmp_arg = 1 / (l+w)
    F = (0.25*r) * np.array([
        [-tmp_arg, tmp_arg, tmp_arg, -tmp_arg],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])
    Vb = F@wheel_speeds.T
    wbz = Vb[0]
    if wbz == 0:
        print("Ass")
        d_qb = Vb
    else:
        print("hole")
        d_qb = np.array([
            Vb[0],
            Vb[1]*np.sin(Vb[0])+Vb[2]*(np.cos(Vb[0])-1)/Vb[0],
            Vb[2]*np.sin(Vb[0])+Vb[1]*(1-np.cos(Vb[0]))/Vb[0]
        ])
    
    phi_k = old_chassis_config[0]

    d_q = np.array([
        [1, 0, 0],
        [0, np.cos(phi_k), -np.sin(phi_k)],
        [0, np.sin(phi_k), np.cos(phi_k)]
    ])@d_qb

    new_chassis_config = old_chassis_config + d_q*dt
    print("old_chassis_config", old_chassis_config)
    print("d_qb", d_qb)
    print("old_chassis_config + d_q*dt", old_chassis_config + d_q*dt)


    return new_arm_joint_angles, new_wheel_angles, new_chassis_config

# NextState([1,2,3,4,5,6,7,8,9,10,11,12],[0,1,2,3,4,5,6,7,8],0.01,12)


current_config = [0,0,0,0,0,0,0,0,0,0,0,0]
u = [-10, 10, 10, -10, 0, 0, 0, 0, 0]
dt = 0.01
matrix_for_csv = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0]
for i in range(100):
    arm, wheel, chassis = NextState(current_config, u, dt, 12)
    print("arm size", arm.shape)
    print("wheel size", wheel.shape)
    print("chassis size", chassis.shape)

    current_config = np.hstack((chassis, arm, wheel))
    config4Mat = np.hstack((current_config, 0))
    matrix_for_csv = np.vstack((matrix_for_csv, config4Mat))

np.savetxt("test.csv", matrix_for_csv, delimiter = ",") 

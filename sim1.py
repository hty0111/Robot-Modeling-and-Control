#python
from IK.IKSolver import IKSolver
import numpy as np
from pprint import pprint

####################################
### You Can Write Your Code Here ###
####################################

def sysCall_init():
    # initialization the simulation
    doSomeInit()    # must have    
    
    #------------------------------------------------------------------------
    # using the codes, you can obtain the poses and positions of four blocks
    pointHandles = []
    for i in range(2):
        pointHandles.append(sim.getObject('::/Platform1/Cuboid' + str(i+1) + '/SuckPoint'))
    for i in range(2):
        pointHandles.append(sim.getObject('::/Platform1/Prism' + str(i+1) + '/SuckPoint'))
    # get the pose of Cuboid/SuckPoint
    #for i in range(4):
    #    print(sim.getObjectPose(pointHandles[i], -1))
    #-------------------------------------------------------------------------
        
        
    #-------------------------------------------------------------------------
    # following codes show how to call the build-in inverse kinematics solver
    # you may call the codes, or write your own IK solver
    # before you use the codes, you need to convert the above quaternions to X-Y'-Z' Euler angels 
    # you may write your own codes to do the conversion, or you can use other tools (e.g. matlab)
    iks = IKSolver()
    # return the joint angle vector q which belongs to [-PI, PI]
    # Position and orientation of the end-effector are defined by [x, y, z, rx, ry, rz]
    # x,y,z are in meters; rx,ry,rz are X-Y'-Z'Euler angles in radian
    
    #-----------------------------------------------------------

    """
    'p' means start point, 'q' means end point
    'org' means placed point, 'lift' means lifted point
    'rpy' means cartesian space, 'joint' means joint space
    """

    global p1_org_joint, p1_lift_joint, p2_org_joint, p2_lift_joint, p3_org_joint, p3_lift_joint, p4_org_joint, p4_lift_joint
    global q1_org_joint, q1_lift_joint, q2_org_joint, q2_lift_joint, q3_org_joint, q3_lift_joint, q4_org_joint, q4_lift_joint
    global origin, pond_points_joint, pond_above_joint
    origin = np.zeros(6)
    lift_height = 0.052

    p1_org_rpy = np.array([0.4, 0.12, 0.1501, np.pi, 0, 82.0/180*np.pi])    # cartesian coordinate
    p1_org_joint = cartesian_to_joint(iks.solve(p1_org_rpy))                # joint coordinate
    p1_lift_rpy = np.array([p1_org_rpy[i] if i != 2 else p1_org_rpy[i] + lift_height for i in range(6)])    # lift certain height
    p1_lift_joint = cartesian_to_joint(iks.solve(p1_lift_rpy))

    p2_org_rpy = np.array([0.40, 0.04, 0.1251, 151.1/180*np.pi, 36.13/180*np.pi, -46.89/180*np.pi])
    p2_org_joint = cartesian_to_joint(iks.solve(p2_org_rpy))
    p2_lift_rpy = np.array([p2_org_rpy[i] if i != 2 else p2_org_rpy[i] + lift_height for i in range(6)])
    p2_lift_joint = cartesian_to_joint(iks.solve(p2_lift_rpy))

    p3_org_rpy = np.array([0.40, -0.04, 0.1251, -148.95/180*np.pi, 34.37/180*np.pi, -136.86/180*np.pi])
    p3_org_joint = cartesian_to_joint(iks.solve(p3_org_rpy))
    p3_lift_rpy = np.array([p3_org_rpy[i] if i != 2 else p3_org_rpy[i] + lift_height for i in range(6)])
    p3_lift_joint = cartesian_to_joint(iks.solve(p3_lift_rpy))

    p4_org_rpy = np.array([0.4, -0.12, 0.1501, np.pi, 0, 121.1/180*np.pi])
    p4_org_joint = cartesian_to_joint(iks.solve(p4_org_rpy))
    p4_lift_rpy = np.array([p4_org_rpy[i] if i != 2 else p4_org_rpy[i] + lift_height for i in range(6)])
    p4_lift_joint = cartesian_to_joint(iks.solve(p4_lift_rpy))

    pond_start_rpy = np.array([0.15, 0.35, 0.2, np.pi, 0, np.pi/2])
    pond_end_rpy = np.array([-0.15, 0.35, 0.2, np.pi, 0, np.pi/2])
    # linear interpolation from start to end
    pond_points_rpy = np.array([[(pond_start_rpy[0] + i*(pond_end_rpy[0]-pond_start_rpy[0])/100)] + list(pond_start_rpy[1:])
                                for i in range(100)])
    pond_points_joint = np.array([cartesian_to_joint(iks.solve(pond_points_rpy[i])) for i in range(100)])

    # mid point above the pond when returning from target to origin 
    pond_above_rpy = np.array([0, 0.35, 0.3, np.pi, 0, np.pi])
    pond_above_joint = cartesian_to_joint(iks.solve(pond_above_rpy))

    q1_org_rpy = np.array([-0.351, 0, 0.201, np.pi, 0, np.pi/2])
    q1_org_joint = cartesian_to_joint(iks.solve(q1_org_rpy))
    q1_lift_rpy = np.array([q1_org_rpy[i] if i != 2 else q1_org_rpy[i] + lift_height for i in range(6)])
    q1_lift_joint = cartesian_to_joint(iks.solve(q1_lift_rpy))

    q2_org_rpy = np.array([-0.35, 0.052, 0.175, 135/180*np.pi, 0, 0])
    q2_org_joint = iks.solve(q2_org_rpy)[:, 2]
    q2_lift_rpy = np.array([q2_org_rpy[i] if i != 2 else q2_org_rpy[i] + lift_height for i in range(6)])
    q2_lift_joint = iks.solve(q2_lift_rpy)[:, 2]

    q3_org_rpy = np.array([-0.35, -0.051, 0.175, -135/180*np.pi, 0, 0])
    q3_org_joint = np.array([194.18732687, 52.88551453, 68.22740879, -17.34159865, -43.27806873, -70.32710395])/180*np.pi
    q3_lift_rpy = np.array([q3_org_rpy[i] if i != 2 else q3_org_rpy[i] + lift_height for i in range(6)])
    q3_lift_joint = np.array([194.67868547, 43.15794751, 58.23478484, 2.82660785, -43.15923557, -69.67286484])/180*np.pi

    q4_org_rpy = np.array([-0.351, 0, 0.252, np.pi, 0, np.pi/2])
    q4_org_joint = cartesian_to_joint(iks.solve(q4_org_rpy))
    q4_lift_rpy = np.array([q4_org_rpy[i] if i != 2 else q4_org_rpy[i] + lift_height for i in range(6)])
    q4_lift_joint = cartesian_to_joint(iks.solve(q4_lift_rpy))


def sysCall_actuation():
    # put your actuation code in this function   
    
    # get absolute time, t
    t = sim.getSimulationTime()
    
    # if t>20s, pause the simulation
    if t > 136:
        sim.pauseSimulation()    
    # robot takes 5s to move from q0 to q1. 
    # the vaccum gripper takes effect after wating 0.2s. 
    if t < 5:
        # call the trajactory planning funcion
        # return the joint angles at time t
        q = quintic_spline(origin, p1_org_joint, t, 5)
        state = False # vacumm gripper is off
        
    #########
    # block 1
    #########
    elif t < 8:     # lift a block  
        q = quintic_spline(p1_org_joint, p1_lift_joint, t-5, 3)
        state = True
    elif t < 13:    # move to start of pond
        q = quintic_spline(p1_lift_joint, pond_points_joint[0], t-8, 5)
        state = True    
    elif t < 18:    # pass through pond
        num = int((t - 13) / 0.05)
        if num >= 99: num = 99
        q = pond_points_joint[num]
        state = True
    elif t < 23:    # move to target
        q = quintic_spline(pond_points_joint[-1], q1_lift_joint, t-18, 5)
        state = True
    elif t < 26:    # lower the block
        q = quintic_spline(q1_lift_joint, q1_org_joint, t-23, 3)
        state = True
    elif t < 31:    # return
        q = quintic_spline(q1_org_joint, pond_above_joint, t-26, 5)
        state = False
    elif t < 36:
        q = quintic_spline(pond_above_joint, p2_lift_joint, t-31, 5)
        state = False

    #########
    # block 2
    #########
    elif t < 39:
        q = quintic_spline(p2_lift_joint, p2_org_joint, t-36, 3)
        state = False
    elif t < 42:
        q = quintic_spline(p2_org_joint, p2_lift_joint, t-39, 3)
        state = True
    elif t < 47:
        q = quintic_spline(p2_lift_joint, pond_points_joint[0], t-42, 5)
        state = True
    elif t < 52:
        num = int((t - 47) / 0.05)
        if num >= 99: num = 98
        q = pond_points_joint[num]
        state = True
    elif t < 57:
        q = quintic_spline(pond_points_joint[-1], q2_lift_joint, t-52, 5)
        state = True
    elif t < 60:
        q = quintic_spline(q2_lift_joint, q2_org_joint, t-57, 3)
        state = True
    elif t < 65:
        q = quintic_spline(q2_org_joint, pond_above_joint, t-60, 5)
        state = False
    elif t < 70:
        q = quintic_spline(pond_above_joint, p3_lift_joint, t-65, 5)
        state = False

    #########
    # block 3
    #########
    elif t < 73:
        q = quintic_spline(p3_lift_joint, p3_org_joint, t-70, 3)
        state = False
    elif t < 76:
        q = quintic_spline(p3_org_joint, p3_lift_joint, t-73, 3)
        state = True
    elif t < 81:
        q = quintic_spline(p3_lift_joint, pond_points_joint[0], t-76, 5)
        state = True
    elif t < 86:
        num = int((t - 81) / 0.05)
        if num >= 99: num = 98
        q = pond_points_joint[num]
        state = True
    elif t < 91:
        q = quintic_spline(pond_points_joint[-1], q3_lift_joint, t-86, 5)
        state = True
    elif t < 94:
        q = quintic_spline(q3_lift_joint, q3_org_joint, t-91, 3)
        state = True
    elif t < 97:
        q = quintic_spline(q3_org_joint, q3_lift_joint, t-94, 3)
        state = False
    elif t < 102:
        q = quintic_spline(q3_lift_joint, pond_above_joint, t-97, 5)
        state = False
    elif t < 107:
        q = quintic_spline(pond_above_joint, p4_lift_joint, t-102, 5)
        state = False

    #########
    # block 4
    #########
    elif t < 110:
        q = quintic_spline(p4_lift_joint, p4_org_joint, t-107, 3)
        state = False
    elif t < 113:
        q = quintic_spline(p4_org_joint, p4_lift_joint, t-110, 3)
        state = True
    elif t < 118:
        q = quintic_spline(p4_lift_joint, pond_points_joint[0], t-113, 5)
        state = True
    elif t < 123:
        num = int((t - 118) / 0.05)
        if num >= 99: num = 98
        q = pond_points_joint[num]
        state = True
    elif t < 128:
        q = quintic_spline(pond_points_joint[-1], q4_lift_joint, t-123, 5)
        state = True
    elif t < 131:
        q = quintic_spline(q4_lift_joint, q4_org_joint, t-128, 3)
        state = True
    else:
        q = quintic_spline(q4_org_joint, origin, t-131, 5)
        state = False
    
    # check if the joint velocities beyond limitations.
    # if they do, the simulation will stops and report errors.
    runState = move(q, state)

    if not runState:
        sim.pauseSimulation()
        
    """
    The following codes shows a procedure of trajectory planning using the 5th-order polynomial
    You may write your own code to replace this function, e.g. trapezoidal velocity planning
    """
def quintic_spline(start, end, t, time, start_v=0, end_v=0, start_a=0, end_a=0):
    """ Quintic Polynomial: x = k5*t^5 + k4*t^4 + k3*t^3 + k2*t^2 + k1*t + k0
    :param start: Start point
    :param end: End point
    :param t: Current time
    :param time: Expected time spent
    :return: The value of the current time in this trajectory planning
    """
    if t < time:
        tMatrix = np.matrix([
        [         0,           0,             0,          0,        0,   1],
        [   time**5,     time**4,       time**3,    time**2,     time,   1],
        [         0,           0,             0,          0,        1,   0],
        [ 5*time**4,   4*time**3,     3*time**2,     2*time,        1,   0],
        [         0,           0,             0,          2,        0,   0],
        [20*time**3,  12*time**2,        6*time,          2,        0,   0]])
        
        xArray = []
        for i in range(len(start)):
            xArray.append([start[i], end[i], start_v, end_v, start_a, end_a])
        xMatrix = np.matrix(xArray).T
        
        kMatrix = tMatrix.I * xMatrix
        
        timeVector = np.matrix([t**5, t**4, t**3, t**2, t, 1]).T
        x = (kMatrix.T * timeVector).T.A[0]
        
    else:
        x = end
    
    return x


####################################################
### You Don't Have to Change the following Codes ###
####################################################

def doSomeInit():
    global Joint_limits, Vel_limits, Acc_limits
    Joint_limits = np.array([[-200, -90, -120, -150, -150, -180],
                            [200, 90, 120, 150, 150, 180]]).transpose()/180*np.pi
    Vel_limits = np.array([100, 100, 100, 100, 100, 100])/180*np.pi
    Acc_limits = np.array([500, 500, 500, 500, 500, 500])/180*np.pi
    
    global lastPos, lastVel, sensorVel
    lastPos = np.zeros(6)
    lastVel = np.zeros(6)
    sensorVel = np.zeros(6)
    
    global robotHandle, suctionHandle, jointHandles
    robotHandle = sim.getObject('.')
    suctionHandle = sim.getObject('./SuctionCup')
    jointHandles = []
    for i in range(6):
        jointHandles.append(sim.getObject('./Joint' + str(i+1)))
    sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    sim.writeCustomDataBlock(robotHandle, 'error', '0')
    
    global dataPos, dataVel, dataAcc, graphPos, graphVel, graphAcc
    dataPos = []
    dataVel = []
    dataAcc = []
    graphPos = sim.getObject('./DataPos')
    graphVel = sim.getObject('./DataVel')
    graphAcc = sim.getObject('./DataAcc')
    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
    for i in range(6):
        dataPos.append(sim.addGraphStream(graphPos, 'Joint'+str(i+1), 'deg', 0, color[i]))
        dataVel.append(sim.addGraphStream(graphVel, 'Joint'+str(i+1), 'deg/s', 0, color[i]))
        dataAcc.append(sim.addGraphStream(graphAcc, 'Joint'+str(i+1), 'deg/s2', 0, color[i]))


def sysCall_sensing():
    # put your sensing code here
    if sim.readCustomDataBlock(robotHandle,'error') == '1':
        return
    global sensorVel
    for i in range(6):
        pos = sim.getJointPosition(jointHandles[i])
        if i == 0:
            if pos < -160/180*np.pi:
                pos += 2*np.pi
        vel = sim.getJointVelocity(jointHandles[i])
        acc = (vel - sensorVel[i])/sim.getSimulationTimeStep()
        if pos < Joint_limits[i, 0] or pos > Joint_limits[i, 1]:
            print("Error: Joint" + str(i+1) + " Position Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return
        
        if abs(vel) > Vel_limits[i]:
            print("Error: Joint" + str(i+1) + " Velocity Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return
        
        if abs(acc) > Acc_limits[i]:
            print("Error: Joint" + str(i+1) + " Acceleration Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return
        
        sim.setGraphStreamValue(graphPos,dataPos[i], pos*180/np.pi)
        sim.setGraphStreamValue(graphVel,dataVel[i], vel*180/np.pi)
        sim.setGraphStreamValue(graphAcc,dataAcc[i], acc*180/np.pi)
        sensorVel[i] = vel


def sysCall_cleanup():
    # do some clean-up here
    sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    sim.writeCustomDataBlock(robotHandle, 'error', '0')


def move(q, state):
    if sim.readCustomDataBlock(robotHandle,'error') == '1':
        return
    global lastPos, lastVel
    for i in range(6):
        if q[i] < Joint_limits[i, 0] or q[i] > Joint_limits[i, 1]:
            print("x(): Joint" + str(i+1) + " Position Out of Range!")
            return False
        if abs(q[i] - lastPos[i])/sim.getSimulationTimeStep() > Vel_limits[i]:
            print("move(): Joint" + str(i+1) + " Velocity Out of Range!")
            return False
        if abs(lastVel[i] - (q[i] - lastPos[i]))/sim.getSimulationTimeStep() > Acc_limits[i]:
            print("move(): Joint" + str(i+1) + " Acceleration Out of Range!")
            return False
            
    lastPos = q
    lastVel = q - lastPos
    
    for i in range(6):
        sim.setJointTargetPosition(jointHandles[i], q[i])
        
    if state:
        sim.writeCustomDataBlock(suctionHandle, 'activity', 'on')
    else:
        sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    
    return True
    

def cartesian_to_joint(possible_angles):
    """
    :param possible_angles: 6*n matrix
    :return: 1*6 matrix in joint space
    """
    for i in range(possible_angles.shape[1]):
        pass_flag = 0
        for j in range(6):
            angle = possible_angles[j][i]
            if Joint_limits[j][0] + 5/180*np.pi < angle < Joint_limits[j][1] - 5/180*np.pi:
                pass
            else:
                pass_flag = 1
                break
        if not pass_flag:
            return possible_angles[:, i]
    print(f"No possible inverse solutions!")
    exit(1)


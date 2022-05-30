import numpy as np
import sim
import time
import argparse
from matplotlib import pyplot as plt

def getArgs():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--ori_angles",     type=float,             help="Original joint angles",   nargs="+",
                        default=[0, 0, 0, 0, 0, 0])
    parser.add_argument("--target_angles",  type=float,             help="Target joint angles",     nargs="+",
                        default=[-42.15, 63.17, 60.34, -50.16, 4.29, -0.73])
    parser.add_argument("--total_t",        type=float,             help="Total time of motion",
                        default=2.0)
    parser.add_argument("--delta_t",        type=float,             help="Control interval",
                        default=0.02)
    parser.add_argument("--acc_scale",      type=float,             help="Times of acceleration to minimal",
                        default=1.2)
    parser.add_argument("--visualize",      action="store_true",    help="Visualize trajectory")
    args = parser.parse_args()
    assert len(args.ori_angles) == 6, f"6 original joint angles needed, {len(args.ori_angles)} are given."
    assert len(args.target_angles) == 6, f"6 target joint angles needed, {len(args.target_angles)} are given."

    return args.delta_t, args.total_t, args.ori_angles, args.target_angles, args.acc_scale, args.visualize


def initArm():
    # connect to server
    sim.simxFinish(-1)
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    assert client_id != -1, f'Failed to connect to server.'
    print(f'Connected to server')

    # initialize 6 joints
    ret_value, joint_handle = [], []
    for i in range(6):
        joint_name = "Revolute_joint" + str(i + 1)
        ret, joint = sim.simxGetObjectHandle(client_id, joint_name, sim.simx_opmode_blocking)
        ret_value.append(ret)
        joint_handle.append(joint)
    # initialize 'tip'
    ret_dummy, tip = sim.simxGetObjectHandle(client_id, 'tip', sim.simx_opmode_blocking)
    ret_value.append(ret_dummy)
    joint_handle.append(tip)

    assert all(ret_value) == 0, f'Failed to connect to all joints'
    print(f'Connected to joints, handles are: {joint_handle}')

    return client_id, joint_handle


def calcTrajectory(ori_angles, target_angles, total_t, delta_t, acc_scale, visualize):
    # calculate acceleration and transient time
    acc = np.zeros(6)
    for i in range(6):
        acc[i] = 4 * (target_angles[i] - ori_angles[i]) / (total_t ** 2) * acc_scale
    transient_t = total_t / 2 - np.sqrt(
        acc[0] ** 2 * total_t ** 2 - 4 * acc[0] * (target_angles[0] - ori_angles[0])) / abs(2 * acc[0])

    # calculate trajectories
    x, y = [], []
    for i in range(6):
        vel = acc[i] * transient_t
        assert abs(vel) < 60, f'Velocity must be lower than 60 degree/s.'
        # first parabola
        x1 = np.arange(0, transient_t, delta_t)
        y1 = np.array(0.5 * acc[i] * x1 ** 2) + [ori_angles[i]] * x1.shape[0]
        # straight line
        x2 = np.arange(transient_t + delta_t, total_t - transient_t, delta_t)
        y2 = np.array(vel * (x2 - transient_t)) + [y1[-1]] * x2.shape[0]
        # last parabola
        x3 = np.arange(total_t - transient_t + delta_t, total_t + delta_t, delta_t)
        y3 = np.array(vel * (transient_t - total_t + x3) - 0.5 * acc[i] * (transient_t - total_t + x3) ** 2) \
             + [y2[-1]] * x3.shape[0]
        # splice three segments
        x.append(np.hstack((x1, x2, x3)))
        y.append(np.hstack((y1, y2, y3)))

    # draw trajectories
    if visualize:
        fig = plt.figure(figsize=(12, 6))
        fig.suptitle('Trajectories')
        ax = []
        for i in range(6):
            ax.append(fig.add_subplot(2, 3, i + 1))
            ax[i].set_title(f'joint{i + 1}')
            ax[i].set_xlabel('t(s)', fontsize=10, family='Arial')
            ax[i].set_ylabel('theta(degree)', fontsize=10, family='Arial')
            ax[i].plot(x[i], y[i])
            ax[i].grid()
        fig.tight_layout()
        plt.show()

    return y


def main():
    # get arguments from CLI or default
    delta_t, total_t, ori_angles, target_angles, acc_scale, visualize = getArgs()
    # initialize
    client_id, joint_handle = initArm()
    # calculate trajectory
    trajectory = calcTrajectory(ori_angles, target_angles, total_t, delta_t, acc_scale, visualize)

    # loop
    current_t, loop_times = 0, 0
    print(f'Control started.')
    while True:
        for i in range(6):
            angle = trajectory[i][loop_times] * np.pi / 180  # degree to radius
            sim.simxSetJointPosition(client_id, joint_handle[i], angle, sim.simx_opmode_streaming)

        current_t += delta_t
        loop_times += 1
        if current_t > total_t:
            print(f'Control finished.')
            break
        time.sleep(delta_t)

if __name__ == '__main__':
    main()


from ntpath import join
import re
import numpy as np
import sim
import math
import time

# 仿真控制周期（秒）
dt = 0.02

# 用于连接
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

joint_angle = np.array(([np.pi/6, 0, np.pi/6, 0, np.pi/3, 0],
                        [np.pi/6, np.pi/6, np.pi/3, 0, np.pi/3, np.pi/6],
                        [np.pi/2, 0, np.pi/2, np.pi/3, np.pi/3, np.pi/6],
                        [-np.pi/6, -np.pi/6, -np.pi/3, 0, np.pi/12, np.pi/2],
                        [np.pi/12, np.pi/12, np.pi/12, np.pi/12, np.pi/12, np.pi/12]))


def main():
    # 初始化
    global tip
    if clientID != -1:
        print ('Connected to remote API server')
        # 初始化机械臂关节
        retValue = []
        jointHandle = []
        for i in range(6):
            jointName = "Revolute_joint" + str(i+1)
            ret, joint = sim.simxGetObjectHandle(clientID, jointName, sim.simx_opmode_blocking)
            retValue.append(ret)
            jointHandle.append(joint)
        # 初始化末端tip点，该点代表了机械臂末端敲铃锤的末端位置
            retDummy, tip = sim.simxGetObjectHandle(clientID, 'tip', sim.simx_opmode_blocking)
        # 初始化时间
        loop_time = 2
        interval_time = 1
        current_angle = np.zeros(6)
        last_angle = np.zeros(6)
        # 开始控制周期
        for loop in range(5):
            t = 0
            if loop == 0:
                average_vel = joint_angle[loop] / loop_time  # 平均速度
            else:
                average_vel = (joint_angle[loop] - joint_angle[loop - 1]) / loop_time
            print(f"average vel: {average_vel}")
            while True:
                # 算法写在这里
                # 将仿真机械臂的关节值控制为代码期望的关节值
                for i in range(6):
                    if loop == 0:
                        current_angle[i] = average_vel[i] * t
                    else:
                        current_angle[i] = joint_angle[loop-1][i] + average_vel[i] * t  # 计算当前时刻的关节角度
                    # if loop == 1 and i == 1:
                    #     print(f'last angle:{last_angle[i]}, current angle: {current_angle[i]}, t:{t}')
                    sim.simxSetJointPosition(clientID, jointHandle[i], current_angle[i], sim.simx_opmode_streaming)
                # 读取当前末端tip点的位置(基于世界坐标系原点的坐标，单位m)
                tipPosition = sim.simxGetObjectPosition(clientID, tip, -1, sim.simx_opmode_streaming)
                # 读取当前末端tip点的姿态（欧拉角x-y'-z'内旋，返回值顺序为alpha,beta,gamma，单位rad）
                tipOrient = sim.simxGetObjectOrientation(clientID, tip, -1, sim.simx_opmode_streaming)
                # print(tipPosition)
                # print(tipOrient)

                t = t + dt
                time.sleep(dt)

                # 循环终止条件
                if t > loop_time:
                    print(f'Loop {loop+1} Finished')
                    print(f'position: {tipPosition}')
                    print(f'orientation: {tipOrient}')
                    time.sleep(interval_time)
                    print("---------------")
                    break
    else:
        print('Failed connecting to remote API server')


if __name__ == '__main__':
    main()

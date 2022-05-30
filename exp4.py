from Robot import Robot
import numpy as np
import time
import math

# ------------------  写在前面  ----------------- #
# poly_seg 是个套娃列表，保存了每个关节角分段的路径：[时间，[多项式系数]]
# 其中，多项式系数以列表给出，[k0, k1, k2, k3...]为从常数项开始，幂增的系数
# 如，关节2的第3段轨迹的时间为：poly_seg[1][2][0]
# 如，关节1的第1段轨迹的多项式系数为：poly_seg[0][0][1]
# --------------------------------------------- #


def linear_2p(t1, p0, p1):
    """
    初始时间为0，始末速度为0，线性速度规划
    :param t1: 终止时间
    :param p0: 初始位置
    :param p1: 终止位置
    :return: 套娃seg
    """
    k = np.array([p0, (p1-p0)/t1])
    return np.array([t1, k])


def cubic_2p(t1, p0, p1, t0=0, v0=0.0, v1=0.0):
    """
    以三次多项式k0+k1t+k2t2+k3t3规划两点轨迹
    :param t0: 起始时间
    :param t1: 终止时间
    :param p0: 初始位置
    :param p1: 终止位置
    :param v0: 起始速度
    :param v1: 中止速度
    :return: 套娃，第一个元素1x1，表示截止时间，第二个数组为多项式系数1xn，从常数项从小到大
    """
    b = np.array([p0, p1, v0, v1])
    A = np.array([[1, t0, t0**2, t0**3], [1, t1, t1**2, t1**3], [0, 1, 2*t0, 3*t0**2], [0, 1, 2*t1, 3*t1**2]])
    k = np.linalg.solve(A, b)
    # t = np.array([t0, t1])
    return np.array([t1, k])


def max_acc_by_vol_2p(p0, p1, t_list, v0=0.0, v1=0.0, v_max=60, a_max=400, mid=-0.08845569/math.pi*180):
    """
    以梯形速度规划轨迹，加速度max=400，速度max=60
    根据初位置和末位置判断往哪边走
    懒得判断输入合理性了，默认存在匀速段！！
    :param t_list: 作为可变参数传入，元素1为全程时间，元素2为到指定中间点的时间
    :param mid: 指定一个中间位置，返回运行到此的时间
    :param a_max: 预设最大速度60deg/s
    :param v_max: 预设最大加速度400°/s^2
    :param p0: 起始位置
    :param p1: 结束位置
    :param v0: 开始速度
    :param v1: 结束速度
    :return: 套娃，第一个元素1x1，表示截止时间，第二个数组为多项式系数1xn，从常数项从小到大
    """
    vm = np.sign(p1-p0)*v_max  # 判断是正着走还是倒着走,正着走vm是正的
    ta = abs(vm-v0)/a_max  # 加速段时间
    tc = abs(v1-vm)/a_max  # 减速段时间
    pa = v0*ta+np.sign(p1-p0)*0.5*a_max*ta**2  # 加速段能跑多远
    ka = np.array([p0, v0, np.sign(p1-p0)*0.5*a_max])  # 加速段表达式系数
    pc = -(v1**2-vm**2)/2/np.sign(p1-p0)/a_max  # 减速段能跑多远
    tb = (p1-p0-pa-pc)/vm  # 匀速段时间
    if tb <= 0:
        print("梯形速度规划无匀速段请检查！")
        exit()
    pb = tb*vm  # 匀速段能跑多远
    kb = np.array([p0+pa, vm])  # 匀速段表达式系数
    kc = np.array([p0+pa+pb, vm, -np.sign(p1-p0)*0.5*a_max])  # 减速段表达式系数
    k = [np.array([ta, ka]), np.array([tb, kb]), np.array([tc, kc])]
    t_list[0] = ta+tb+tc
    print("梯形速度轨迹规划单程时间：", t_list[0])
    if np.sign(p1-p0) > 0:
        if p0+pa >= mid:
            t_list[1] = (-v0+math.sqrt(v0**2+2*np.sign(p1-p0)*a_max*(mid-p0)))/np.sign(p1-p0)/a_max
            print("所求中点在加速段，时间：", t_list[1])
        elif p0+pa+pb >= mid:
            t_list[1] = ta+(mid-p0-pa)/vm
            print("所求中点在匀速段，时间：", t_list[1])
        else:
            t_list[1] = ta+tb+(-vm+math.sqrt(vm**2+2*np.sign(p1-p0)*a_max*(mid-p0-pa-pb)))/np.sign(p1-p0)/a_max
            print("所求中点在减速段，时间：", t_list[1])
    return k


def get_poly_val(t, k):
    """
    给定多项式系数与时间t，返回多项式的值
    :param t: 时间t
    :param k: 多项式系数k，k0为常数项，从小到大
    :return: 多项式的值
    """
    y = 0
    for i in range(len(k)):
        y += k[i]*t**i
    return y


def traj(t, poly_seg):
    """
    给定时间t与多项式段poly_seg，返回对应位置
    :param t: 给定时间
    :param poly_seg:多项式段数组，每一个元素代表一个关节，即每一元素是套娃，第一个元素1x1，表示截止时间，第二个数组为多项式系数1xn，从常数项从小到大
    :return: 对应位置
    """

    T = 0  # 运行周期
    for seg in poly_seg:
        T += seg[0]  # 每一段的终点时间
    while t >= T:
        t -= T
    t_base = 0
    for seg in poly_seg:
        if t-t_base >= seg[0]:
            t_base += seg[0]
            continue
        pos = get_poly_val(t - t_base, seg[1])
        return pos


def hold_pos(pre_seg, t_hold):
    """
    返回一个时间为t_hold的常数，以保持关节角不变
    :param pre_seg: 上一段多项式
    :param t_hold: 需要保持的时间
    :return: 套娃数组，seg形式
    """
    if t_hold < 0:
        print("无法规划pos_hold轨迹！时间为负！")
        exit()
    pos = get_poly_val(pre_seg[0], pre_seg[1])
    k = np.array([pos])
    return np.array([t_hold, k])


def check(t, pos, pos_p, pos_pp, v_max=60.1, a_max=400.1):
    """
    以0.02s为周期使用差商求速度、加速度，与限制对比，并抛出警告
    :param t: 当前时间
    :param pos: 当前位置
    :param pos_p: 上周期位置
    :param pos_pp: 上上周期位置
    :param v_max: 最大速度限制
    :param a_max: 最大加速度限制
    :return: 无
    """
    vol = (pos-pos_p)/0.02
    vol_p = (pos_p-pos_pp)/0.02
    acc = (vol-vol_p)/0.02
    pos_limit = [150, 90, 120, 150, 150, 180]
    for i in range(6):
        # print("t=", t, "时，关节", i, "角度:", pos[i])
        if i == 0:
            continue
            # print("t=", t, "时，关节", i, "角度:", pos[i])
            # print("t=", t, "时，关节", i, "角速度:", vol[i])
            # print("t=", t, "时，关节", i, "角加速度:", acc[i])
        if abs(pos[i]) > pos_limit[i]:
            print("Warning: t=", t, "时，关节", i+1, "角度超限！pos=", pos[i])
        if abs(vol[i]) > v_max:
            print("Warning: t=", t, "时，关节", i+1, "角速度超限！vol=", vol[i])
        if abs(acc[i]) > a_max:
            print("Warning: t=", t, "时，关节", i+1, "角加速度超限！acc=", acc[i])


def test():
    # -----------路径规划------------- #
    t_list1 = [0, 0]
    t_else = [0, 0]
    v_m = 180
    a_m = 1000
    t_init = 0.8
    # # 逆运动学给定的位姿
    # joint_ang1 = np.array([-0.73841174, 0.73410972, 1.65482929, -0.84614813, -0.03075535, 0])/math.pi*180
    # joint_ang2 = np.array([-0.08845569, 0.11414149, 1.54840006, -0.5902079, 0.04097188, 0])/math.pi*180
    # joint_ang3 = np.array([0.73568646, 0.29145575, 1.61017211, -0.2354593, -0.10481398, 0])/math.pi*180

    # 经过实际调整后的位姿
    joint_ang1 = np.array([-0.49841174, 0.73410972, 1.65482929, -0.84614813, 0.03075535, 0])/math.pi*180
    joint_ang2 = np.array([-0.08845569, 0.11414149, 1.54840006, -0.5902079, 0.04097188, 0])/math.pi*180
    joint_ang3 = np.array([0.51568646, 0.29145575, 1.61017211, -0.2354593, 0.10481398, 0])/math.pi*180

    poly_seg = []  # ABC循环的轨迹
    poly_seg_init = []  # 零到A的轨迹

    # ------- 初始化：零位-A点-线性速度 ---------- #
    for i in range(6):
        k1 = linear_2p(t_init, 0.0, joint_ang1[i])
        poly_seg_init.append([k1])

    # ------------- 方法1：关节1梯形速度，其余三次曲线 -------------- #
    k1 = max_acc_by_vol_2p(joint_ang1[0], joint_ang3[0], t_list1, v_max=v_m, a_max=a_m)
    k2 = max_acc_by_vol_2p(joint_ang3[0], joint_ang1[0], t_else, v_max=v_m, a_max=a_m)
    poly_seg.append(k1)
    for k in k2:
        poly_seg[0].append(k)

    # 可以把关节2单独拿出来规划，过B点的速度做适当限制，放置关节2在A-B过程中过分超速
    k1 = cubic_2p(t_list1[1], joint_ang1[1], joint_ang2[1], v0=0, v1=-v_m/4)
    k2 = cubic_2p(t_list1[0] - t_list1[1], joint_ang2[1], joint_ang3[1], v0=-v_m/4, v1=0)
    k3 = cubic_2p(t_list1[0], joint_ang3[1], joint_ang1[1], v0=0, v1=0)
    poly_seg.append([k1, k2, k3])

    # 当然也可以不把关节2拿出来规划，改成range1，6就行
    for i in range(2, 6):
        k1 = cubic_2p(t_list1[1], joint_ang1[i], joint_ang2[i])
        k2 = cubic_2p(t_list1[0] - t_list1[1], joint_ang2[i], joint_ang3[i])
        k3 = cubic_2p(t_list1[0], joint_ang3[i], joint_ang1[i])
        poly_seg.append([k1, k2, k3])

    # 打开设备管理器确定连接的COM口，linux和mac只要选择对应串口就行，需要根据具体的串口进行更改，但是波特率不要改
    r = Robot(com='COM4', baud=250000)
    # 连接到真实机器人
    r.connect()
    # 控制周期（推荐周期，最好不要改）
    T = 0.02
    # 初始化
    t = 0
    pos = np.zeros(6)
    pos_p = np.zeros(6)
    pos_pp = np.zeros(6)

    # 开始控制机械臂运动
    # 使用该函数可以使机械臂回到零位
    r.go_home()

    # 开始控制机器人
    while (1):
        start = time.time()
        # 重新开始一次循环
        if t >= 75:
            print('Control Finished')
            break
        # 通过时间与规划目标关节位置的关系，得到挡墙时刻下期望机械臂关节到达的位置
        if t <= t_init:  # 零位-A
            for i in range(6):
                pos[i] = traj(t, poly_seg_init[i])
        else:  # A-B-C-A循环
            for i in range(6):
                pos[i] = traj(t - t_init, poly_seg[i])

        q = pos.copy()
        check(t, pos, pos_p, pos_pp, v_max=v_m, a_max=a_m)
        pos_pp = pos_p.copy()
        pos_p = pos.copy()

        # 控制机械臂运动，syncMove输入格式为1*6的np.array，单位为度度度度度度度度度度度，代表的含义是当前周期下机械臂关节的位置
        r.syncMove(q)
        # 更新时间
        t = t + T
        print(t)
        # 定时器操作
        end = time.time()
        spend_time = end - start
        if spend_time < T:
            time.sleep(T - spend_time)
        else:
            print("timeout!")


if __name__ == '__main__':
    test()

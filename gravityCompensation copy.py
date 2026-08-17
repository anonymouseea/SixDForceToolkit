from numpy import *
import numpy as np
import math
import json
import os
from Robotic_Arm.rm_robot_interface import *
import time
import pandas as pd
# 实例化RoboticArm类
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

# 创建机械臂连接，打印连接id
handle = arm.rm_create_robot_arm("192.168.1.19", 8080)

# #获取一次力传感器数据
# status, data = arm.rm_get_force_data()
# for k, v in data.items():
#     print(f"{k}: {list(v)}")



# 总结一下整个流程：
# 1. 采集多组力传感器数据，包括力、力矩和末端姿态角（欧拉角）。
# 2. 使用这些数据构造矩阵F和M，求解质心位置和常数k1,k2,k3。
# 3. 使用力数据和姿态角构造矩阵R和f，求解重力加速度g，姿态角U,V以及传感器零点力F_x0,F_y0,F_z0。
# 4. 使用求解得到的参数，对新的力和力矩数据进行补偿，得到接触力和接触力矩。   

# def get_sensor_data():
#     # 这里写实际采集代码
#     # force_data = read_force_from_serial()
#     # torque_data = read_torque_from_serial()
#     # euler_data = read_euler_from_serial()
#     return force_data, torque_data, euler_data


class GravityCompensation:

    M = np.empty((0, 0))
    F = np.empty((0, 0))
    f = np.empty((0, 0))
    R = np.empty((0, 0))

    #末端质心的位置,在力传感器坐标系下
    x = 0
    y = 0
    z = 0
    #三个常数
    k1 = 0
    k2 = 0
    k3 = 0

    #基座的姿态角以及末端的重力
    U = 0
    V = 0
    g = 0

    #传感器零点力和力矩
    F_x0 = 0
    F_y0 = 0
    F_z0 = 0

    M_x0 = 0
    M_y0 = 0
    M_z0 = 0

    # 补偿后的力和力矩
    F_ex = 0
    F_ey = 0
    F_ez = 0

    M_ex = 0
    M_ey = 0
    M_ez = 0

    # 力传感器相对末端法兰的固定绕Z旋转角（弧度），如安装有偏转请在此设置
    sensor_alpha = 0

    #这个函数用于更新力矩数据
    def Update_M(self, torque_data):
        M_x = torque_data[0]
        M_y = torque_data[1]
        M_z = torque_data[2]
        #tanspose代表矩阵转置,现在M是3行1列
        if (any(self.M)):
            M_1 = matrix([M_x, M_y, M_z]).transpose()
            self.M = vstack((self.M, M_1))
        else:
            self.M = matrix([M_x, M_y, M_z]).transpose()

    #此处F用于构造大矩阵，形成Nx6矩阵，用来求解A，其中N为采集的数据组数
    def Update_F(self, force_data):
        F_x = force_data[0]
        F_y = force_data[1]
        F_z = force_data[2]

        if (any(self.F)):
            F_1 = matrix([[0, F_z, -F_y, 1, 0, 0],
                          [-F_z, 0, F_x, 0, 1, 0],
                          [F_y, -F_x, 0, 0, 0, 1]])
            self.F = vstack((self.F, F_1))
        else:
            self.F = matrix([[0, F_z, -F_y, 1, 0, 0],
                             [-F_z, 0, F_x, 0, 1, 0],
                             [F_y, -F_x, 0, 0, 0, 1]])

    #求出质心位置和常数k1,k2,k3
    def Solve_A(self):
        # 使用最小二乘求解，避免显式求逆带来的数值不稳定
        A, *_ = np.linalg.lstsq(self.F, self.M, rcond=None)

        self.x = A[0, 0]
        self.y = A[1, 0]
        self.z = A[2, 0]

        #就是文档里的a1，a2，a3
        self.k1 = A[3, 0]
        self.k2 = A[4, 0]
        self.k3 = A[5, 0]

        print("A=" , A)
        print("x= ", self.x)
        print("y= ", self.y)
        print("z= ", self.z)
        print("k1= ", self.k1)
        print("k2= ", self.k2)
        print("k3= ", self.k3)
        # 拟合残差评估（RMSE）
        M_hat = self.F @ A
        try:
            rmse = math.sqrt(np.mean(np.array(self.M - M_hat)**2))
            print(f"力矩拟合RMSE: {rmse:.4f} (N·m)")
        except Exception:
            pass

    def Refine_A_with_gravity(self):
        """
        在已求得 B=[gx,gy,gz,Fx0,Fy0,Fz0] 后，使用仅由重力产生的力 f_g = R_sb @ g_vec
        重新拟合 r=[x,y,z] 与 常量力矩偏置 t0=[t0x,t0y,t0z]：
            M ≈ t0 + r × f_g
        线性化后可写为 [S(f_g) | I] · [r; t0] = M，其中 S(f_g) 实现 r×f_g 的矩阵形式。

        拟合得到 r 与 t0 后，为兼容现有 Solve_Torque 的实现（其中 M_zero = k - F0×r），
        将 k 设置为 k = t0 + F0 × r。
        """
        if self.R.size == 0 or self.M.size == 0:
            return
        # 提取每组的 R_sb（R 中存的是 [R_sb | I]）与 M
        N = self.R.shape[0] // 3
        if N == 0:
            return
        R_blocks = []
        for i in range(N):
            R_block = self.R[3*i:3*(i+1), 0:3]
            R_blocks.append(R_block)
        g_vec = np.array([[self.gx], [self.gy], [self.gz]])

        # 构造新的大矩阵 G 和观测向量 M_all
        G = None
        M_all = None
        for i, R_sb in enumerate(R_blocks):
            f_g = R_sb @ g_vec  # 3x1
            Fx, Fy, Fz = float(f_g[0, 0]), float(f_g[1, 0]), float(f_g[2, 0])
            # S矩阵（用于 r×f_g）：与 Update_F 中一致的构造
            S = np.matrix([[0,     Fz,   -Fy],
                           [-Fz,  0,     Fx],
                           [Fy,  -Fx,    0 ]])
            # 拼接 [S | I]
            Gi = np.hstack((S, np.eye(3)))
            if G is None:
                G = Gi
            else:
                G = np.vstack((G, Gi))
            # 观测力矩
            Mi = self.M[3*i:3*(i+1), :]
            if M_all is None:
                M_all = Mi
            else:
                M_all = np.vstack((M_all, Mi))

        # 最小二乘求解 [r; t0]
        X, *_ = np.linalg.lstsq(G, M_all, rcond=None)
        rx, ry, rz = X[0, 0], X[1, 0], X[2, 0]
        t0x, t0y, t0z = X[3, 0], X[4, 0], X[5, 0]

        # 更新 r
        self.x, self.y, self.z = rx, ry, rz

        # 计算 F0×r，并据此回填 k = t0 + F0×r，使得现有 Solve_Torque 公式仍然成立
        Fx0, Fy0, Fz0 = self.F_x0, self.F_y0, self.F_z0
        cross_F0_r = np.array([
            Fy0*rz - Fz0*ry,
            Fz0*rx - Fx0*rz,
            Fx0*ry - Fy0*rx
        ])
        self.k1 = t0x + cross_F0_r[0]
        self.k2 = t0y + cross_F0_r[1]
        self.k3 = t0z + cross_F0_r[2]

        # 评估残差
        M_hat = G @ X
        try:
            rmse = math.sqrt(np.mean(np.array(M_all - M_hat)**2))
            print(f"力矩重拟合RMSE: {rmse:.4f} (N·m)")
        except Exception:
            pass
    #此处f用于构造力矩阵，形成Nx3矩阵，用来求解B，其中N为采集的数据组数
    def Update_f(self, force_data):
        F_x = force_data[0]
        F_y = force_data[1]
        F_z = force_data[2]

        if (any(self.f)):
            f_1 = matrix([F_x, F_y, F_z]).transpose()
            self.f = vstack((self.f, f_1))
        else:
            self.f = matrix([F_x, F_y, F_z]).transpose()

    def Update_R(self, euler_data):
        # 基坐标系 -> 传感器坐标系 的旋转矩阵（与补偿阶段保持一致）
        # 注意：测量的力/力矩在传感器坐标系下，因此应使用 R_sb（base->sensor）
        R_array = self.sensor_to_base_R(euler_data, self.sensor_alpha).T
        print("R_array= ", R_array)
        #构造大矩阵R，形成Nx6矩阵，用来求解B，其中N为采集的数据组数，B包含末端重力g，姿态角U,V以及传感器零点力F_x0,F_y0,F_z0
        if (any(self.R)):
            R_1 = hstack((R_array, np.eye(3)))
            self.R = vstack((self.R, R_1))
        else:
            self.R = hstack((R_array, np.eye(3)))

    def Solve_B(self):
        # 使用最小二乘求解，避免显式求逆带来的数值不稳定
        B, *_ = np.linalg.lstsq(self.R, self.f, rcond=None)
        # 拟合残差评估（RMSE）
        f_hat = self.R @ B
        res = self.f - f_hat
        try:
            rmse = math.sqrt(np.mean(np.array(res)**2))
            print(f"拟合RMSE: {rmse:.4f} (N)")
        except Exception:
            pass
        # 计算重力加速度g，姿态角U,V以及传感器零点力F_x0,F_y0,F_z0，g=根号下的三轴力平方和 u=arcsin(-Fy/g) v=arctan(-Fx/Fz)
        # 直接保存重力向量分量（传感器坐标中的重力等效力在基坐标中的投影经R构建求得）
        self.gx = B[0].item()
        self.gy = B[1].item()
        self.gz = B[2].item()
        self.g = math.sqrt(self.gx**2 + self.gy**2 + self.gz**2)
        # 仅用于显示（可选）：通过分量反解U/V，使用 atan2 保证象限正确
        # 注意：这里的U/V是从[gx,gy,gz]推导出的姿态参数，不一定与机器人本体的rx/ry直接等价
        self.U =0# math.asin(-self.gy / self.g) if self.g != 0 else 0.0
        self.V = 0#math.atan2(-self.gx, self.gz) if self.g != 0 else 0.0

        # 存储原始零点值用于计算，打印时再四舍五入
        self.F_x0 = B[3, 0]
        self.F_y0 = B[4, 0]
        self.F_z0 = B[5, 0]

        print("B= \n" , B)
        mass_kg = self.g / 9.81 if self.g != 0 else 0.0
        print("g=", round(mass_kg, 4), "kg")
        # 当质量很小时，U/V 对噪声非常敏感，提示并跳过显示
        if mass_kg < 0.2:
            print("U/V: 载荷过小（<0.2 kg），此时由拟合得到的U/V不可靠，建议忽略或用机械臂姿态计算。")
        else:
            print("U=", round(self.U * 180 / math.pi, 4),"°")
            print("V=", round(self.V * 180 / math.pi, 4),"°")
        print("F_x0= ", round(float(self.F_x0), 2), "N")
        print("F_y0= ", round(float(self.F_y0), 2), "N")
        print("F_z0= ", round(float(self.F_z0), 2), "N")
        # 使用重力分量进行二次拟合以稳定r与力矩零点
        self.Refine_A_with_gravity()

    def Solve_Force(self, force_data, euler_data):
        Force_input = matrix([force_data[0], force_data[1], force_data[2]]).transpose()
        # 使用解算得到的重力分量，避免通过U/V重构引入误差
        my_f = matrix([self.gx, self.gy, self.gz, self.F_x0, self.F_y0, self.F_z0]).transpose()

        # 使用与参数求解相同的 基->传感器 旋转矩阵（R_sb = R_bs^T），确保一致性
        R_sb = self.sensor_to_base_R(euler_data, self.sensor_alpha).T
        R_1 = hstack((R_sb, np.eye(3)))

        Force_ex = Force_input - dot(R_1, my_f)
        # 保存结果
        self.F_ex = Force_ex[0, 0]
        self.F_ey = Force_ex[1, 0]
        self.F_ez = Force_ex[2, 0]
        print('接触力：', Force_ex.T)

    def Solve_Torque(self, torque_data, euler_data):
        Torque_input = matrix([torque_data[0], torque_data[1], torque_data[2]]).transpose()
        M_x0 = self.k1 - self.F_y0 * self.z + self.F_z0 * self.y
        M_y0 = self.k2 - self.F_z0 * self.x + self.F_x0 * self.z
        M_z0 = self.k3 - self.F_x0 * self.y + self.F_y0 * self.x

        Torque_zero = matrix([M_x0, M_y0, M_z0]).transpose()

        Gravity_param = matrix([[0, -self.z, self.y],
                                [self.z, 0, -self.x],
                                [-self.y, self.x, 0]])

        # 使用解算得到的重力分量
        Gravity_input = matrix([self.gx, self.gy, self.gz]).transpose()

        # 与上面一致，使用 基->传感器 旋转
        R_sb = self.sensor_to_base_R(euler_data, self.sensor_alpha).T

        Torque_ex = Torque_input - Torque_zero - dot(dot(Gravity_param, R_sb), Gravity_input)
        # 保存结果
        self.M_ex = Torque_ex[0, 0]
        self.M_ey = Torque_ex[1, 0]
        self.M_ez = Torque_ex[2, 0]
        print('接触力矩：', Torque_ex.T)

    def eulerAngles2rotationMat(self, theta):
        # theta: [rx, ry, rz]，单位为弧度
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]
                        ])

        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]
                        ])

        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])

    
        # 内禀XYZ（依次绕X、Y、Z），对应矩阵右乘顺序 Rz * Ry * Rx
        R = np.dot(np.dot(R_z, R_y), R_x)
        return R

    def sensor_to_base_R(self, euler_data, alpha_z_rad: float = 0.0):
        """
        计算 传感器坐标系 -> 基坐标系 的旋转矩阵。
        参数：
        - euler_data: [rx, ry, rz] 弧度，末端（法兰/工具）相对基坐标的欧拉角（XYZ顺序）
        - alpha_z_rad: 传感器相对末端坐标系绕Z轴的固定安装角偏差（弧度），无偏差填0

        约定：
        v_b = R_be @ R_es @ v_s，其中 R_be 为末端->基坐标旋转，R_es 为传感器->末端旋转
        返回 R_bs = R_be @ R_es
        """
        R_be = self.eulerAngles2rotationMat(euler_data)  # 末端 -> 基坐标
        R_es = np.array([[math.cos(alpha_z_rad), -math.sin(alpha_z_rad), 0],
                         [math.sin(alpha_z_rad),  math.cos(alpha_z_rad), 0],
                         [0,                      0,                     1]])  # 传感器 -> 末端（绕Z）
        R_bs = np.dot(R_be, R_es)
        return R_bs

    def compensate(self, force_data, torque_data, euler_data):
        # 计算补偿后的力
        self.Solve_Force(force_data, euler_data)
        # 计算补偿后的力矩
        self.Solve_Torque(torque_data, euler_data)
        # 可以返回补偿后的力和力矩
        # 这里假设你在Solve_Force和Solve_Torque里保存了结果到self.F_ex, self.M_ex等
        return self.F_ex, self.M_ex

    def save_params(self, filename):
        """保存标定参数到 JSON 文件"""
        params = {
            'x': self.x, 'y': self.y, 'z': self.z,
            'k1': self.k1, 'k2': self.k2, 'k3': self.k3,
            'gx': self.gx, 'gy': self.gy, 'gz': self.gz,
            'g': self.g,
            'U': self.U, 'V': self.V,
            'F_x0': self.F_x0, 'F_y0': self.F_y0, 'F_z0': self.F_z0,
            'M_x0': self.M_x0, 'M_y0': self.M_y0, 'M_z0': self.M_z0,
            'sensor_alpha': self.sensor_alpha
        }
        with open(filename, 'w') as f:
            json.dump(params, f, indent=4)
        print(f"参数已保存到 {filename}")

    def load_params(self, filename):
        """从 JSON 文件加载标定参数"""
        with open(filename, 'r') as f:
            params = json.load(f)
        self.x = params['x']
        self.y = params['y']
        self.z = params['z']
        self.k1 = params['k1']
        self.k2 = params['k2']
        self.k3 = params['k3']
        self.gx = params['gx']
        self.gy = params['gy']
        self.gz = params['gz']
        self.g = params['g']
        self.U = params['U']
        self.V = params['V']
        self.F_x0 = params['F_x0']
        self.F_y0 = params['F_y0']
        self.F_z0 = params['F_z0']
        self.M_x0 = params['M_x0']
        self.M_y0 = params['M_y0']
        self.M_z0 = params['M_z0']
        self.sensor_alpha = params['sensor_alpha']
        print(f"参数已从 {filename} 加载")

def main():

    #第一个点，关节数据
    arm.rm_movej([0,0,0,90,0,90,45],25,0,0,1)
    time.sleep(0.5)
    #欧拉角数据
    information=arm.rm_get_current_arm_state()
    euler_data=information[1]['pose'][3:]
    #获取一次力传感器数据
    status, data = arm.rm_get_force_data()
    force_data= list(data['force_data'][:3])
    torque_data= list(data['force_data'][3:])
    print("force_data:", force_data," euler_data:", euler_data)
    
    #第二个点，关节数据
    arm.rm_movej([0,0,0,-90,0,0,-45],25,0,0,1)
    time.sleep(0.5)
    #欧拉角数据
    information=arm.rm_get_current_arm_state()
    euler_data1=information[1]['pose'][3:]
    #获取一次力传感器数据
    status, data = arm.rm_get_force_data()
    force_data1= list(data['force_data'][:3])
    torque_data1= list(data['force_data'][3:])
    print("force_data1:", force_data1," euler_data1:", euler_data1)

    #第三个点，关节数据
    arm.rm_movej([0,0,0,90,0,-90,90],25,0,0,1)
    time.sleep(0.5)
    #欧拉角数据
    information=arm.rm_get_current_arm_state()
    euler_data2=information[1]['pose'][3:]
    #获取一次力传感器数据
    status, data = arm.rm_get_force_data()
    force_data2= list(data['force_data'][:3])
    torque_data2= list(data['force_data'][3:])
    print("force_data2:", force_data2," euler_data2:", euler_data2)

    #第四个点，关节数据
    arm.rm_movej([0,0,0,-90,20,-45,45],25,0,0,1)
    time.sleep(0.5)
    #欧拉角数据
    information=arm.rm_get_current_arm_state()
    euler_data3=information[1]['pose'][3:]
    #获取一次力传感器数据
    status, data = arm.rm_get_force_data()
    force_data3= list(data['force_data'][:3])
    torque_data3= list(data['force_data'][3:])
    print("force_data3:", force_data3," euler_data3:", euler_data3)

    #第五个点，关节数据
    arm.rm_movej([0,0,0,-90,25,45,-30],25,0,0,1)
    time.sleep(0.5)
    #欧拉角数据
    information=arm.rm_get_current_arm_state()
    euler_data4=information[1]['pose'][3:]
    #获取一次力传感器数据
    status, data = arm.rm_get_force_data()
    force_data4= list(data['force_data'][:3])
    torque_data4= list(data['force_data'][3:])
    print("force_data4:", force_data4," euler_data4:", euler_data4)

    #第六个点，关节数据
    arm.rm_movej([0,0,0,90,-25,45,-30],25,0,0,1)
    time.sleep(0.5)
    information = arm.rm_get_current_arm_state()
    euler_data5 = information[1]['pose'][3:]
    status, data = arm.rm_get_force_data()
    force_data5 = list(data['force_data'][:3])
    torque_data5 = list(data['force_data'][3:])
    print("force_data5:", force_data5, " euler_data5:", euler_data5)

    #第七个点，关节数据
    arm.rm_movej([0,0,0,-90,45,-45,45],25,0,0,1)
    time.sleep(0.5)
    information = arm.rm_get_current_arm_state()
    euler_data6 = information[1]['pose'][3:]
    status, data = arm.rm_get_force_data()
    force_data6 = list(data['force_data'][:3])
    torque_data6 = list(data['force_data'][3:])
    print("force_data6:", force_data6, " euler_data6:", euler_data6)

    #第八个点，关节数据
    arm.rm_movej([0,0,0,90,45,45,45],25,0,0,1)
    time.sleep(0.5)
    information = arm.rm_get_current_arm_state()
    euler_data7 = information[1]['pose'][3:]
    status, data = arm.rm_get_force_data()
    force_data7 = list(data['force_data'][:3])
    torque_data7 = list(data['force_data'][3:])
    print("force_data7:", force_data7, " euler_data7:", euler_data7)

    #第九个点，关节数据
    arm.rm_movej([0,0,0,-90,30,30,30],25,0,0,1)
    time.sleep(0.5)
    information = arm.rm_get_current_arm_state()
    euler_data8 = information[1]['pose'][3:]
    status, data = arm.rm_get_force_data()
    force_data8 = list(data['force_data'][:3])
    torque_data8 = list(data['force_data'][3:])
    print("force_data8:", force_data8, " euler_data8:", euler_data8)


    #第10个点，关节数据
    arm.rm_movej([0,0,0,90,-30,30,30],25,0,0,1)
    time.sleep(0.5)
    information = arm.rm_get_current_arm_state()
    euler_data9 = information[1]['pose'][3:]
    status, data = arm.rm_get_force_data()
    force_data9 = list(data['force_data'][:3])
    torque_data9 = list(data['force_data'][3:])
    print("force_data9:", force_data9, " euler_data9:", euler_data9)

    #第11个点，关节数据
    arm.rm_movej([0,0,0,90,-30,-30,30],25,0,0,1)
    time.sleep(0.5)
    information = arm.rm_get_current_arm_state()
    euler_data10 = information[1]['pose'][3:]
    status, data = arm.rm_get_force_data()
    force_data10 = list(data['force_data'][:3])
    torque_data10= list(data['force_data'][3:])
    print("force_data10:", force_data10, " euler_data10:", euler_data10)

    #第12个点，关节数据
    arm.rm_movej([0,0,0,-90,30,-30,-30],25,0,0,1)
    time.sleep(0.5)
    information = arm.rm_get_current_arm_state()
    euler_data11 = information[1]['pose'][3:]
    status, data = arm.rm_get_force_data()
    force_data11 = list(data['force_data'][:3])
    torque_data11 = list(data['force_data'][3:])
    print("force_data11:", force_data11, " euler_data11:", euler_data11)




    compensation = GravityCompensation()

    compensation.Update_F(force_data)
    compensation.Update_F(force_data1)
    compensation.Update_F(force_data2)
    compensation.Update_F(force_data3)
    compensation.Update_F(force_data4)
    compensation.Update_F(force_data5)
    compensation.Update_F(force_data6)
    compensation.Update_F(force_data7)
    compensation.Update_F(force_data8)
    compensation.Update_F(force_data9)
    compensation.Update_F(force_data10)
    compensation.Update_F(force_data11)
    

    compensation.Update_M(torque_data)
    compensation.Update_M(torque_data1)
    compensation.Update_M(torque_data2)
    compensation.Update_M(torque_data3)
    compensation.Update_M(torque_data4)
    compensation.Update_M(torque_data5)
    compensation.Update_M(torque_data6)
    compensation.Update_M(torque_data7)
    compensation.Update_M(torque_data8)
    compensation.Update_M(torque_data9)
    compensation.Update_M(torque_data10)
    compensation.Update_M(torque_data11)
    


    compensation.Solve_A()

    compensation.Update_f(force_data)
    compensation.Update_f(force_data1)
    compensation.Update_f(force_data2)
    compensation.Update_f(force_data3)
    compensation.Update_f(force_data4)
    compensation.Update_f(force_data5)
    compensation.Update_f(force_data6)
    compensation.Update_f(force_data7)
    compensation.Update_f(force_data8)
    compensation.Update_f(force_data9)
    compensation.Update_f(force_data10)
    compensation.Update_f(force_data11)

    compensation.Update_R(euler_data)
    compensation.Update_R(euler_data1)
    compensation.Update_R(euler_data2)
    compensation.Update_R(euler_data3)
    compensation.Update_R(euler_data4)
    compensation.Update_R(euler_data5)
    compensation.Update_R(euler_data6)
    compensation.Update_R(euler_data7)
    compensation.Update_R(euler_data8)
    compensation.Update_R(euler_data9)
    compensation.Update_R(euler_data10)
    compensation.Update_R(euler_data11)

    compensation.Solve_B()

    
    results = []  # 用于存储每次采集的补偿力和力矩

    import pandas as pd
    results = []
    try:
        while True:
            info = arm.rm_get_current_arm_state()
            euler = info[1]['pose'][3:]
            status, data = arm.rm_get_force_data()
            force = list(data['force_data'][:3])
            torque = list(data['force_data'][3:])
            compensation.Solve_Force(force, euler)
            compensation.Solve_Torque(torque, euler)
            # 保存补偿后的力和力矩到列表
            results.append([
                compensation.F_ex, compensation.F_ey, compensation.F_ez,
                compensation.M_ex, compensation.M_ey, compensation.M_ez
            ])
            # 每采集100次保存一次Excel（可根据实际需求调整）
            if len(results) % 100 == 0:
                df = pd.DataFrame(results, columns=['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
                df.to_excel('compensated_forces.xlsx', index=False)
                print("已保存100组数据到 compensated_forces.xlsx")
    except KeyboardInterrupt:
        # 程序被中断时自动保存所有已采集数据
        if results:
            df = pd.DataFrame(results, columns=['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
            df.to_excel('compensated_forces.xlsx', index=False)
            print("已保存所有数据到 compensated_forces.xlsx")
        else:
            print("没有采集到数据，无需保存。")

        # 保存补偿后的力和力矩到列表
        results.append([
            compensation.F_ex, compensation.F_ey, compensation.F_ez,
            compensation.M_ex, compensation.M_ey, compensation.M_ez
        ])

        # 每采集100次保存一次Excel（可根据实际需求调整）
        if len(results) % 8000 == 0:
            df = pd.DataFrame(results, columns=['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
            df.to_excel('compensated_forces.xlsx', index=False)
            print("已保存8000组数据到 compensated_forces.xlsx")
        


if __name__ == '__main__':
    main()

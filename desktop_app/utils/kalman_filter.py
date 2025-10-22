"""
扩展卡尔曼滤波(EKF)姿态估计算法
融合加速度计、陀螺仪和磁力计数据，实现高精度、无漂移的姿态估计

优势：
1. 融合多传感器数据，动态调整信任度
2. 平滑输出，响应快速
3. 无漂移（通过磁力计校正）
4. 自适应噪声协方差调整
"""
import numpy as np
import math


class ExtendedKalmanFilter:
    """
    扩展卡尔曼滤波器 - 用于姿态估计
    
    状态向量: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
    - roll, pitch, yaw: 姿态角（弧度）
    - gyro_bias_*: 陀螺仪零偏（弧度/秒）
    """
    
    def __init__(self, process_noise=0.01, accel_noise=0.1, gyro_noise=0.01, mag_noise=0.05):
        """
        初始化EKF滤波器
        
        Args:
            process_noise: 过程噪声（系统模型不确定性）
            accel_noise: 加速度计测量噪声标准差
            gyro_noise: 陀螺仪测量噪声标准差
            mag_noise: 磁力计（角度传感器）测量噪声标准差
        """
        # 状态向量 [roll, pitch, yaw, bias_x, bias_y, bias_z]
        self.state = np.zeros(6, dtype=np.float64)
        
        # 状态协方差矩阵
        self.P = np.eye(6, dtype=np.float64) * 1.0
        
        # 过程噪声协方差
        self.Q = np.eye(6, dtype=np.float64)
        self.Q[0:3, 0:3] *= process_noise  # 姿态角过程噪声
        self.Q[3:6, 3:6] *= process_noise * 0.001  # 零偏变化很慢
        
        # 测量噪声协方差
        self.R_accel = np.eye(3, dtype=np.float64) * (accel_noise ** 2)
        self.R_mag = np.eye(1, dtype=np.float64) * (mag_noise ** 2)
        
        # 陀螺仪噪声
        self.gyro_noise = gyro_noise
        
        # 动态调整参数
        self.accel_trust_factor = 1.0  # 加速度计信任度 (0-1)
        self.mag_trust_factor = 1.0    # 磁力计信任度 (0-1)
        
        # 统计信息
        self.update_count = 0
        
    def predict(self, gyro_x, gyro_y, gyro_z, dt):
        """
        预测步骤：使用陀螺仪数据预测下一时刻的姿态
        
        Args:
            gyro_x, gyro_y, gyro_z: 陀螺仪角速度（度/秒）
            dt: 时间间隔（秒）
        """
        # 转换为弧度/秒
        gx = math.radians(gyro_x)
        gy = math.radians(gyro_y)
        gz = math.radians(gyro_z)
        
        # 提取当前状态
        roll = self.state[0]
        pitch = self.state[1]
        yaw = self.state[2]
        bias_x = self.state[3]
        bias_y = self.state[4]
        bias_z = self.state[5]
        
        # 去除零偏
        gx_corrected = gx - bias_x
        gy_corrected = gy - bias_y
        gz_corrected = gz - bias_z
        
        # 欧拉角微分方程（考虑万向节锁的影响）
        # droll/dt, dpitch/dt, dyaw/dt
        sin_roll = math.sin(roll)
        cos_roll = math.cos(roll)
        sin_pitch = math.sin(pitch)
        cos_pitch = math.cos(pitch)
        tan_pitch = math.tan(pitch)
        
        # 防止除零
        if abs(cos_pitch) < 0.01:
            cos_pitch = 0.01 if cos_pitch >= 0 else -0.01
        
        # 姿态角更新
        droll = gx_corrected + sin_roll * tan_pitch * gy_corrected + cos_roll * tan_pitch * gz_corrected
        dpitch = cos_roll * gy_corrected - sin_roll * gz_corrected
        dyaw = (sin_roll / cos_pitch) * gy_corrected + (cos_roll / cos_pitch) * gz_corrected
        
        # 更新状态（欧拉积分）
        self.state[0] += droll * dt
        self.state[1] += dpitch * dt
        self.state[2] += dyaw * dt
        # 零偏保持不变（在预测阶段）
        
        # 归一化角度到 [-pi, pi]
        self.state[0] = self._normalize_angle(self.state[0])
        self.state[1] = self._normalize_angle(self.state[1])
        self.state[2] = self._normalize_angle(self.state[2])
        
        # 计算状态转移雅可比矩阵 F
        F = self._compute_state_jacobian(gx_corrected, gy_corrected, gz_corrected, dt)
        
        # 更新协方差矩阵
        # P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q
        
    def update_accel(self, accel_x, accel_y, accel_z):
        """
        更新步骤：使用加速度计数据校正姿态（Roll和Pitch）
        
        Args:
            accel_x, accel_y, accel_z: 加速度计读数（g）
        """
        # 归一化加速度
        norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if norm < 0.1:  # 加速度太小，不可信
            return
        
        ax = accel_x / norm
        ay = accel_y / norm
        az = accel_z / norm
        
        # 动态调整加速度计信任度（根据加速度大小）
        # 当加速度接近1g时，更可信（静止或匀速运动）
        accel_magnitude_error = abs(norm - 1.0)
        if accel_magnitude_error > 0.5:  # 加速度变化大，不太可信
            self.accel_trust_factor = 0.3
        elif accel_magnitude_error > 0.2:
            self.accel_trust_factor = 0.7
        else:
            self.accel_trust_factor = 1.0
        
        # 从加速度计计算Roll和Pitch
        # Roll: atan2(ay, az)
        # Pitch: atan2(-ax, sqrt(ay^2 + az^2))
        measured_roll = math.atan2(ay, az)
        measured_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        # 测量向量 z = [measured_roll, measured_pitch]
        # 注意：加速度计不能测量Yaw
        z = np.array([measured_roll, measured_pitch, 0.0], dtype=np.float64)
        
        # 预测的测量 h(x) = [roll, pitch, 0]
        h = np.array([self.state[0], self.state[1], 0.0], dtype=np.float64)
        
        # 测量残差 y = z - h(x)
        y = z - h
        y[0] = self._normalize_angle(y[0])
        y[1] = self._normalize_angle(y[1])
        
        # 测量雅可比矩阵 H (3x6)
        H = np.zeros((3, 6), dtype=np.float64)
        H[0, 0] = 1.0  # roll
        H[1, 1] = 1.0  # pitch
        # yaw不由加速度计测量
        
        # 调整测量噪声（根据信任度）
        R = self.R_accel.copy()
        R *= (1.0 / max(0.1, self.accel_trust_factor))
        
        # 卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 更新状态
        self.state += K @ y
        
        # 归一化角度
        self.state[0] = self._normalize_angle(self.state[0])
        self.state[1] = self._normalize_angle(self.state[1])
        self.state[2] = self._normalize_angle(self.state[2])
        
        # 更新协方差 P = (I - K * H) * P
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ H) @ self.P
        
    def update_magnetometer(self, mag_angle):
        """
        更新步骤：使用磁力计（MT6701角度传感器）数据校正Yaw角
        
        Args:
            mag_angle: 磁力计测量的角度（度，0-360）
        """
        # 转换为弧度，并归一化到 [-pi, pi]
        measured_yaw = self._normalize_angle(math.radians(mag_angle))
        
        # 测量向量 z = [measured_yaw]
        z = np.array([measured_yaw], dtype=np.float64)
        
        # 预测的测量 h(x) = [yaw]
        h = np.array([self.state[2]], dtype=np.float64)
        
        # 测量残差 y = z - h(x)
        y = z - h
        y[0] = self._normalize_angle(y[0])
        
        # 测量雅可比矩阵 H (1x6)
        H = np.zeros((1, 6), dtype=np.float64)
        H[0, 2] = 1.0  # yaw
        
        # 调整测量噪声（根据信任度）
        R = self.R_mag.copy()
        R *= (1.0 / max(0.1, self.mag_trust_factor))
        
        # 卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 更新状态
        self.state += K @ y
        
        # 归一化角度
        self.state[0] = self._normalize_angle(self.state[0])
        self.state[1] = self._normalize_angle(self.state[1])
        self.state[2] = self._normalize_angle(self.state[2])
        
        # 更新协方差 P = (I - K * H) * P
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ H) @ self.P
    
    def _compute_state_jacobian(self, gx, gy, gz, dt):
        """计算状态转移雅可比矩阵 F"""
        F = np.eye(6, dtype=np.float64)
        
        roll = self.state[0]
        pitch = self.state[1]
        
        sin_roll = math.sin(roll)
        cos_roll = math.cos(roll)
        sin_pitch = math.sin(pitch)
        cos_pitch = math.cos(pitch)
        tan_pitch = math.tan(pitch)
        
        # 防止除零
        if abs(cos_pitch) < 0.01:
            cos_pitch = 0.01 if cos_pitch >= 0 else -0.01
        
        # droll/droll, droll/dpitch
        F[0, 0] = 1.0 + dt * (cos_roll * tan_pitch * gy - sin_roll * tan_pitch * gz)
        F[0, 1] = dt * (sin_roll / (cos_pitch**2) * gy + cos_roll / (cos_pitch**2) * gz)
        
        # dpitch/droll
        F[1, 0] = dt * (-sin_roll * gy - cos_roll * gz)
        F[1, 1] = 1.0
        
        # dyaw/droll, dyaw/dpitch
        F[2, 0] = dt * ((cos_roll / cos_pitch) * gy - (sin_roll / cos_pitch) * gz)
        F[2, 1] = dt * (sin_roll * sin_pitch / (cos_pitch**2) * gy + 
                       cos_roll * sin_pitch / (cos_pitch**2) * gz)
        F[2, 2] = 1.0
        
        # 零偏影响
        F[0, 3] = -dt
        F[0, 4] = -dt * sin_roll * tan_pitch
        F[0, 5] = -dt * cos_roll * tan_pitch
        
        F[1, 4] = -dt * cos_roll
        F[1, 5] = dt * sin_roll
        
        F[2, 4] = -dt * sin_roll / cos_pitch
        F[2, 5] = -dt * cos_roll / cos_pitch
        
        return F
    
    def _normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_euler_angles(self):
        """
        获取当前姿态角
        
        Returns:
            (roll, pitch, yaw): 姿态角（度）
        """
        return (
            math.degrees(self.state[0]),
            math.degrees(self.state[1]),
            math.degrees(self.state[2])
        )
    
    def get_gyro_bias(self):
        """
        获取陀螺仪零偏估计
        
        Returns:
            (bias_x, bias_y, bias_z): 零偏（度/秒）
        """
        return (
            math.degrees(self.state[3]),
            math.degrees(self.state[4]),
            math.degrees(self.state[5])
        )
    
    def set_mag_trust(self, trust_factor):
        """
        设置磁力计信任度
        
        Args:
            trust_factor: 信任度 (0-1)，0表示完全不信任，1表示完全信任
        """
        self.mag_trust_factor = max(0.0, min(1.0, trust_factor))
    
    def set_process_noise(self, noise_level):
        """
        动态调整过程噪声（用于适应运动状态）
        
        Args:
            noise_level: 噪声级别 (0.001-1.0)
                        - 低噪声：静止或缓慢运动
                        - 高噪声：快速运动或振动
        """
        self.Q[0:3, 0:3] = np.eye(3) * noise_level
        self.Q[3:6, 3:6] = np.eye(3) * noise_level * 0.001
    
    def reset(self):
        """重置滤波器"""
        self.state = np.zeros(6, dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 1.0
        self.update_count = 0
        self.accel_trust_factor = 1.0
        self.mag_trust_factor = 1.0
    
    def get_state_covariance(self):
        """
        获取状态协方差（用于评估估计的不确定性）
        
        Returns:
            姿态角的标准差 (roll_std, pitch_std, yaw_std) 单位：度
        """
        return (
            math.degrees(math.sqrt(self.P[0, 0])),
            math.degrees(math.sqrt(self.P[1, 1])),
            math.degrees(math.sqrt(self.P[2, 2]))
        )


class AdaptiveEKFAttitudeEstimator:
    """
    自适应EKF姿态估计器 - 高级封装类
    
    特点：
    1. 自动融合加速度计、陀螺仪、磁力计数据
    2. 动态调整传感器信任度
    3. 自适应过程噪声调整
    4. 提供平滑、快速响应、无漂移的姿态估计
    """
    
    def __init__(self):
        """初始化自适应EKF估计器"""
        self.ekf = ExtendedKalmanFilter(
            process_noise=0.01,
            accel_noise=0.1,
            gyro_noise=0.01,
            mag_noise=0.05
        )
        
        # 运动检测
        self.accel_history = []
        self.max_history_len = 10
        
        # 统计
        self.update_count = 0
        
    def update(self, accel_x, accel_y, accel_z, 
               gyro_x, gyro_y, gyro_z, 
               mag_angle=None, mag_valid=False,
               dt=0.1):
        """
        更新姿态估计
        
        Args:
            accel_x, accel_y, accel_z: 加速度（g）
            gyro_x, gyro_y, gyro_z: 角速度（度/秒）
            mag_angle: 磁力计角度（度，0-360），可选
            mag_valid: 磁力计数据是否有效
            dt: 时间间隔（秒）
        
        Returns:
            (roll, pitch, yaw): 姿态角（度）
        """
        # 1. 预测步骤（使用陀螺仪）
        self.ekf.predict(gyro_x, gyro_y, gyro_z, dt)
        
        # 2. 更新步骤（使用加速度计）
        self.ekf.update_accel(accel_x, accel_y, accel_z)
        
        # 3. 更新步骤（使用磁力计，如果可用）
        if mag_valid and mag_angle is not None:
            self.ekf.update_magnetometer(mag_angle)
        
        # 4. 自适应调整过程噪声
        self._adapt_process_noise(accel_x, accel_y, accel_z)
        
        self.update_count += 1
        
        # 返回姿态角
        return self.ekf.get_euler_angles()
    
    def _adapt_process_noise(self, ax, ay, az):
        """根据运动状态自适应调整过程噪声"""
        # 计算加速度大小
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        
        # 维护历史记录
        self.accel_history.append(accel_mag)
        if len(self.accel_history) > self.max_history_len:
            self.accel_history.pop(0)
        
        # 计算加速度变化（标准差）
        if len(self.accel_history) >= 3:
            accel_std = np.std(self.accel_history)
            
            # 根据加速度变化调整过程噪声
            if accel_std > 0.3:  # 快速运动/振动
                noise_level = 0.05
            elif accel_std > 0.1:  # 中等运动
                noise_level = 0.02
            else:  # 静止或缓慢运动
                noise_level = 0.01
            
            self.ekf.set_process_noise(noise_level)
    
    def get_euler_angles(self):
        """获取当前姿态角（度）"""
        return self.ekf.get_euler_angles()
    
    def get_gyro_bias(self):
        """获取陀螺仪零偏估计（度/秒）"""
        return self.ekf.get_gyro_bias()
    
    def get_uncertainty(self):
        """获取估计的不确定性（度）"""
        return self.ekf.get_state_covariance()
    
    def set_mag_trust(self, trust_factor):
        """设置磁力计信任度 (0-1)"""
        self.ekf.set_mag_trust(trust_factor)
    
    def reset(self):
        """重置估计器"""
        self.ekf.reset()
        self.accel_history.clear()
        self.update_count = 0


# 测试代码
if __name__ == '__main__':
    print("=" * 80)
    print(" 扩展卡尔曼滤波(EKF)姿态估计算法测试")
    print("=" * 80)
    
    # 测试1: 基本功能测试（静止状态）
    print("\n【测试1】静止状态")
    print("-" * 80)
    estimator = AdaptiveEKFAttitudeEstimator()
    
    for i in range(20):
        roll, pitch, yaw = estimator.update(
            accel_x=0.0, accel_y=0.0, accel_z=1.0,
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            mag_angle=0.0, mag_valid=True,
            dt=0.1
        )
        if i % 5 == 0:
            uncertainty = estimator.get_uncertainty()
            print(f"  Frame {i:2d}: Roll={roll:7.2f}° Pitch={pitch:7.2f}° Yaw={yaw:7.2f}°  "
                  f"不确定性: ±{uncertainty[0]:.2f}° ±{uncertainty[1]:.2f}° ±{uncertainty[2]:.2f}°")
    
    # 测试2: 绕Z轴旋转
    print("\n【测试2】绕Z轴旋转（20°/s）")
    print("-" * 80)
    estimator.reset()
    
    for i in range(20):
        mag_angle = (i * 2.0) % 360  # 模拟磁力计读数
        roll, pitch, yaw = estimator.update(
            accel_x=0.0, accel_y=0.0, accel_z=1.0,
            gyro_x=0.0, gyro_y=0.0, gyro_z=20.0,
            mag_angle=mag_angle, mag_valid=True,
            dt=0.1
        )
        if i % 5 == 0:
            print(f"  Frame {i:2d}: Roll={roll:7.2f}° Pitch={pitch:7.2f}° Yaw={yaw:7.2f}°  "
                  f"磁力计={mag_angle:7.2f}°")
    
    # 测试3: 倾斜测试（30度）
    print("\n【测试3】设备倾斜30度")
    print("-" * 80)
    estimator.reset()
    
    angle_30 = math.radians(30)
    for i in range(30):
        roll, pitch, yaw = estimator.update(
            accel_x=0.0,
            accel_y=math.sin(angle_30),
            accel_z=math.cos(angle_30),
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            mag_angle=0.0, mag_valid=True,
            dt=0.1
        )
        if i % 10 == 0 or i == 29:
            print(f"  Frame {i:2d}: Roll={roll:7.2f}° Pitch={pitch:7.2f}° Yaw={yaw:7.2f}°")
    
    # 测试4: 陀螺仪零偏估计
    print("\n【测试4】陀螺仪零偏估计")
    print("-" * 80)
    estimator.reset()
    
    # 模拟陀螺仪有固定零偏
    bias_x_sim = 2.0  # 2度/秒的零偏
    for i in range(50):
        roll, pitch, yaw = estimator.update(
            accel_x=0.0, accel_y=0.0, accel_z=1.0,
            gyro_x=bias_x_sim, gyro_y=0.0, gyro_z=0.0,  # 实际静止，但陀螺仪有读数
            mag_angle=0.0, mag_valid=True,
            dt=0.1
        )
        if i % 10 == 0:
            bias = estimator.get_gyro_bias()
            print(f"  Frame {i:2d}: 零偏估计 X={bias[0]:6.2f}° Y={bias[1]:6.2f}° Z={bias[2]:6.2f}°/s  "
                  f"(真实零偏 X={bias_x_sim:.2f}°/s)")
    
    # 测试5: 磁力计融合测试
    print("\n【测试5】磁力计融合（Yaw漂移校正）")
    print("-" * 80)
    estimator.reset()
    
    for i in range(30):
        # 模拟陀螺仪有微小漂移
        gyro_z_with_drift = 0.5 if i < 20 else 0.0
        mag_angle = 45.0  # 真实方向是45度
        
        roll, pitch, yaw = estimator.update(
            accel_x=0.0, accel_y=0.0, accel_z=1.0,
            gyro_x=0.0, gyro_y=0.0, gyro_z=gyro_z_with_drift,
            mag_angle=mag_angle, mag_valid=True,
            dt=0.1
        )
        if i % 5 == 0:
            print(f"  Frame {i:2d}: Yaw={yaw:7.2f}°  磁力计={mag_angle:7.2f}°  "
                  f"陀螺仪漂移={'有' if gyro_z_with_drift > 0 else '无'}")
    
    # 测试6: 性能测试
    print("\n【测试6】性能测试")
    print("-" * 80)
    import time
    
    estimator = AdaptiveEKFAttitudeEstimator()
    iterations = 1000
    
    start_time = time.time()
    for _ in range(iterations):
        estimator.update(0.0, 0.1, 1.0, 1.0, 2.0, 3.0, mag_angle=45.0, mag_valid=True, dt=0.1)
    end_time = time.time()
    
    elapsed = end_time - start_time
    freq = iterations / elapsed
    
    print(f"  迭代次数: {iterations}")
    print(f"  总耗时: {elapsed:.4f} 秒")
    print(f"  处理速度: {freq:.0f} Hz")
    print(f"  单次耗时: {elapsed/iterations*1000:.3f} 毫秒")
    
    print("\n" + "=" * 80)
    print(" ✅ EKF算法测试完成！")
    print(" 优势：平滑、快速响应、零偏估计、磁力计无漂移")
    print("=" * 80)


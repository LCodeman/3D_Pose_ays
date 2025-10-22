"""
四元数和姿态计算模块 - 优化版
使用四元数Madgwick算法进行姿态估计

优势：
1. 避免万向节锁问题
2. 计算效率高（适合实时系统）
3. 数值稳定性好
4. 自适应增益调整
"""
import numpy as np
import math


class MadgwickQuaternion:
    """
    Madgwick四元数姿态估计器
    
    专为6轴IMU设计（加速度计+陀螺仪），无需磁力计
    使用梯度下降算法融合传感器数据
    """
    
    def __init__(self, beta=0.1, sample_freq=10.0):
        """
        初始化Madgwick滤波器
        
        Args:
            beta: 增益参数，控制加速度计的权重 (0.01-0.5)
                  - 较小值: 更相信陀螺仪，响应慢但稳定
                  - 较大值: 更相信加速度计，响应快但可能抖动
                  - 推荐: 0.1 (通用)，0.033 (高稳定性)，0.2 (快速响应)
            sample_freq: 采样频率 (Hz)
        """
        self.beta = beta
        self.sample_freq = sample_freq
        self.sample_period = 1.0 / sample_freq
        
        # 四元数 [w, x, y, z] - 初始为单位四元数(无旋转)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        # 统计信息
        self.update_count = 0
        
    def update(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt=None):
        """
        更新姿态四元数
        
        Args:
            accel_x, accel_y, accel_z: 加速度 (g)
            gyro_x, gyro_y, gyro_z: 角速度 (度/秒)
            dt: 时间间隔 (秒)，如果为None则使用sample_period
        
        Returns:
            (roll, pitch, yaw): 姿态角(度)
        """
        if dt is not None:
            sample_period = dt
        else:
            sample_period = self.sample_period
        
        # 转换陀螺仪为弧度/秒
        gx = math.radians(gyro_x)
        gy = math.radians(gyro_y)
        gz = math.radians(gyro_z)
        
        # 归一化加速度计数据
        ax, ay, az = accel_x, accel_y, accel_z
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        
        if norm == 0:
            # 加速度为零，只使用陀螺仪
            return self._update_imu_gyro_only(gx, gy, gz, sample_period)
        
        ax /= norm
        ay /= norm
        az /= norm
        
        # 提取四元数
        q0, q1, q2, q3 = self.q
        
        # 辅助变量（避免重复计算）
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        
        # 梯度下降算法：计算目标函数的梯度
        # 目标：使预测的重力方向与加速度计测量一致
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
        
        # 归一化梯度
        norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm > 0:
            s0 /= norm
            s1 /= norm
            s2 /= norm
            s3 /= norm
        
        # 陀螺仪积分的四元数导数
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        
        # 应用梯度修正
        q0 += (qDot1 - self.beta * s0) * sample_period
        q1 += (qDot2 - self.beta * s1) * sample_period
        q2 += (qDot3 - self.beta * s2) * sample_period
        q3 += (qDot4 - self.beta * s3) * sample_period
        
        # 归一化四元数
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q[0] = q0 / norm
        self.q[1] = q1 / norm
        self.q[2] = q2 / norm
        self.q[3] = q3 / norm
        
        self.update_count += 1
        
        # 转换为欧拉角返回
        return self.get_euler_angles()
    
    def _update_imu_gyro_only(self, gx, gy, gz, dt):
        """仅使用陀螺仪更新（当加速度计数据无效时）"""
        q0, q1, q2, q3 = self.q
        
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        
        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt
        
        # 归一化
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q[0] = q0 / norm
        self.q[1] = q1 / norm
        self.q[2] = q2 / norm
        self.q[3] = q3 / norm
        
        return self.get_euler_angles()
    
    def get_euler_angles(self):
        """
        从四元数获取欧拉角
        
        Returns:
            (roll, pitch, yaw): 姿态角(度)
        """
        q0, q1, q2, q3 = self.q
        
        # Roll (X轴)
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (Y轴)
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # ±90度
        else:
            pitch = math.asin(sinp)
        
        # Yaw (Z轴)
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 转换为度数
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def get_quaternion(self):
        """
        获取当前四元数
        
        Returns:
            (w, x, y, z): 四元数
        """
        return tuple(self.q)
    
    def get_rotation_matrix(self):
        """
        获取旋转矩阵（4x4，用于OpenGL）
        
        Returns:
            4x4旋转矩阵 (numpy array)
        """
        q0, q1, q2, q3 = self.q
        
        # 优化的四元数到旋转矩阵转换
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        
        q01 = q0 * q1
        q02 = q0 * q2
        q03 = q0 * q3
        q12 = q1 * q2
        q13 = q1 * q3
        q23 = q2 * q3
        
        matrix = np.array([
            [q0q0 + q1q1 - q2q2 - q3q3, 2.0 * (q12 - q03),           2.0 * (q13 + q02),           0.0],
            [2.0 * (q12 + q03),           q0q0 - q1q1 + q2q2 - q3q3, 2.0 * (q23 - q01),           0.0],
            [2.0 * (q13 - q02),           2.0 * (q23 + q01),           q0q0 - q1q1 - q2q2 + q3q3, 0.0],
            [0.0,                         0.0,                         0.0,                         1.0]
        ], dtype=np.float32)
        
        return matrix
    
    def reset(self):
        """重置为初始姿态（单位四元数）"""
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.update_count = 0
    
    def set_beta(self, beta):
        """
        动态调整beta参数
        
        Args:
            beta: 新的beta值 (0.01-0.5)
        """
        self.beta = max(0.01, min(0.5, beta))
    
    def get_stats(self):
        """
        获取统计信息
        
        Returns:
            dict: 包含更新次数、当前四元数等信息
        """
        return {
            'updates': self.update_count,
            'quaternion': self.q.tolist(),
            'beta': self.beta,
            'sample_freq': self.sample_freq
        }


class AttitudeCalculator(MadgwickQuaternion):
    """
    姿态计算器 - 兼容旧接口的四元数Madgwick算法
    
    这是一个包装类，提供与旧版本相同的接口，但使用优化的四元数算法
    """
    
    def __init__(self, alpha=0.98):
        """
        初始化姿态计算器（兼容旧接口）
        
        Args:
            alpha: 互补滤波系数 (0.9-0.99) - 将被转换为Madgwick的beta参数
        """
        # 将alpha转换为beta
        # alpha越大越信任陀螺仪，对应beta越小
        beta = (1 - alpha) * 2.0  # 经验转换公式
        beta = max(0.033, min(0.3, beta))  # 限制范围
        
        super().__init__(beta=beta, sample_freq=10.0)
        
        # 保存欧拉角用于兼容性
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
    
    def update(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt=0.1):
        """
        更新姿态角（兼容旧接口）
        
        Args:
            accel_x, accel_y, accel_z: 加速度 (g)
            gyro_x, gyro_y, gyro_z: 角速度 (度/秒)
            dt: 时间间隔 (秒)
        
        Returns:
            (roll, pitch, yaw): 姿态角(度)
        """
        # 调用Madgwick更新
        roll, pitch, yaw = super().update(accel_x, accel_y, accel_z, 
                                          gyro_x, gyro_y, gyro_z, dt)
        
        # 保存结果
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        
        return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """
    欧拉角转四元数
    
    Args:
        roll, pitch, yaw: 欧拉角(度)
    
    Returns:
        (w, x, y, z): 四元数
    """
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return w, x, y, z


def quaternion_to_euler(w, x, y, z):
    """
    四元数转欧拉角
    
    Args:
        w, x, y, z: 四元数
    
    Returns:
        (roll, pitch, yaw): 欧拉角(度)
    """
    # Roll (x轴)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y轴)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z轴)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# 测试代码
if __name__ == '__main__':
    print("=" * 70)
    print(" 四元数Madgwick算法测试")
    print("=" * 70)
    
    # 测试1: 基本功能测试
    print("\n【测试1】静止状态（无旋转）")
    print("-" * 70)
    calculator = MadgwickQuaternion(beta=0.1)
    
    for i in range(10):
        roll, pitch, yaw = calculator.update(
            accel_x=0.0, accel_y=0.0, accel_z=1.0,
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            dt=0.1
        )
        q = calculator.get_quaternion()
        print(f"  Frame {i}: Roll={roll:6.2f}° Pitch={pitch:6.2f}° Yaw={yaw:6.2f}°  "
              f"Quat=[{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}]")
    
    # 测试2: 绕Z轴旋转
    print("\n【测试2】绕Z轴旋转（10°/s）")
    print("-" * 70)
    calculator.reset()
    
    for i in range(10):
        roll, pitch, yaw = calculator.update(
            accel_x=0.0, accel_y=0.0, accel_z=1.0,
            gyro_x=0.0, gyro_y=0.0, gyro_z=10.0,  # 10度/秒
            dt=0.1
        )
        print(f"  Frame {i}: Roll={roll:6.2f}° Pitch={pitch:6.2f}° Yaw={yaw:6.2f}°")
    
    # 测试3: 模拟倾斜
    print("\n【测试3】模拟45度倾斜")
    print("-" * 70)
    calculator.reset()
    
    # 模拟设备向右倾斜45度
    import math
    angle = math.radians(45)
    for i in range(20):
        roll, pitch, yaw = calculator.update(
            accel_x=0.0, 
            accel_y=math.sin(angle), 
            accel_z=math.cos(angle),
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            dt=0.1
        )
        if i % 5 == 0:
            print(f"  Frame {i}: Roll={roll:6.2f}° Pitch={pitch:6.2f}° Yaw={yaw:6.2f}°")
    
    # 测试4: 兼容性接口测试
    print("\n【测试4】兼容性接口测试（AttitudeCalculator）")
    print("-" * 70)
    old_calculator = AttitudeCalculator(alpha=0.98)
    
    for i in range(5):
        roll, pitch, yaw = old_calculator.update(
            accel_x=0.0, accel_y=0.0, accel_z=1.0,
            gyro_x=5.0, gyro_y=0.0, gyro_z=0.0,
            dt=0.1
        )
        print(f"  Frame {i}: Roll={roll:6.2f}° Pitch={pitch:6.2f}° Yaw={yaw:6.2f}°")
    
    # 测试5: 性能测试
    print("\n【测试5】性能测试")
    print("-" * 70)
    import time
    
    calculator = MadgwickQuaternion(beta=0.1)
    iterations = 1000
    
    start_time = time.time()
    for _ in range(iterations):
        calculator.update(0.0, 0.1, 1.0, 1.0, 2.0, 3.0, 0.1)
    end_time = time.time()
    
    elapsed = end_time - start_time
    freq = iterations / elapsed
    
    print(f"  迭代次数: {iterations}")
    print(f"  总耗时: {elapsed:.4f} 秒")
    print(f"  处理速度: {freq:.0f} Hz")
    print(f"  单次耗时: {elapsed/iterations*1000:.3f} 毫秒")
    
    # 测试6: 欧拉角与四元数互转
    print("\n【测试6】欧拉角与四元数互转测试")
    print("-" * 70)
    
    test_angles = [(30, 45, 60), (0, 90, 0), (45, 0, 180)]
    for roll_in, pitch_in, yaw_in in test_angles:
        # 欧拉角 -> 四元数
        q = euler_to_quaternion(roll_in, pitch_in, yaw_in)
        # 四元数 -> 欧拉角
        roll_out, pitch_out, yaw_out = quaternion_to_euler(*q)
        
        print(f"  输入: R={roll_in:6.2f}° P={pitch_in:6.2f}° Y={yaw_in:6.2f}°")
        print(f"  四元数: [{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}]")
        print(f"  输出: R={roll_out:6.2f}° P={pitch_out:6.2f}° Y={yaw_out:6.2f}°")
        print()
    
    print("=" * 70)
    print(" ✅ 所有测试完成！")
    print("=" * 70)


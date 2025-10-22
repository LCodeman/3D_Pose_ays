"""
实时曲线图组件 - 性能优化版
用于显示传感器数据和姿态角的实时变化
优化：使用blit加速绘制，减少不必要的重绘
"""
import os
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QTimer
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties, fontManager
import numpy as np
from collections import deque
import time

# 配置中文字体（使用项目本地字体文件）
def setup_chinese_font():
    """设置matplotlib支持中文显示 - 使用项目本地字体"""
    # 获取字体文件路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.join(os.path.dirname(current_dir), 'assets')
    font_path = os.path.join(assets_dir, 'STHeiti Light.ttc')
    
    if os.path.exists(font_path):
        # 添加字体到matplotlib
        fontManager.addfont(font_path)
        
        # 设置为默认字体
        plt.rcParams['font.sans-serif'] = ['STHeiti Light', 'STHeiti', 'PingFang SC']
        plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
        
        print(f"✅ 已加载字体: {font_path}")
        return font_path
    else:
        # 备用方案：使用系统字体
        print(f"⚠️ 未找到字体文件: {font_path}，使用系统字体")
        plt.rcParams['font.sans-serif'] = ['PingFang SC', 'STHeiti', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False
        return None

# 初始化字体并保存路径
FONT_PATH = setup_chinese_font()


class RealtimePlotWidget(QWidget):
    """实时曲线图基础类"""
    
    def __init__(self, title, y_labels, colors=None, max_points=100, y_range=None):
        """
        初始化实时曲线图
        
        Args:
            title: 图表标题
            y_labels: Y轴标签列表 (如 ['X', 'Y', 'Z'])
            colors: 颜色列表
            max_points: 最大显示点数
            y_range: Y轴范围 (min, max)，None则自动
        """
        super().__init__()
        
        self.title = title
        self.y_labels = y_labels
        self.max_points = max_points
        self.y_range = y_range
        
        # 默认颜色
        if colors is None:
            self.colors = ['#ff6b6b', '#4ecdc4', '#95e1d3', '#ffd93d', '#a8e6cf']
        else:
            self.colors = colors
        
        # 数据缓冲区
        self.data_buffers = [deque(maxlen=max_points) for _ in y_labels]
        self.time_buffer = deque(maxlen=max_points)
        self.time_counter = 0
        
        # 性能优化
        self.update_timer = None
        self.pending_update = False
        self.last_update_time = 0
        self.min_update_interval = 0.05  # 最小更新间隔50ms（20fps）
        self.background = None  # 用于blit优化
        
        self.init_ui()
        
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        self.setLayout(layout)
        
        # 创建matplotlib图表
        self.figure = Figure(figsize=(4, 3), facecolor='#1a1a1a')
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, facecolor='#2a2a2a')
        
        # 初始化曲线
        self.lines = []
        for i, (label, color) in enumerate(zip(self.y_labels, self.colors)):
            line, = self.ax.plot([], [], color=color, linewidth=1.2, label=label, alpha=0.9)
            self.lines.append(line)
        
        # 设置样式（使用配置好的中文字体）
        self.ax.set_title(self.title, color='#e0e0e0', fontsize=10, fontweight='bold', pad=8)
        self.ax.set_xlabel('时间 (s)', color='#888', fontsize=8)
        self.ax.set_ylabel('数值', color='#888', fontsize=8)
        
        # 设置坐标轴颜色
        self.ax.spines['bottom'].set_color('#3a3a3a')
        self.ax.spines['top'].set_color('#3a3a3a')
        self.ax.spines['left'].set_color('#3a3a3a')
        self.ax.spines['right'].set_color('#3a3a3a')
        self.ax.tick_params(colors='#888', labelsize=7)
        
        # 网格
        self.ax.grid(True, alpha=0.2, color='#555')
        
        # 图例（使用配置好的中文字体）
        # 创建字体属性对象
        if FONT_PATH and os.path.exists(FONT_PATH):
            font_prop = FontProperties(fname=FONT_PATH, size=7)
        else:
            font_prop = FontProperties(size=7)
        
        legend = self.ax.legend(loc='upper right', framealpha=0.3, 
                               facecolor='#2a2a2a', edgecolor='#3a3a3a',
                               prop=font_prop)
        # 设置图例文字颜色
        for text in legend.get_texts():
            text.set_color('#e0e0e0')
        
        # 紧凑布局
        self.figure.tight_layout()
        
        layout.addWidget(self.canvas)
        
        # 性能优化：初始化blit背景
        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        
        # 设置背景色
        self.setStyleSheet("""
            QWidget {
                background-color: #1a1a1a;
            }
        """)
    
    def add_data(self, values):
        """
        添加新数据点（优化版：控制更新频率）
        
        Args:
            values: 数据值列表，长度应与y_labels相同
        """
        if len(values) != len(self.y_labels):
            return
        
        # 添加时间点
        self.time_buffer.append(self.time_counter * 0.1)  # 假设100ms采样
        self.time_counter += 1
        
        # 添加数据
        for i, value in enumerate(values):
            self.data_buffers[i].append(value)
        
        # 性能优化：控制更新频率，避免过于频繁的重绘
        current_time = time.time()
        if current_time - self.last_update_time >= self.min_update_interval:
            self.update_plot()
            self.last_update_time = current_time
        else:
            # 标记有待更新，但不立即执行
            self.pending_update = True
    
    def update_plot(self):
        """更新图表显示（优化版：使用blit加速）"""
        if len(self.time_buffer) == 0:
            return
        
        try:
            # 更新每条曲线数据
            time_array = np.array(self.time_buffer)
            for i, line in enumerate(self.lines):
                data_array = np.array(self.data_buffers[i])
                line.set_data(time_array, data_array)
            
            # 自动调整坐标轴范围
            x_min = max(0, time_array[-1] - 10)
            x_max = time_array[-1] + 0.5
            
            # 只在范围变化时重新绘制背景
            current_xlim = self.ax.get_xlim()
            xlim_changed = abs(current_xlim[0] - x_min) > 0.1 or abs(current_xlim[1] - x_max) > 0.1
            
            if xlim_changed:
                self.ax.set_xlim(x_min, x_max)
            
            if self.y_range is not None:
                if self.ax.get_ylim() != self.y_range:
                    self.ax.set_ylim(self.y_range[0], self.y_range[1])
                    xlim_changed = True
            else:
                # 自动计算Y轴范围
                all_data = []
                for buffer in self.data_buffers:
                    all_data.extend(buffer)
                
                if all_data:
                    y_min = min(all_data)
                    y_max = max(all_data)
                    y_margin = (y_max - y_min) * 0.1 if y_max != y_min else 1.0
                    new_ylim = (y_min - y_margin, y_max + y_margin)
                    current_ylim = self.ax.get_ylim()
                    if abs(current_ylim[0] - new_ylim[0]) > 0.1 or abs(current_ylim[1] - new_ylim[1]) > 0.1:
                        self.ax.set_ylim(new_ylim[0], new_ylim[1])
                        xlim_changed = True
            
            # 性能优化：使用blit只重绘变化的部分
            if xlim_changed or self.background is None:
                # 轴范围变化，需要完全重绘
                self.canvas.draw()
                self.background = self.canvas.copy_from_bbox(self.ax.bbox)
            else:
                # 使用blit快速更新
                if self.background is not None:
                    self.canvas.restore_region(self.background)
                    for line in self.lines:
                        self.ax.draw_artist(line)
                    self.canvas.blit(self.ax.bbox)
                else:
                    self.canvas.draw_idle()
                    
            self.pending_update = False
            
        except Exception as e:
            # 发生错误时回退到普通绘制
            self.canvas.draw_idle()
            self.background = None
    
    def clear(self):
        """清空数据"""
        self.time_buffer.clear()
        for buffer in self.data_buffers:
            buffer.clear()
        self.time_counter = 0
        self.background = None  # 重置背景
        self.update_plot()


class AccelerometerPlot(RealtimePlotWidget):
    """加速度计曲线图"""
    
    def __init__(self):
        super().__init__(
            title='加速度计',
            y_labels=['X轴', 'Y轴', 'Z轴'],
            colors=['#ff6b6b', '#4ecdc4', '#95e1d3'],
            max_points=100,
            y_range=(-2, 2)  # ±2g
        )
        self.ax.set_ylabel('加速度 (g)', color='#888', fontsize=8)


class GyroscopePlot(RealtimePlotWidget):
    """陀螺仪曲线图"""
    
    def __init__(self):
        super().__init__(
            title='陀螺仪',
            y_labels=['X轴', 'Y轴', 'Z轴'],
            colors=['#ff6b6b', '#4ecdc4', '#95e1d3'],
            max_points=100,
            y_range=(-200, 200)  # ±200°/s
        )
        self.ax.set_ylabel('角速度 (°/s)', color='#888', fontsize=8)


class AttitudePlot(RealtimePlotWidget):
    """姿态角曲线图"""
    
    def __init__(self):
        super().__init__(
            title='姿态角',
            y_labels=['Roll', 'Pitch', 'Yaw'],
            colors=['#ff6b6b', '#4ecdc4', '#ffd93d'],
            max_points=100,
            y_range=(-180, 180)  # ±180°
        )
        self.ax.set_ylabel('角度 (°)', color='#888', fontsize=8)


class EncoderAnglePlot(RealtimePlotWidget):
    """MT6701磁编码器角度曲线图"""
    
    def __init__(self):
        super().__init__(
            title='MT6701 磁编码器角度',
            y_labels=['角度'],
            colors=['#ffd93d'],
            max_points=150,
            y_range=(0, 360)  # 0-360°
        )
        self.ax.set_ylabel('角度 (°)', color='#888', fontsize=8)
        
        # 添加水平参考线
        self.ax.axhline(y=0, color='#444', linestyle='--', linewidth=0.5, alpha=0.5)
        self.ax.axhline(y=90, color='#444', linestyle='--', linewidth=0.5, alpha=0.3)
        self.ax.axhline(y=180, color='#444', linestyle='--', linewidth=0.5, alpha=0.5)
        self.ax.axhline(y=270, color='#444', linestyle='--', linewidth=0.5, alpha=0.3)
        self.ax.axhline(y=360, color='#444', linestyle='--', linewidth=0.5, alpha=0.5)


# 测试代码
if __name__ == '__main__':
    import sys
    from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
    import random
    from PyQt5.QtCore import QTimer
    
    app = QApplication(sys.argv)
    
    # 创建测试窗口
    window = QMainWindow()
    window.setWindowTitle('实时曲线图测试')
    window.setGeometry(100, 100, 800, 600)
    
    central = QWidget()
    layout = QVBoxLayout()
    central.setLayout(layout)
    window.setCentralWidget(central)
    
    # 创建3个曲线图
    accel_plot = AccelerometerPlot()
    gyro_plot = GyroscopePlot()
    attitude_plot = AttitudePlot()
    
    layout.addWidget(accel_plot)
    layout.addWidget(gyro_plot)
    layout.addWidget(attitude_plot)
    
    # 模拟数据更新
    def update_data():
        # 模拟传感器数据
        accel_data = [random.uniform(-1, 1) for _ in range(3)]
        gyro_data = [random.uniform(-50, 50) for _ in range(3)]
        attitude_data = [random.uniform(-90, 90) for _ in range(3)]
        
        accel_plot.add_data(accel_data)
        gyro_plot.add_data(gyro_data)
        attitude_plot.add_data(attitude_data)
    
    # 定时器更新
    timer = QTimer()
    timer.timeout.connect(update_data)
    timer.start(100)  # 100ms
    
    window.show()
    sys.exit(app.exec_())


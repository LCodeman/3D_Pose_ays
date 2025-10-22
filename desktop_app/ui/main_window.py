"""
主窗口界面
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QGroupBox, QLineEdit, 
                             QComboBox, QGridLayout, QFrame, QSplitter,
                             QAction, QFileDialog, QMessageBox, QDockWidget, QApplication,
                             QScrollArea, QToolButton, QSlider, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QRect
from PyQt5.QtGui import QFont, QPalette, QColor, QIcon

from renderer import GL3DWidget
from data_fetcher import DataFetcher
from quaternion import AttitudeCalculator, MadgwickQuaternion
from kalman_filter import AdaptiveEKFAttitudeEstimator
from model_loader import load_model
from realtime_plot import AccelerometerPlot, GyroscopePlot, AttitudePlot, EncoderAnglePlot


class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        
        # 姿态计算器 - 新增：扩展卡尔曼滤波(EKF)算法
        self.use_ekf = True  # 默认使用EKF算法
        self.ekf_estimator = AdaptiveEKFAttitudeEstimator()
        
        # 备用：Madgwick算法（用于对比）
        self.attitude_calculator = AttitudeCalculator(alpha=0.98)
        
        # UI更新节流（减少卡顿）- 在数据获取器之前初始化
        self.ui_update_counter = 0
        self.ui_update_skip = 2  # 每3次数据更新一次UI文本（保持3D和曲线图实时）
        
        # 显示算法信息
        if self.use_ekf:
            print("🎯 使用扩展卡尔曼滤波(EKF)算法进行姿态估计")
            print("   ✨ 融合加速度计、陀螺仪、磁力计数据")
            print("   ✨ 自适应传感器信任度调整")
            print("   ✨ 平滑、快速响应、无漂移")
        else:
            print("🎯 使用四元数Madgwick算法进行姿态估计")
            print(f"   Beta参数: {self.attitude_calculator.beta:.3f}")
            print(f"   采样频率: {self.attitude_calculator.sample_freq} Hz")
        
        # 数据获取器
        self.data_fetcher = None
        
        # 原始传感器数据
        self.sensor_data = {
            'accelX': 0.0, 'accelY': 0.0, 'accelZ': 0.0,
            'gyroX': 0.0, 'gyroY': 0.0, 'gyroZ': 0.0,
            'temperature': 0.0,
            'angle': 0.0,
            'angleRaw': 0,
            'angleValid': False
        }
        
        # 实时曲线图（停靠窗口）
        self.accel_plot = None
        self.gyro_plot = None
        self.attitude_plot = None
        self.encoder_plot = None
        self.accel_dock = None
        self.gyro_dock = None
        self.attitude_dock = None
        self.encoder_dock = None
        
        # 控制面板状态
        self.control_panel_expanded = True
        self.control_panel_widget = None
        self.toggle_panel_btn = None
        
        self.init_ui()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle('ICM42688-P + MT6701 3D姿态可视化系统')
        
        # 窗口最大化显示
        self.showMaximized()
        
        # 创建菜单栏
        self.create_menu_bar()
        
        # 设置深色主题
        self.set_dark_theme()
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 左侧：3D渲染窗口
        self.gl_widget = GL3DWidget()
        # 设置最小尺寸，但允许自适应扩展
        self.gl_widget.setMinimumSize(600, 400)
        
        # 右侧：控制面板
        control_panel = self.create_control_panel()
        
        # 使用分割器
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.gl_widget)
        splitter.addWidget(control_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        
        # 设置分割器的大小策略，确保控制面板不会被过度压缩
        splitter.setCollapsible(0, False)  # 3D窗口不可折叠
        splitter.setCollapsible(1, False)  # 控制面板不可折叠
        
        # 设置初始大小（3D窗口占更多空间，控制面板固定宽度）
        # 总宽度按比例分配，控制面板默认330px
        splitter.setSizes([1000, 330])
        
        main_layout.addWidget(splitter)
        
        # 默认显示曲线图（延迟到窗口完全加载后）
        QTimer.singleShot(100, self._show_default_plots)
    
    def _show_default_plots(self):
        """默认显示所有曲线图"""
        # 默认显示所有4个曲线图
        
        # 1. 加速度计曲线图
        self.accel_plot_action.setChecked(True)
        self.toggle_accel_plot(True)
        
        # 2. 陀螺仪曲线图
        self.gyro_plot_action.setChecked(True)
        self.toggle_gyro_plot(True)
        
        # 3. 姿态角曲线图
        self.attitude_plot_action.setChecked(True)
        self.toggle_attitude_plot(True)
        
        # 4. MT6701磁编码器角度曲线图
        self.encoder_plot_action.setChecked(True)
        self.toggle_encoder_plot(True)
        
        print("✅ 已自动打开所有曲线图（加速度计、陀螺仪、姿态角、MT6701角度）")
    
    def _calculate_plot_height(self):
        """
        计算曲线图窗口的合适高度
        
        在Mac上，考虑程序坞高度，将可用高度均分给3个曲线图
        
        Returns:
            int: 推荐的单个曲线图窗口高度
        """
        # 获取屏幕可用区域（排除程序坞、菜单栏等）
        screen = QApplication.primaryScreen()
        available_geometry = screen.availableGeometry()
        
        # 可用高度
        available_height = available_geometry.height()
        
        # 预留空间：菜单栏(约25px) + 标题栏(约30px) + 间距(约50px)
        reserved_height = 105
        
        # 可用于曲线图的高度
        usable_height = available_height - reserved_height
        
        # 均分给3个曲线图窗口
        plot_height = usable_height // 3
        
        # 确保最小高度200px，最大高度400px
        plot_height = max(200, min(400, plot_height))
        
        return plot_height
    
    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 设置菜单栏样式
        menubar.setStyleSheet("""
            QMenuBar {
                background-color: #2a2a2a;
                color: #e0e0e0;
                padding: 5px;
            }
            QMenuBar::item {
                background-color: transparent;
                padding: 5px 15px;
            }
            QMenuBar::item:selected {
                background-color: #4ecdc4;
                color: #1a1a1a;
            }
            QMenu {
                background-color: #2a2a2a;
                color: #e0e0e0;
                border: 1px solid #3a3a3a;
            }
            QMenu::item {
                padding: 8px 30px;
            }
            QMenu::item:selected {
                background-color: #4ecdc4;
                color: #1a1a1a;
            }
        """)
        
        # 文件菜单
        file_menu = menubar.addMenu('文件')
        
        # 导入模型动作
        import_action = QAction('导入3D模型...', self)
        import_action.setShortcut('Ctrl+O')
        import_action.setStatusTip('导入自定义3D模型 (OBJ/STL格式)')
        import_action.triggered.connect(self.import_model)
        file_menu.addAction(import_action)
        
        # 重置模型动作
        reset_model_action = QAction('重置为默认模型', self)
        reset_model_action.setStatusTip('恢复为默认的飞机模型')
        reset_model_action.triggered.connect(self.reset_to_default_model)
        file_menu.addAction(reset_model_action)
        
        file_menu.addSeparator()
        
        # 退出动作
        exit_action = QAction('退出', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.setStatusTip('退出应用程序')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # 视图菜单
        view_menu = menubar.addMenu('视图')
        
        # 实时曲线图子菜单
        plot_menu = view_menu.addMenu('实时曲线图')
        
        # 加速度计曲线图
        self.accel_plot_action = QAction('加速度计', self, checkable=True)
        self.accel_plot_action.setStatusTip('显示/隐藏加速度计实时曲线')
        self.accel_plot_action.triggered.connect(self.toggle_accel_plot)
        plot_menu.addAction(self.accel_plot_action)
        
        # 陀螺仪曲线图
        self.gyro_plot_action = QAction('陀螺仪', self, checkable=True)
        self.gyro_plot_action.setStatusTip('显示/隐藏陀螺仪实时曲线')
        self.gyro_plot_action.triggered.connect(self.toggle_gyro_plot)
        plot_menu.addAction(self.gyro_plot_action)
        
        # 姿态角曲线图
        self.attitude_plot_action = QAction('姿态角', self, checkable=True)
        self.attitude_plot_action.setStatusTip('显示/隐藏姿态角实时曲线')
        self.attitude_plot_action.triggered.connect(self.toggle_attitude_plot)
        plot_menu.addAction(self.attitude_plot_action)
        
        # MT6701角度曲线图
        self.encoder_plot_action = QAction('MT6701角度', self, checkable=True)
        self.encoder_plot_action.setStatusTip('显示/隐藏MT6701磁编码器角度曲线')
        self.encoder_plot_action.triggered.connect(self.toggle_encoder_plot)
        plot_menu.addAction(self.encoder_plot_action)
        
        view_menu.addSeparator()
        
        # 重置视角动作
        reset_camera_action = QAction('重置视角', self)
        reset_camera_action.setShortcut('Ctrl+R')
        reset_camera_action.setStatusTip('重置3D相机视角')
        reset_camera_action.triggered.connect(self.reset_camera)
        view_menu.addAction(reset_camera_action)
        
        # 帮助菜单
        help_menu = menubar.addMenu('帮助')
        
        # 关于动作
        about_action = QAction('关于', self)
        about_action.setStatusTip('关于本软件')
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
        
        # 使用说明动作
        usage_action = QAction('使用说明', self)
        usage_action.setStatusTip('查看使用说明')
        usage_action.triggered.connect(self.show_usage)
        help_menu.addAction(usage_action)
        
        # 模型格式说明动作
        format_action = QAction('支持的3D模型格式', self)
        format_action.setStatusTip('查看支持的3D模型格式说明')
        format_action.triggered.connect(self.show_model_formats)
        help_menu.addAction(format_action)
        
    def create_control_panel(self):
        """创建控制面板（带滚动和收起功能）"""
        # 外层容器
        container = QWidget()
        container_layout = QHBoxLayout()
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(0)
        container.setLayout(container_layout)
        # 设置容器宽度
        container.setMinimumWidth(300)
        container.setMaximumWidth(450)
        
        # 收起/展开按钮（独立显示，始终可见）
        self.toggle_panel_btn = QPushButton('◀')
        self.toggle_panel_btn.setFixedSize(25, 60)
        self.toggle_panel_btn.setToolTip('收起面板')
        self.toggle_panel_btn.clicked.connect(self.toggle_control_panel)
        self.toggle_panel_btn.setStyleSheet("""
            QPushButton {
                background-color: #3a3a3a;
                color: #4ecdc4;
                border: none;
                border-radius: 3px;
                font-size: 14px;
                font-weight: bold;
                margin: 2px;
            }
            QPushButton:hover {
                background-color: #4ecdc4;
                color: #1a1a1a;
            }
        """)
        
        # 可滚动的面板内容
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        # 设置滚动区域宽度
        scroll_area.setMinimumWidth(300)
        scroll_area.setMaximumWidth(450)
        
        # 设置滚动区域样式
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: #1a1a1a;
            }
            QScrollBar:vertical {
                background-color: #2a2a2a;
                width: 10px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background-color: #4ecdc4;
                border-radius: 5px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #45b8ac;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        
        # 面板内容
        panel = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        panel.setLayout(layout)
        # 设置控制面板最小和最大宽度
        panel.setMinimumWidth(300)
        panel.setMaximumWidth(450)
        
        # 标题（不再包含收起按钮，按钮已移到外面）
        title = QLabel('🚀 姿态监控面板')
        title.setFont(QFont('Arial', 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 连接设置
        conn_group = self.create_connection_group()
        layout.addWidget(conn_group)
        
        # 姿态角度显示
        attitude_group = self.create_attitude_group()
        layout.addWidget(attitude_group)
        
        # 传感器数据显示
        sensor_group = self.create_sensor_group()
        layout.addWidget(sensor_group)
        
        # MT6701磁编码器角度显示
        encoder_group = self.create_encoder_group()
        layout.addWidget(encoder_group)
        
        # 温度显示
        temp_group = self.create_temperature_group()
        layout.addWidget(temp_group)
        
        # 显示设置
        display_group = self.create_display_group()
        layout.addWidget(display_group)
        
        # 算法设置（新增）
        algorithm_group = self.create_algorithm_group()
        layout.addWidget(algorithm_group)
        
        # 控制按钮
        button_group = self.create_button_group()
        layout.addWidget(button_group)
        
        # 状态栏
        self.status_label = QLabel('⚪ 未连接')
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #2a2a2a;
                color: #888;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
        """)
        layout.addWidget(self.status_label)
        
        layout.addStretch()
        
        # 设置面板到滚动区域
        scroll_area.setWidget(panel)
        
        # 保存引用
        self.control_panel_widget = scroll_area
        
        # 添加到容器：先添加收起按钮，再添加面板内容
        container_layout.addWidget(self.toggle_panel_btn)
        container_layout.addWidget(scroll_area)
        
        return container
    
    def toggle_control_panel(self):
        """切换控制面板的显示/隐藏状态"""
        if self.control_panel_widget is None:
            return
        
        # 获取容器
        container = self.control_panel_widget.parent()
        
        if self.control_panel_expanded:
            # 收起面板 - 隐藏内容，容器变窄只保留按钮
            self.control_panel_widget.setVisible(False)
            if container:
                container.setMaximumWidth(30)  # 只保留按钮的宽度
                container.setMinimumWidth(30)
            self.toggle_panel_btn.setText('▶')
            self.toggle_panel_btn.setToolTip('展开面板')
            self.control_panel_expanded = False
        else:
            # 展开面板 - 显示内容，恢复容器宽度
            self.control_panel_widget.setVisible(True)
            if container:
                container.setMaximumWidth(450)
                container.setMinimumWidth(300)
            self.toggle_panel_btn.setText('◀')
            self.toggle_panel_btn.setToolTip('收起面板')
            self.control_panel_expanded = True
    
    def create_connection_group(self):
        """创建连接设置组"""
        group = QGroupBox('📡 连接设置')
        layout = QGridLayout()
        
        # ESP32 IP地址
        layout.addWidget(QLabel('设备IP:'), 0, 0)
        self.ip_input = QLineEdit('192.168.3.57')
        layout.addWidget(self.ip_input, 0, 1)
        
        # 连接按钮
        self.connect_btn = QPushButton('🔌 连接设备')
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn, 1, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def create_attitude_group(self):
        """创建姿态角度显示组"""
        group = QGroupBox('📐 姿态角度')
        layout = QGridLayout()
        
        # Roll
        layout.addWidget(QLabel('横滚 (Roll):'), 0, 0)
        self.roll_label = QLabel('0.00°')
        self.roll_label.setStyleSheet('font-weight: bold; color: #ff6b6b;')
        layout.addWidget(self.roll_label, 0, 1)
        
        # Pitch
        layout.addWidget(QLabel('俯仰 (Pitch):'), 1, 0)
        self.pitch_label = QLabel('0.00°')
        self.pitch_label.setStyleSheet('font-weight: bold; color: #4ecdc4;')
        layout.addWidget(self.pitch_label, 1, 1)
        
        # Yaw
        layout.addWidget(QLabel('偏航 (Yaw):'), 2, 0)
        self.yaw_label = QLabel('0.00°')
        self.yaw_label.setStyleSheet('font-weight: bold; color: #95e1d3;')
        layout.addWidget(self.yaw_label, 2, 1)
        
        group.setLayout(layout)
        return group
    
    def create_sensor_group(self):
        """创建传感器数据显示组"""
        group = QGroupBox('📊 传感器数据')
        layout = QGridLayout()
        
        # 加速度
        layout.addWidget(QLabel('加速度 X:'), 0, 0)
        self.accel_x_label = QLabel('0.000 g')
        layout.addWidget(self.accel_x_label, 0, 1)
        
        layout.addWidget(QLabel('加速度 Y:'), 1, 0)
        self.accel_y_label = QLabel('0.000 g')
        layout.addWidget(self.accel_y_label, 1, 1)
        
        layout.addWidget(QLabel('加速度 Z:'), 2, 0)
        self.accel_z_label = QLabel('0.000 g')
        layout.addWidget(self.accel_z_label, 2, 1)
        
        # 陀螺仪
        layout.addWidget(QLabel('陀螺仪 X:'), 3, 0)
        self.gyro_x_label = QLabel('0.00 °/s')
        layout.addWidget(self.gyro_x_label, 3, 1)
        
        layout.addWidget(QLabel('陀螺仪 Y:'), 4, 0)
        self.gyro_y_label = QLabel('0.00 °/s')
        layout.addWidget(self.gyro_y_label, 4, 1)
        
        layout.addWidget(QLabel('陀螺仪 Z:'), 5, 0)
        self.gyro_z_label = QLabel('0.00 °/s')
        layout.addWidget(self.gyro_z_label, 5, 1)
        
        group.setLayout(layout)
        return group
    
    def create_encoder_group(self):
        """创建MT6701磁编码器显示组"""
        group = QGroupBox('🧭 MT6701 磁编码器')
        layout = QGridLayout()
        
        # 角度显示
        layout.addWidget(QLabel('角度:'), 0, 0)
        self.encoder_angle_label = QLabel('--')
        self.encoder_angle_label.setStyleSheet('font-weight: bold; color: #ffd93d; font-size: 16px;')
        layout.addWidget(self.encoder_angle_label, 0, 1)
        
        # 原始值显示
        layout.addWidget(QLabel('原始值:'), 1, 0)
        self.encoder_raw_label = QLabel('-- / 16383')
        self.encoder_raw_label.setStyleSheet('color: #bbb; font-size: 12px;')
        layout.addWidget(self.encoder_raw_label, 1, 1)
        
        # 状态指示
        layout.addWidget(QLabel('状态:'), 2, 0)
        self.encoder_status_label = QLabel('⚪ 未连接')
        self.encoder_status_label.setStyleSheet('font-size: 11px;')
        layout.addWidget(self.encoder_status_label, 2, 1)
        
        group.setLayout(layout)
        return group
    
    def create_temperature_group(self):
        """创建温度显示组"""
        group = QGroupBox('🌡️ 温度')
        layout = QHBoxLayout()
        
        self.temp_label = QLabel('0.00 °C')
        self.temp_label.setFont(QFont('Arial', 14, QFont.Bold))
        self.temp_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.temp_label)
        
        group.setLayout(layout)
        return group
    
    def create_display_group(self):
        """创建显示设置组"""
        group = QGroupBox('🎨 显示设置')
        layout = QGridLayout()
        
        # 模型选择
        layout.addWidget(QLabel('模型:'), 0, 0)
        self.model_combo = QComboBox()
        self.model_combo.addItems(['飞机', '立方体'])
        self.model_combo.currentTextChanged.connect(self.on_model_changed)
        layout.addWidget(self.model_combo, 0, 1)
        
        group.setLayout(layout)
        return group
    
    def create_algorithm_group(self):
        """创建算法设置组（新增）"""
        group = QGroupBox('🧮 算法设置')
        layout = QVBoxLayout()
        
        # 算法选择
        algo_layout = QHBoxLayout()
        algo_layout.addWidget(QLabel('算法:'))
        self.algo_combo = QComboBox()
        self.algo_combo.addItems(['EKF (推荐)', 'Madgwick'])
        self.algo_combo.setCurrentIndex(0)
        self.algo_combo.currentIndexChanged.connect(self.on_algorithm_changed)
        algo_layout.addWidget(self.algo_combo)
        layout.addLayout(algo_layout)
        
        # 磁力计信任度调整
        mag_trust_layout = QVBoxLayout()
        mag_trust_label = QLabel('磁力计信任度:')
        mag_trust_layout.addWidget(mag_trust_label)
        
        mag_slider_layout = QHBoxLayout()
        self.mag_trust_slider = QSlider(Qt.Horizontal)
        self.mag_trust_slider.setMinimum(0)
        self.mag_trust_slider.setMaximum(100)
        self.mag_trust_slider.setValue(100)  # 默认100%信任
        self.mag_trust_slider.setToolTip('调整磁力计的信任度（0-100%）')
        self.mag_trust_slider.valueChanged.connect(self.on_mag_trust_changed)
        mag_slider_layout.addWidget(self.mag_trust_slider)
        
        self.mag_trust_value_label = QLabel('100%')
        self.mag_trust_value_label.setFixedWidth(40)
        self.mag_trust_value_label.setStyleSheet('color: #4ecdc4; font-weight: bold;')
        mag_slider_layout.addWidget(self.mag_trust_value_label)
        
        mag_trust_layout.addLayout(mag_slider_layout)
        layout.addLayout(mag_trust_layout)
        
        # 估计不确定性显示（仅EKF）
        self.uncertainty_label = QLabel('不确定性: --')
        self.uncertainty_label.setStyleSheet('color: #888; font-size: 10px;')
        layout.addWidget(self.uncertainty_label)
        
        group.setLayout(layout)
        return group
    
    def create_button_group(self):
        """创建控制按钮组"""
        group = QGroupBox('⚙️ 控制')
        layout = QVBoxLayout()
        
        # 重置姿态按钮
        reset_btn = QPushButton('🔄 重置姿态')
        reset_btn.clicked.connect(self.reset_attitude)
        layout.addWidget(reset_btn)
        
        group.setLayout(layout)
        return group
    
    def set_dark_theme(self):
        """设置深色主题"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a1a;
            }
            QWidget {
                background-color: #1a1a1a;
                color: #e0e0e0;
            }
            QGroupBox {
                border: 2px solid #3a3a3a;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
                font-weight: bold;
                color: #4ecdc4;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #4ecdc4;
                color: #1a1a1a;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45b8ac;
            }
            QPushButton:pressed {
                background-color: #3da89d;
            }
            QLineEdit {
                background-color: #2a2a2a;
                border: 2px solid #3a3a3a;
                border-radius: 5px;
                padding: 5px;
                color: #e0e0e0;
            }
            QLineEdit:focus {
                border: 2px solid #4ecdc4;
            }
            QComboBox {
                background-color: #2a2a2a;
                border: 2px solid #3a3a3a;
                border-radius: 5px;
                padding: 5px;
                color: #e0e0e0;
            }
            QLabel {
                color: #e0e0e0;
            }
        """)
    
    def toggle_connection(self):
        """切换连接状态"""
        if self.data_fetcher is None or not self.data_fetcher.running:
            # 开始连接
            url = f"http://{self.ip_input.text()}/data"
            self.data_fetcher = DataFetcher(url, interval=100)
            self.data_fetcher.data_received.connect(self.on_data_received)
            self.data_fetcher.connection_status.connect(self.on_connection_status)
            self.data_fetcher.start()
            
            self.connect_btn.setText('🔌 断开连接')
            self.connect_btn.setStyleSheet("""
                QPushButton {
                    background-color: #ff6b6b;
                    color: white;
                }
            """)
            self.ip_input.setEnabled(False)
        else:
            # 断开连接
            self.data_fetcher.stop()
            self.connect_btn.setText('🔌 连接设备')
            self.connect_btn.setStyleSheet('')
            self.ip_input.setEnabled(True)
            self.status_label.setText('⚪ 已断开')
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #2a2a2a;
                    color: #888;
                    padding: 10px;
                    border-radius: 5px;
                    font-weight: bold;
                }
            """)
    
    def on_data_received(self, data):
        """处理接收到的数据 - 优化版，减少UI刷新卡顿"""
        self.sensor_data = data
        
        # UI更新节流：跳过部分UI文本更新，但保持3D和曲线图实时
        self.ui_update_counter += 1
        should_update_text = (self.ui_update_counter % (self.ui_update_skip + 1) == 0)
        
        # 文本标签更新（节流）
        if should_update_text:
            self.accel_x_label.setText(f"{data['accelX']:.3f} g")
            self.accel_y_label.setText(f"{data['accelY']:.3f} g")
            self.accel_z_label.setText(f"{data['accelZ']:.3f} g")
            
            self.gyro_x_label.setText(f"{data['gyroX']:.2f} °/s")
            self.gyro_y_label.setText(f"{data['gyroY']:.2f} °/s")
            self.gyro_z_label.setText(f"{data['gyroZ']:.2f} °/s")
            
            self.temp_label.setText(f"{data['temperature']:.2f} °C")
        
        # 更新MT6701磁编码器数据显示（节流）
        mag_angle = None
        mag_valid = False
        if 'angle' in data and data.get('angleValid', False):
            mag_angle = data['angle']
            mag_valid = True
            if should_update_text:
                self.encoder_angle_label.setText(f"{data['angle']:.2f}°")
                self.encoder_raw_label.setText(f"{data.get('angleRaw', 0)} / 16383")
                self.encoder_status_label.setText('🟢 正常')
                self.encoder_status_label.setStyleSheet('color: #7ed321; font-size: 11px; font-weight: bold;')
        elif 'angle' in data:
            # 数据存在但无效
            if should_update_text:
                self.encoder_angle_label.setText('N/A')
                self.encoder_raw_label.setText('-- / 16383')
                self.encoder_status_label.setText('🔴 无效')
                self.encoder_status_label.setStyleSheet('color: #ff6b6b; font-size: 11px; font-weight: bold;')
        else:
            # 没有MT6701数据（向后兼容旧版固件）
            if should_update_text:
                self.encoder_angle_label.setText('--')
                self.encoder_raw_label.setText('-- / 16383')
                self.encoder_status_label.setText('⚪ 未安装')
                self.encoder_status_label.setStyleSheet('color: #888; font-size: 11px;')
        
        # 根据选择的算法计算姿态（始终执行，保证数据准确）
        if self.use_ekf:
            # 使用EKF算法（融合加速度计、陀螺仪、磁力计）
            roll, pitch, yaw = self.ekf_estimator.update(
                data['accelX'], data['accelY'], data['accelZ'],
                data['gyroX'], data['gyroY'], data['gyroZ'],
                mag_angle=mag_angle, mag_valid=mag_valid,
                dt=0.1
            )
            
            # 更新不确定性显示（节流）
            if should_update_text:
                uncertainty = self.ekf_estimator.get_uncertainty()
                self.uncertainty_label.setText(
                    f"不确定性: ±{uncertainty[0]:.2f}° ±{uncertainty[1]:.2f}° ±{uncertainty[2]:.2f}°"
                )
            
            # 从欧拉角计算四元数（用于3D渲染）
            from quaternion import euler_to_quaternion
            quaternion = euler_to_quaternion(roll, pitch, yaw)
        else:
            # 使用Madgwick算法
            roll, pitch, yaw = self.attitude_calculator.update(
                data['accelX'], data['accelY'], data['accelZ'],
                data['gyroX'], data['gyroY'], data['gyroZ'],
                dt=0.1
            )
            
            # 获取四元数
            quaternion = self.attitude_calculator.get_quaternion()
            if should_update_text:
                self.uncertainty_label.setText('不确定性: --')
        
        # 更新姿态角显示（节流）
        if should_update_text:
            self.roll_label.setText(f"{roll:.2f}°")
            self.pitch_label.setText(f"{pitch:.2f}°")
            self.yaw_label.setText(f"{yaw:.2f}°")
        
        # 更新3D模型（传入四元数以获得更好的旋转效果）
        self.gl_widget.update_attitude(roll, pitch, yaw, quaternion=quaternion)
        
        # 更新实时曲线图
        if self.accel_plot is not None:
            self.accel_plot.add_data([data['accelX'], data['accelY'], data['accelZ']])
        
        if self.gyro_plot is not None:
            self.gyro_plot.add_data([data['gyroX'], data['gyroY'], data['gyroZ']])
        
        if self.attitude_plot is not None:
            self.attitude_plot.add_data([roll, pitch, yaw])
        
        # 更新MT6701角度曲线图
        if self.encoder_plot is not None and mag_valid:
            self.encoder_plot.add_data([mag_angle])
    
    def on_connection_status(self, connected, message):
        """处理连接状态变化"""
        if connected:
            self.status_label.setText(f'🟢 {message}')
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #2d5016;
                    color: #7ed321;
                    padding: 10px;
                    border-radius: 5px;
                    font-weight: bold;
                }
            """)
        else:
            self.status_label.setText(f'🔴 {message}')
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #5a1616;
                    color: #ff6b6b;
                    padding: 10px;
                    border-radius: 5px;
                    font-weight: bold;
                }
            """)
    
    def on_model_changed(self, model_name):
        """模型类型改变"""
        if model_name == '飞机':
            self.gl_widget.set_model_type('airplane')
        else:
            self.gl_widget.set_model_type('cube')
    
    def reset_attitude(self):
        """重置姿态"""
        if self.use_ekf:
            self.ekf_estimator.reset()
        else:
            self.attitude_calculator.reset()
        self.gl_widget.update_attitude(0, 0, 0)
    
    def on_algorithm_changed(self, index):
        """算法选择改变"""
        if index == 0:  # EKF
            self.use_ekf = True
            self.ekf_estimator.reset()
            self.mag_trust_slider.setEnabled(True)
            print("🎯 切换到EKF算法（融合多传感器）")
        else:  # Madgwick
            self.use_ekf = False
            self.attitude_calculator.reset()
            self.mag_trust_slider.setEnabled(False)
            print("🎯 切换到Madgwick算法")
    
    def on_mag_trust_changed(self, value):
        """磁力计信任度调整"""
        trust_factor = value / 100.0
        self.mag_trust_value_label.setText(f"{value}%")
        
        if self.use_ekf:
            self.ekf_estimator.set_mag_trust(trust_factor)
            print(f"⚙️ 磁力计信任度调整为 {value}%")
    
    def import_model(self):
        """导入自定义3D模型"""
        # 打开文件选择对话框
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择3D模型文件",
            "",
            "3D模型文件 (*.obj *.stl);;OBJ文件 (*.obj);;STL文件 (*.stl);;所有文件 (*.*)"
        )
        
        if file_path:
            # 加载模型
            model = load_model(file_path)
            
            if model:
                # 设置自定义模型到渲染器
                self.gl_widget.set_custom_model(model)
                
                # 更新模型选择下拉框
                if self.model_combo.findText('自定义模型') == -1:
                    self.model_combo.addItem('自定义模型')
                self.model_combo.setCurrentText('自定义模型')
                
                QMessageBox.information(
                    self,
                    "导入成功",
                    f"成功导入模型: {model.name}\n\n"
                    f"顶点数: {len(model.vertices)}\n"
                    f"面数: {len(model.faces)}"
                )
            else:
                QMessageBox.warning(
                    self,
                    "导入失败",
                    "无法加载该3D模型文件。\n\n"
                    "请确保文件格式正确，支持的格式：\n"
                    "- OBJ (.obj)\n"
                    "- STL (.stl, ASCII格式)"
                )
    
    def reset_to_default_model(self):
        """重置为默认模型"""
        self.gl_widget.set_custom_model(None)
        self.model_combo.setCurrentText('飞机')
        
        # 移除自定义模型选项
        index = self.model_combo.findText('自定义模型')
        if index != -1:
            self.model_combo.removeItem(index)
    
    def reset_camera(self):
        """重置相机视角"""
        self.gl_widget.reset_camera()
    
    def toggle_accel_plot(self, checked):
        """切换加速度计曲线图显示"""
        if checked:
            if self.accel_dock is None:
                # 计算合适的窗口高度
                plot_height = self._calculate_plot_height()
                
                # 创建停靠窗口
                self.accel_dock = QDockWidget("加速度计曲线", self)
                self.accel_dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | 
                                               Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
                
                # 创建曲线图
                self.accel_plot = AccelerometerPlot()
                self.accel_dock.setWidget(self.accel_plot)
                
                # 设置样式
                self.accel_dock.setStyleSheet("""
                    QDockWidget {
                        color: #e0e0e0;
                        font-weight: bold;
                    }
                    QDockWidget::title {
                        background-color: #2a2a2a;
                        padding: 5px;
                    }
                """)
                
                # 默认停靠在左侧
                self.addDockWidget(Qt.LeftDockWidgetArea, self.accel_dock)
                
                # 设置合适的尺寸
                self.accel_dock.setMinimumWidth(300)
                self.accel_dock.setMaximumWidth(500)
                self.accel_dock.setMinimumHeight(plot_height)
                self.accel_dock.setMaximumHeight(plot_height)
                
                # 连接关闭信号
                self.accel_dock.visibilityChanged.connect(
                    lambda visible: self.accel_plot_action.setChecked(visible)
                )
            else:
                self.accel_dock.show()
        else:
            if self.accel_dock:
                self.accel_dock.hide()
    
    def toggle_gyro_plot(self, checked):
        """切换陀螺仪曲线图显示"""
        if checked:
            if self.gyro_dock is None:
                # 计算合适的窗口高度
                plot_height = self._calculate_plot_height()
                
                # 创建停靠窗口
                self.gyro_dock = QDockWidget("陀螺仪曲线", self)
                self.gyro_dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | 
                                              Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
                
                # 创建曲线图
                self.gyro_plot = GyroscopePlot()
                self.gyro_dock.setWidget(self.gyro_plot)
                
                # 设置样式
                self.gyro_dock.setStyleSheet("""
                    QDockWidget {
                        color: #e0e0e0;
                        font-weight: bold;
                    }
                    QDockWidget::title {
                        background-color: #2a2a2a;
                        padding: 5px;
                    }
                """)
                
                # 默认停靠在左侧
                self.addDockWidget(Qt.LeftDockWidgetArea, self.gyro_dock)
                
                # 如果加速度计窗口存在，则堆叠在下方
                if self.accel_dock and self.accel_dock.isVisible():
                    self.splitDockWidget(self.accel_dock, self.gyro_dock, Qt.Vertical)
                
                # 设置合适的尺寸
                self.gyro_dock.setMinimumWidth(300)
                self.gyro_dock.setMaximumWidth(500)
                self.gyro_dock.setMinimumHeight(plot_height)
                self.gyro_dock.setMaximumHeight(plot_height)
                
                # 连接关闭信号
                self.gyro_dock.visibilityChanged.connect(
                    lambda visible: self.gyro_plot_action.setChecked(visible)
                )
            else:
                self.gyro_dock.show()
        else:
            if self.gyro_dock:
                self.gyro_dock.hide()
    
    def toggle_attitude_plot(self, checked):
        """切换姿态角曲线图显示"""
        if checked:
            if self.attitude_dock is None:
                # 计算合适的窗口高度
                plot_height = self._calculate_plot_height()
                
                # 创建停靠窗口
                self.attitude_dock = QDockWidget("姿态角曲线", self)
                self.attitude_dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | 
                                                  Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
                
                # 创建曲线图
                self.attitude_plot = AttitudePlot()
                self.attitude_dock.setWidget(self.attitude_plot)
                
                # 设置样式
                self.attitude_dock.setStyleSheet("""
                    QDockWidget {
                        color: #e0e0e0;
                        font-weight: bold;
                    }
                    QDockWidget::title {
                        background-color: #2a2a2a;
                        padding: 5px;
                    }
                """)
                
                # 默认停靠在左侧
                self.addDockWidget(Qt.LeftDockWidgetArea, self.attitude_dock)
                
                # 如果陀螺仪窗口存在，则堆叠在下方
                if self.gyro_dock and self.gyro_dock.isVisible():
                    self.splitDockWidget(self.gyro_dock, self.attitude_dock, Qt.Vertical)
                elif self.accel_dock and self.accel_dock.isVisible():
                    self.splitDockWidget(self.accel_dock, self.attitude_dock, Qt.Vertical)
                
                # 设置合适的尺寸
                self.attitude_dock.setMinimumWidth(300)
                self.attitude_dock.setMaximumWidth(500)
                self.attitude_dock.setMinimumHeight(plot_height)
                self.attitude_dock.setMaximumHeight(plot_height)
                
                # 连接关闭信号
                self.attitude_dock.visibilityChanged.connect(
                    lambda visible: self.attitude_plot_action.setChecked(visible)
                )
            else:
                self.attitude_dock.show()
        else:
            if self.attitude_dock:
                self.attitude_dock.hide()
    
    def toggle_encoder_plot(self, checked):
        """切换MT6701角度曲线图显示"""
        if checked:
            if self.encoder_dock is None:
                # 计算合适的窗口高度
                plot_height = self._calculate_plot_height()
                
                # 创建停靠窗口
                self.encoder_dock = QDockWidget("MT6701角度曲线", self)
                self.encoder_dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | 
                                                  Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
                
                # 创建曲线图
                self.encoder_plot = EncoderAnglePlot()
                self.encoder_dock.setWidget(self.encoder_plot)
                
                # 设置样式
                self.encoder_dock.setStyleSheet("""
                    QDockWidget {
                        color: #e0e0e0;
                        font-weight: bold;
                    }
                    QDockWidget::title {
                        background-color: #2a2a2a;
                        padding: 5px;
                    }
                """)
                
                # 默认停靠在左侧
                self.addDockWidget(Qt.LeftDockWidgetArea, self.encoder_dock)
                
                # 根据已有窗口进行堆叠
                if self.attitude_dock and self.attitude_dock.isVisible():
                    self.splitDockWidget(self.attitude_dock, self.encoder_dock, Qt.Vertical)
                elif self.gyro_dock and self.gyro_dock.isVisible():
                    self.splitDockWidget(self.gyro_dock, self.encoder_dock, Qt.Vertical)
                elif self.accel_dock and self.accel_dock.isVisible():
                    self.splitDockWidget(self.accel_dock, self.encoder_dock, Qt.Vertical)
                
                # 设置合适的尺寸
                self.encoder_dock.setMinimumWidth(300)
                self.encoder_dock.setMaximumWidth(500)
                self.encoder_dock.setMinimumHeight(plot_height)
                self.encoder_dock.setMaximumHeight(plot_height)
                
                # 连接关闭信号
                self.encoder_dock.visibilityChanged.connect(
                    lambda visible: self.encoder_plot_action.setChecked(visible)
                )
            else:
                self.encoder_dock.show()
        else:
            if self.encoder_dock:
                self.encoder_dock.hide()
    
    def show_about(self):
        """显示关于对话框"""
        QMessageBox.about(
            self,
            "关于",
            "<h2>ICM42688-P + MT6701 3D姿态可视化系统</h2>"
            "<p><b>版本:</b> 3.0 (扩展卡尔曼滤波 EKF)</p>"
            "<p><b>技术栈:</b> Python + PyQt5 + PyOpenGL</p>"
            "<p><b>核心算法:</b></p>"
            "<ul>"
            "<li>🎯 <b>扩展卡尔曼滤波(EKF)</b> - 多传感器融合姿态估计 ⭐推荐</li>"
            "<li>✨ 融合加速度计、陀螺仪、磁力计数据</li>"
            "<li>⚡ 自适应传感器信任度调整</li>"
            "<li>📐 平滑、快速响应、无漂移</li>"
            "<li>🔄 备选：四元数Madgwick算法</li>"
            "</ul>"
            "<p><b>功能特性:</b></p>"
            "<ul>"
            "<li>实时显示ICM42688-P传感器姿态</li>"
            "<li>支持MT6701磁编码器角度监测</li>"
            "<li>支持自定义3D模型导入(OBJ/STL)</li>"
            "<li>四元数精确旋转渲染</li>"
            "<li>美观的现代化界面</li>"
            "</ul>"
            "<p><b>EKF算法优势:</b></p>"
            "<ul>"
            "<li>融合多传感器：加速度计 + 陀螺仪 + 磁力计</li>"
            "<li>动态调整传感器权重，适应运动状态</li>"
            "<li>实时估计陀螺仪零偏，消除长期漂移</li>"
            "<li>磁力计校正Yaw角，实现绝对方向定位</li>"
            "<li>提供估计不确定性，可靠性评估</li>"
            "</ul>"
            "<p><b>开发时间:</b> 2025</p>"
        )
    
    def show_usage(self):
        """显示使用说明"""
        QMessageBox.information(
            self,
            "使用说明",
            "<h3>基本操作</h3>"
            "<p><b>1. 连接设备:</b></p>"
            "<ul>"
            "<li>输入ESP32-S3的IP地址</li>"
            "<li>点击'连接设备'按钮</li>"
            "</ul>"
            "<p><b>2. 3D视图操作:</b></p>"
            "<ul>"
            "<li>左键拖拽: 旋转视角</li>"
            "<li>鼠标滚轮: 缩放视图</li>"
            "</ul>"
            "<p><b>3. 导入自定义模型:</b></p>"
            "<ul>"
            "<li>菜单栏 → 文件 → 导入3D模型</li>"
            "<li>选择OBJ或STL格式的3D模型</li>"
            "<li>模型会自动缩放并居中</li>"
            "</ul>"
            "<p><b>快捷键:</b></p>"
            "<ul>"
            "<li>Ctrl+O: 导入模型</li>"
            "<li>Ctrl+R: 重置视角</li>"
            "<li>Ctrl+Q: 退出程序</li>"
            "</ul>"
        )
    
    def show_model_formats(self):
        """显示支持的模型格式说明"""
        QMessageBox.information(
            self,
            "支持的3D模型格式",
            "<h3>📐 支持的3D模型格式</h3>"
            "<p>本软件支持以下3D模型格式：</p>"
            "<hr>"
            "<h4>1. OBJ格式 (.obj) ⭐推荐</h4>"
            "<p><b>特点:</b></p>"
            "<ul>"
            "<li>最常用的3D模型格式</li>"
            "<li>文本格式，易于编辑</li>"
            "<li>几乎所有3D软件都支持导出</li>"
            "</ul>"
            "<p><b>获取方式:</b></p>"
            "<ul>"
            "<li>Blender: File → Export → Wavefront (.obj)</li>"
            "<li>3ds Max: Export → OBJ</li>"
            "<li>Maya: Export → OBJ</li>"
            "<li>在线下载: sketchfab.com, turbosquid.com</li>"
            "</ul>"
            "<hr>"
            "<h4>2. STL格式 (.stl)</h4>"
            "<p><b>特点:</b></p>"
            "<ul>"
            "<li>3D打印标准格式</li>"
            "<li>支持ASCII格式（注意：不支持二进制格式）</li>"
            "</ul>"
            "<p><b>注意事项:</b></p>"
            "<ul>"
            "<li>导出时选择<b>ASCII格式</b></li>"
            "<li>模型会自动缩放到合适大小</li>"
            "<li>复杂模型可能加载较慢</li>"
            "</ul>"
            "<hr>"
            "<h4>📥 推荐的免费3D模型网站</h4>"
            "<ul>"
            "<li>Sketchfab.com - 大量免费模型</li>"
            "<li>Free3D.com - 免费3D资源</li>"
            "<li>TurboSquid.com - 有免费模型专区</li>"
            "<li>Thingiverse.com - 3D打印模型(STL)</li>"
            "</ul>"
        )
        
    def closeEvent(self, event):
        """窗口关闭事件"""
        if self.data_fetcher and self.data_fetcher.running:
            self.data_fetcher.stop()
        event.accept()


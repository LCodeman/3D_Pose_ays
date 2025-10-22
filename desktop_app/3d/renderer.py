"""
OpenGL 3D渲染引擎
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QOpenGLWidget
from PyQt5.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

from models import Cube, Axes, Grid, Airplane


class GL3DWidget(QOpenGLWidget):
    """OpenGL 3D渲染窗口"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 模型选择：'cube', 'airplane', 'custom'
        self.model_type = 'airplane'
        
        # 3D模型
        self.cube = Cube(1.5)
        self.airplane = Airplane(1.0)
        self.custom_model = None  # 自定义模型
        self.axes = Axes(2.5)
        self.grid = Grid(10, 20)
        
        # 姿态数据
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 四元数姿态（用于更精确的旋转）
        self.use_quaternion = True
        self.quaternion = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
        
        # 相机参数
        self.camera_distance = 8.0
        self.camera_rotation_x = 30.0
        self.camera_rotation_y = 45.0
        
        # 鼠标交互
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        self.mouse_pressed = False
        
        # 启用鼠标追踪
        self.setMouseTracking(False)
        
    def initializeGL(self):
        """初始化OpenGL"""
        # 设置背景颜色（深色背景）
        glClearColor(0.1, 0.1, 0.15, 1.0)
        
        # 启用深度测试
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        
        # 启用光照
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # 设置光源
        light_position = [5.0, 5.0, 5.0, 1.0]
        light_ambient = [0.3, 0.3, 0.3, 1.0]
        light_diffuse = [0.8, 0.8, 0.8, 1.0]
        
        glLightfv(GL_LIGHT0, GL_POSITION, light_position)
        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse)
        
        # 启用平滑着色
        glShadeModel(GL_SMOOTH)
        
        # 启用抗锯齿
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
    def resizeGL(self, w, h):
        """窗口大小改变时调用"""
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        
        aspect = w / h if h != 0 else 1
        gluPerspective(45.0, aspect, 0.1, 100.0)
        
        glMatrixMode(GL_MODELVIEW)
        
    def paintGL(self):
        """绘制场景"""
        # 清除缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # 设置相机
        glTranslatef(0.0, 0.0, -self.camera_distance)
        glRotatef(self.camera_rotation_x, 1.0, 0.0, 0.0)
        glRotatef(self.camera_rotation_y, 0.0, 1.0, 0.0)
        
        # 绘制网格（地面）
        glDisable(GL_LIGHTING)
        self.grid.draw()
        glEnable(GL_LIGHTING)
        
        # 绘制坐标轴
        glDisable(GL_LIGHTING)
        self.axes.draw()
        glEnable(GL_LIGHTING)
        
        # 绘制传感器模型（应用姿态旋转）
        glPushMatrix()
        
        if self.use_quaternion and self.quaternion is not None:
            # 使用四元数旋转（推荐，避免万向节锁）
            self._apply_quaternion_rotation()
        else:
            # 使用欧拉角旋转（传统方法，ZYX顺序）
            glRotatef(self.yaw, 0.0, 0.0, 1.0)      # Z轴 - Yaw
            glRotatef(self.pitch, 0.0, 1.0, 0.0)    # Y轴 - Pitch
            glRotatef(self.roll, 1.0, 0.0, 0.0)     # X轴 - Roll
        
        # 绘制选择的模型
        if self.model_type == 'cube':
            self.cube.draw()
        elif self.model_type == 'custom' and self.custom_model:
            self.custom_model.draw()
        else:
            self.airplane.draw()
        
        glPopMatrix()
        
    def update_attitude(self, roll, pitch, yaw, quaternion=None):
        """
        更新姿态角
        
        Args:
            roll: 横滚角(度)
            pitch: 俯仰角(度)
            yaw: 偏航角(度)
            quaternion: 四元数 [w, x, y, z]，如果提供则优先使用
        """
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        
        if quaternion is not None:
            self.quaternion = quaternion
        
        self.update()  # 触发重绘
    
    def _apply_quaternion_rotation(self):
        """
        应用四元数旋转到OpenGL
        
        使用四元数计算旋转矩阵，比欧拉角更精确
        """
        w, x, y, z = self.quaternion
        
        # 四元数到旋转矩阵的转换
        # 优化算法，避免重复计算
        xx = x * x
        xy = x * y
        xz = x * z
        xw = x * w
        
        yy = y * y
        yz = y * z
        yw = y * w
        
        zz = z * z
        zw = z * w
        
        # 构建3x3旋转矩阵（OpenGL使用列主序）
        matrix = [
            1.0 - 2.0 * (yy + zz), 2.0 * (xy + zw),       2.0 * (xz - yw),       0.0,
            2.0 * (xy - zw),       1.0 - 2.0 * (xx + zz), 2.0 * (yz + xw),       0.0,
            2.0 * (xz + yw),       2.0 * (yz - xw),       1.0 - 2.0 * (xx + yy), 0.0,
            0.0,                   0.0,                   0.0,                   1.0
        ]
        
        # 应用矩阵变换
        glMultMatrixf(matrix)
    
    def set_use_quaternion(self, use_quaternion):
        """
        设置是否使用四元数旋转
        
        Args:
            use_quaternion: True使用四元数，False使用欧拉角
        """
        self.use_quaternion = use_quaternion
        self.update()
    
    def set_model_type(self, model_type):
        """
        切换模型类型
        
        Args:
            model_type: 'cube', 'airplane', 'custom'
        """
        self.model_type = model_type
        self.update()
    
    def set_custom_model(self, model):
        """
        设置自定义模型
        
        Args:
            model: CustomModel对象，如果为None则恢复默认模型
        """
        if model is None:
            self.custom_model = None
            self.model_type = 'airplane'
        else:
            self.custom_model = model
            self.model_type = 'custom'
        self.update()
    
    def reset_camera(self):
        """重置相机到默认位置"""
        self.camera_distance = 8.0
        self.camera_rotation_x = 30.0
        self.camera_rotation_y = 45.0
        self.update()
    
    def mousePressEvent(self, event):
        """鼠标按下事件"""
        self.mouse_pressed = True
        self.last_mouse_x = event.x()
        self.last_mouse_y = event.y()
    
    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        self.mouse_pressed = False
    
    def mouseMoveEvent(self, event):
        """鼠标移动事件 - 用于旋转视角"""
        if self.mouse_pressed:
            dx = event.x() - self.last_mouse_x
            dy = event.y() - self.last_mouse_y
            
            self.camera_rotation_y += dx * 0.5
            self.camera_rotation_x += dy * 0.5
            
            # 限制X轴旋转角度
            self.camera_rotation_x = max(-89, min(89, self.camera_rotation_x))
            
            self.last_mouse_x = event.x()
            self.last_mouse_y = event.y()
            
            self.update()
    
    def wheelEvent(self, event):
        """鼠标滚轮事件 - 用于缩放"""
        delta = event.angleDelta().y()
        self.camera_distance -= delta * 0.01
        
        # 限制缩放范围
        self.camera_distance = max(3.0, min(20.0, self.camera_distance))
        
        self.update()


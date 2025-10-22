"""
3D模型定义 - 用于OpenGL渲染的几何体
"""
from OpenGL.GL import *
from OpenGL.GLU import *
import math


class Cube:
    """立方体模型 - 代表传感器"""
    
    def __init__(self, size=1.0):
        """
        初始化立方体
        
        Args:
            size: 立方体边长
        """
        self.size = size
        
    def draw(self):
        """绘制立方体"""
        s = self.size / 2.0
        
        glBegin(GL_QUADS)
        
        # 前面 (红色)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(-s, -s, s)
        glVertex3f(s, -s, s)
        glVertex3f(s, s, s)
        glVertex3f(-s, s, s)
        
        # 后面 (黄色)
        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(-s, -s, -s)
        glVertex3f(-s, s, -s)
        glVertex3f(s, s, -s)
        glVertex3f(s, -s, -s)
        
        # 上面 (绿色)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(-s, s, -s)
        glVertex3f(-s, s, s)
        glVertex3f(s, s, s)
        glVertex3f(s, s, -s)
        
        # 下面 (橙色)
        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(-s, -s, -s)
        glVertex3f(s, -s, -s)
        glVertex3f(s, -s, s)
        glVertex3f(-s, -s, s)
        
        # 右面 (蓝色)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(s, -s, -s)
        glVertex3f(s, s, -s)
        glVertex3f(s, s, s)
        glVertex3f(s, -s, s)
        
        # 左面 (紫色)
        glColor3f(0.5, 0.0, 0.5)
        glVertex3f(-s, -s, -s)
        glVertex3f(-s, -s, s)
        glVertex3f(-s, s, s)
        glVertex3f(-s, s, -s)
        
        glEnd()
        
        # 绘制边框（黑色）
        glColor3f(0.0, 0.0, 0.0)
        glLineWidth(2.0)
        
        glBegin(GL_LINES)
        # 前面的4条边
        glVertex3f(-s, -s, s); glVertex3f(s, -s, s)
        glVertex3f(s, -s, s); glVertex3f(s, s, s)
        glVertex3f(s, s, s); glVertex3f(-s, s, s)
        glVertex3f(-s, s, s); glVertex3f(-s, -s, s)
        
        # 后面的4条边
        glVertex3f(-s, -s, -s); glVertex3f(s, -s, -s)
        glVertex3f(s, -s, -s); glVertex3f(s, s, -s)
        glVertex3f(s, s, -s); glVertex3f(-s, s, -s)
        glVertex3f(-s, s, -s); glVertex3f(-s, -s, -s)
        
        # 连接前后的4条边
        glVertex3f(-s, -s, s); glVertex3f(-s, -s, -s)
        glVertex3f(s, -s, s); glVertex3f(s, -s, -s)
        glVertex3f(s, s, s); glVertex3f(s, s, -s)
        glVertex3f(-s, s, s); glVertex3f(-s, s, -s)
        glEnd()


class Axes:
    """坐标轴 - XYZ轴"""
    
    def __init__(self, length=2.0):
        """
        初始化坐标轴
        
        Args:
            length: 轴长度
        """
        self.length = length
        
    def draw(self):
        """绘制坐标轴"""
        glLineWidth(3.0)
        glBegin(GL_LINES)
        
        # X轴 (红色)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(self.length, 0.0, 0.0)
        
        # Y轴 (绿色)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, self.length, 0.0)
        
        # Z轴 (蓝色)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, self.length)
        
        glEnd()
        
        # 绘制箭头
        self._draw_arrow_head(self.length, 0, 0, 1.0, 0.0, 0.0)  # X轴
        self._draw_arrow_head(0, self.length, 0, 0.0, 1.0, 0.0)  # Y轴
        self._draw_arrow_head(0, 0, self.length, 0.0, 0.0, 1.0)  # Z轴
    
    def _draw_arrow_head(self, x, y, z, r, g, b):
        """绘制箭头头部"""
        glColor3f(r, g, b)
        arrow_size = 0.15
        
        glPushMatrix()
        glTranslatef(x, y, z)
        
        if x != 0:  # X轴
            glRotatef(90, 0, 1, 0)
        elif y != 0:  # Y轴
            glRotatef(-90, 1, 0, 0)
        # Z轴不需要旋转
        
        # 绘制圆锥
        quadric = gluNewQuadric()
        gluCylinder(quadric, arrow_size, 0.0, arrow_size * 2, 8, 1)
        gluDeleteQuadric(quadric)
        
        glPopMatrix()


class Grid:
    """网格地面"""
    
    def __init__(self, size=10, divisions=20):
        """
        初始化网格
        
        Args:
            size: 网格大小
            divisions: 网格划分数
        """
        self.size = size
        self.divisions = divisions
        
    def draw(self):
        """绘制网格"""
        glColor3f(0.5, 0.5, 0.5)
        glLineWidth(1.0)
        
        step = self.size / self.divisions
        half_size = self.size / 2.0
        
        glBegin(GL_LINES)
        
        # 绘制平行于X轴的线
        for i in range(self.divisions + 1):
            z = -half_size + i * step
            glVertex3f(-half_size, 0, z)
            glVertex3f(half_size, 0, z)
        
        # 绘制平行于Z轴的线
        for i in range(self.divisions + 1):
            x = -half_size + i * step
            glVertex3f(x, 0, -half_size)
            glVertex3f(x, 0, half_size)
        
        glEnd()


class Airplane:
    """波音737风格飞机模型 - 简化优化版"""
    
    def __init__(self, scale=1.0):
        """
        初始化飞机模型
        
        Args:
            scale: 缩放比例
        """
        self.scale = scale
        
    def draw(self):
        """绘制波音737风格飞机"""
        s = self.scale
        
        # 1. 机身和机头
        self._draw_fuselage(s)
        
        # 2. 驾驶舱窗户
        self._draw_windows(s)
        
        # 3. 主机翼
        self._draw_wings(s)
        
        # 4. 发动机
        self._draw_engines(s)
        
        # 5. 尾翼系统
        self._draw_tail(s)
    
    def _draw_fuselage(self, s):
        """绘制机身和机头（一体化）"""
        # 主机身（白色圆柱）
        glColor3f(0.92, 0.92, 0.92)
        glPushMatrix()
        glTranslatef(0, 0, -0.9 * s)
        
        quadric = gluNewQuadric()
        gluQuadricNormals(quadric, GLU_SMOOTH)
        
        # 主机身圆柱
        gluCylinder(quadric, 0.16 * s, 0.16 * s, 1.7 * s, 24, 1)
        
        # 后端封闭
        gluDisk(quadric, 0, 0.16 * s, 24, 1)
        
        # 机身前部收窄
        glTranslatef(0, 0, 1.7 * s)
        gluCylinder(quadric, 0.16 * s, 0.12 * s, 0.25 * s, 24, 1)
        
        # 机头圆锥
        glTranslatef(0, 0, 0.25 * s)
        gluCylinder(quadric, 0.12 * s, 0.01 * s, 0.3 * s, 20, 1)
        
        gluDeleteQuadric(quadric)
        glPopMatrix()
        
        # 蓝色装饰条
        glColor3f(0.1, 0.3, 0.7)
        glPushMatrix()
        glTranslatef(0, 0.165 * s, -0.5 * s)
        quadric = gluNewQuadric()
        gluQuadricNormals(quadric, GLU_SMOOTH)
        gluCylinder(quadric, 0.17 * s, 0.17 * s, 1.3 * s, 24, 1)
        gluDeleteQuadric(quadric)
        glPopMatrix()
    
    def _draw_windows(self, s):
        """绘制驾驶舱窗户"""
        glColor3f(0.05, 0.05, 0.15)
        
        # 左窗
        glPushMatrix()
        glTranslatef(-0.08 * s, 0.05 * s, 1.15 * s)
        glScalef(0.06 * s, 0.04 * s, 0.08 * s)
        self._draw_box()
        glPopMatrix()
        
        # 右窗
        glPushMatrix()
        glTranslatef(0.08 * s, 0.05 * s, 1.15 * s)
        glScalef(0.06 * s, 0.04 * s, 0.08 * s)
        self._draw_box()
        glPopMatrix()
    
    def _draw_wings(self, s):
        """绘制主机翼"""
        glColor3f(0.88, 0.88, 0.88)
        
        # 左翼
        glPushMatrix()
        glTranslatef(-0.16 * s, -0.04 * s, 0.1 * s)
        glRotatef(20, 0, 1, 0)  # 后掠角
        glScalef(1.0 * s, 0.05 * s, 0.35 * s)
        self._draw_box()
        glPopMatrix()
        
        # 右翼
        glPushMatrix()
        glTranslatef(0.16 * s, -0.04 * s, 0.1 * s)
        glRotatef(-20, 0, 1, 0)  # 后掠角
        glScalef(1.0 * s, 0.05 * s, 0.35 * s)
        self._draw_box()
        glPopMatrix()
        
        # 翼尖小翼（左）
        glColor3f(0.85, 0.85, 0.85)
        glPushMatrix()
        glTranslatef(-1.15 * s, 0.05 * s, 0.15 * s)
        glRotatef(80, 1, 0, 0)
        glScalef(0.03 * s, 0.18 * s, 0.06 * s)
        self._draw_box()
        glPopMatrix()
        
        # 翼尖小翼（右）
        glPushMatrix()
        glTranslatef(1.15 * s, 0.05 * s, 0.15 * s)
        glRotatef(80, 1, 0, 0)
        glScalef(0.03 * s, 0.18 * s, 0.06 * s)
        self._draw_box()
        glPopMatrix()
    
    def _draw_engines(self, s):
        """绘制发动机吊舱"""
        engine_x = [-0.5 * s, 0.5 * s]
        
        glColor3f(0.78, 0.78, 0.82)
        for x in engine_x:
            glPushMatrix()
            glTranslatef(x, -0.14 * s, 0.05 * s)
            
            quadric = gluNewQuadric()
            gluQuadricNormals(quadric, GLU_SMOOTH)
            
            # 发动机外壳
            gluCylinder(quadric, 0.09 * s, 0.09 * s, 0.42 * s, 20, 1)
            
            # 进气口
            glColor3f(0.15, 0.15, 0.2)
            gluDisk(quadric, 0, 0.09 * s, 20, 1)
            
            # 尾喷口
            glColor3f(0.25, 0.25, 0.3)
            glTranslatef(0, 0, 0.42 * s)
            gluCylinder(quadric, 0.09 * s, 0.08 * s, 0.05 * s, 20, 1)
            
            gluDeleteQuadric(quadric)
            glColor3f(0.78, 0.78, 0.82)  # 恢复颜色
            glPopMatrix()
    
    def _draw_tail(self, s):
        """绘制尾翼系统"""
        glColor3f(0.88, 0.88, 0.88)
        
        # 垂直尾翼
        glPushMatrix()
        glTranslatef(0, 0.22 * s, -0.7 * s)
        glScalef(0.03 * s, 0.45 * s, 0.3 * s)
        self._draw_box()
        glPopMatrix()
        
        # 红色装饰
        glColor3f(0.8, 0.1, 0.1)
        glPushMatrix()
        glTranslatef(0, 0.28 * s, -0.58 * s)
        glScalef(0.031 * s, 0.08 * s, 0.06 * s)
        self._draw_box()
        glPopMatrix()
        
        # 水平尾翼（左）
        glColor3f(0.88, 0.88, 0.88)
        glPushMatrix()
        glTranslatef(-0.22 * s, 0.18 * s, -0.75 * s)
        glScalef(0.45 * s, 0.03 * s, 0.22 * s)
        self._draw_box()
        glPopMatrix()
        
        # 水平尾翼（右）
        glPushMatrix()
        glTranslatef(0.22 * s, 0.18 * s, -0.75 * s)
        glScalef(0.45 * s, 0.03 * s, 0.22 * s)
        self._draw_box()
        glPopMatrix()
    
    def _draw_box(self):
        """绘制单位立方体"""
        glBegin(GL_QUADS)
        
        # 各个面
        vertices = [
            [[-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5]],
            [[-0.5, -0.5, 0.5], [-0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [0.5, -0.5, 0.5]],
            [[-0.5, -0.5, -0.5], [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [0.5, -0.5, -0.5]],
            [[-0.5, 0.5, -0.5], [0.5, 0.5, -0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5]],
            [[-0.5, -0.5, -0.5], [-0.5, 0.5, -0.5], [-0.5, 0.5, 0.5], [-0.5, -0.5, 0.5]],
            [[0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 0.5, -0.5]]
        ]
        
        for face in vertices:
            for vertex in face:
                glVertex3fv(vertex)
        
        glEnd()


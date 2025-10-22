"""
3D模型加载器 - 支持加载外部3D模型文件
"""
import os
import numpy as np
from OpenGL.GL import *


class CustomModel:
    """自定义3D模型类"""
    
    def __init__(self):
        self.vertices = []  # 顶点列表
        self.faces = []     # 面列表
        self.normals = []   # 法线列表
        self.colors = []    # 颜色列表
        self.name = "Custom Model"
        self.scale = 1.0
        
    def load_from_obj(self, filepath):
        """
        从OBJ文件加载模型
        
        Args:
            filepath: OBJ文件路径
        
        Returns:
            bool: 是否加载成功
        """
        try:
            with open(filepath, 'r') as f:
                temp_vertices = []
                temp_normals = []
                
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    parts = line.split()
                    if not parts:
                        continue
                    
                    # 顶点坐标
                    if parts[0] == 'v':
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        temp_vertices.append([x, y, z])
                    
                    # 法线
                    elif parts[0] == 'vn':
                        nx, ny, nz = float(parts[1]), float(parts[2]), float(parts[3])
                        temp_normals.append([nx, ny, nz])
                    
                    # 面（三角形或四边形）
                    elif parts[0] == 'f':
                        face = []
                        for vertex_data in parts[1:]:
                            # 支持格式: v, v/vt, v/vt/vn, v//vn
                            indices = vertex_data.split('/')
                            vertex_index = int(indices[0]) - 1  # OBJ索引从1开始
                            face.append(vertex_index)
                        
                        # 如果是四边形，分割成两个三角形
                        if len(face) == 4:
                            self.faces.append([face[0], face[1], face[2]])
                            self.faces.append([face[0], face[2], face[3]])
                        else:
                            self.faces.append(face)
                
                self.vertices = temp_vertices
                self.name = os.path.basename(filepath)
                
                # 如果没有法线，自动计算
                if not temp_normals:
                    self._calculate_normals()
                else:
                    self.normals = temp_normals
                
                # 自动缩放模型到合适大小
                self._auto_scale()
                
                print(f"✅ 成功加载模型: {self.name}")
                print(f"   顶点数: {len(self.vertices)}")
                print(f"   面数: {len(self.faces)}")
                
                return True
                
        except Exception as e:
            print(f"❌ 加载OBJ文件失败: {str(e)}")
            return False
    
    def load_from_stl(self, filepath):
        """
        从STL文件加载模型（ASCII格式）
        
        Args:
            filepath: STL文件路径
        
        Returns:
            bool: 是否加载成功
        """
        try:
            with open(filepath, 'r') as f:
                content = f.read()
                
                # 检查是否为ASCII格式
                if not content.startswith('solid'):
                    print("❌ 目前仅支持ASCII格式的STL文件")
                    return False
                
                lines = content.split('\n')
                temp_vertices = []
                current_face = []
                
                for line in lines:
                    line = line.strip()
                    
                    if line.startswith('vertex'):
                        parts = line.split()
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        temp_vertices.append([x, y, z])
                        current_face.append(len(temp_vertices) - 1)
                        
                        if len(current_face) == 3:
                            self.faces.append(current_face[:])
                            current_face = []
                
                self.vertices = temp_vertices
                self.name = os.path.basename(filepath)
                
                # 自动计算法线
                self._calculate_normals()
                
                # 自动缩放
                self._auto_scale()
                
                print(f"✅ 成功加载模型: {self.name}")
                print(f"   顶点数: {len(self.vertices)}")
                print(f"   面数: {len(self.faces)}")
                
                return True
                
        except Exception as e:
            print(f"❌ 加载STL文件失败: {str(e)}")
            return False
    
    def _calculate_normals(self):
        """自动计算面法线"""
        self.normals = []
        
        for face in self.faces:
            if len(face) < 3:
                continue
            
            # 获取三角形的三个顶点
            v0 = np.array(self.vertices[face[0]])
            v1 = np.array(self.vertices[face[1]])
            v2 = np.array(self.vertices[face[2]])
            
            # 计算两条边
            edge1 = v1 - v0
            edge2 = v2 - v0
            
            # 叉乘得到法线
            normal = np.cross(edge1, edge2)
            
            # 归一化
            length = np.linalg.norm(normal)
            if length > 0:
                normal = normal / length
            
            self.normals.append(normal.tolist())
    
    def _auto_scale(self):
        """自动缩放模型到合适大小（约2单位）"""
        if not self.vertices:
            return
        
        vertices_array = np.array(self.vertices)
        
        # 计算边界框
        min_coords = vertices_array.min(axis=0)
        max_coords = vertices_array.max(axis=0)
        
        # 计算最大尺寸
        dimensions = max_coords - min_coords
        max_dimension = dimensions.max()
        
        if max_dimension > 0:
            # 缩放到目标大小（例如2单位）
            target_size = 2.0
            self.scale = target_size / max_dimension
            
            # 缩放所有顶点
            self.vertices = (vertices_array * self.scale).tolist()
            
            # 居中模型
            center = (np.array(self.vertices).max(axis=0) + 
                     np.array(self.vertices).min(axis=0)) / 2
            self.vertices = (np.array(self.vertices) - center).tolist()
    
    def draw(self):
        """绘制模型"""
        if not self.vertices or not self.faces:
            return
        
        glBegin(GL_TRIANGLES)
        
        for i, face in enumerate(self.faces):
            # 设置法线（如果有）
            if i < len(self.normals):
                glNormal3fv(self.normals[i])
            
            # 设置颜色（灰色金属质感）
            glColor3f(0.7, 0.7, 0.75)
            
            # 绘制三角形的三个顶点
            for vertex_index in face:
                if vertex_index < len(self.vertices):
                    glVertex3fv(self.vertices[vertex_index])
        
        glEnd()
        
        # 绘制线框（可选，增强立体感）
        glDisable(GL_LIGHTING)
        glColor3f(0.2, 0.2, 0.2)
        glLineWidth(1.0)
        
        glBegin(GL_LINES)
        for face in self.faces:
            for i in range(len(face)):
                v1 = face[i]
                v2 = face[(i + 1) % len(face)]
                if v1 < len(self.vertices) and v2 < len(self.vertices):
                    glVertex3fv(self.vertices[v1])
                    glVertex3fv(self.vertices[v2])
        glEnd()
        
        glEnable(GL_LIGHTING)


def load_model(filepath):
    """
    根据文件扩展名自动选择加载器
    
    Args:
        filepath: 模型文件路径
    
    Returns:
        CustomModel or None: 加载的模型对象
    """
    ext = os.path.splitext(filepath)[1].lower()
    
    model = CustomModel()
    
    if ext == '.obj':
        if model.load_from_obj(filepath):
            return model
    elif ext == '.stl':
        if model.load_from_stl(filepath):
            return model
    else:
        print(f"❌ 不支持的文件格式: {ext}")
        print("   支持的格式: .obj, .stl")
        return None
    
    return None


# 测试代码
if __name__ == '__main__':
    print("3D模型加载器测试")
    print("-" * 50)
    
    # 这里可以测试加载模型
    # model = load_model("test.obj")
    # if model:
    #     print("模型加载成功!")


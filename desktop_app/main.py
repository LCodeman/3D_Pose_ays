#!/usr/bin/env python3
"""
ICM42688-P 3D姿态可视化系统
主程序入口

使用方法：
    python main.py
"""
import sys
import os

# 添加路径到sys.path（必须在导入之前）
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ui'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '3d'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'network'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'utils'))

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

# 导入主窗口类
from main_window import MainWindow  # noqa: E402


def main():
    """主函数"""
    # 创建应用
    app = QApplication(sys.argv)
    app.setApplicationName('3D姿态可视化')
    
    # 设置高DPI支持
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    # 创建主窗口（已在MainWindow中设置为最大化）
    window = MainWindow()
    
    print("=" * 60)
    print(" ICM42688-P 3D姿态可视化系统")
    print("=" * 60)
    print("\n使用说明:")
    print("  1. 在右侧面板输入ESP32-S3的IP地址")
    print("  2. 点击'连接设备'按钮")
    print("  3. 观察3D空间中的实时姿态")
    print("\n鼠标操作:")
    print("  - 左键拖拽: 旋转视角")
    print("  - 滚轮: 缩放视图")
    print("\n功能:")
    print("  - 实时显示传感器姿态")
    print("  - 切换飞机/立方体模型")
    print("  - 显示传感器原始数据")
    print("  - 姿态角度计算(Roll/Pitch/Yaw)")
    print("=" * 60)
    print()
    
    # 运行应用
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()


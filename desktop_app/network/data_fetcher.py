"""
数据获取模块 - 从ESP32-S3获取传感器数据
高性能版本：异步处理、智能重试、减少卡顿
"""
import sys
import os
import requests
import json
from PyQt5.QtCore import QThread, pyqtSignal, QMutex
import time
from collections import deque
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry


class DataFetcher(QThread):
    """后台线程：从ESP32获取传感器数据"""
    
    # 信号：传输数据
    data_received = pyqtSignal(dict)
    connection_status = pyqtSignal(bool, str)  # (是否连接, 状态消息)
    
    def __init__(self, url="http://192.168.3.57/data", interval=100):
        """
        初始化数据获取器
        
        Args:
            url: ESP32的数据API地址
            interval: 数据获取间隔(毫秒)
        """
        super().__init__()
        self.url = url
        self.interval = interval / 1000.0  # 转换为秒
        self.running = False
        self.timeout = 0.5  # 请求超时时间(秒) - 减少到0.5秒，避免长时间卡顿
        
        # 性能优化：减少状态更新频率
        self.last_status_time = 0
        self.status_update_interval = 2.0  # 最多每2秒更新一次状态
        
        # 创建优化的requests session（连接池复用）
        self.session = requests.Session()
        
        # 配置连接池和重试策略
        retry_strategy = Retry(
            total=0,  # 不自动重试（我们手动控制）
            backoff_factor=0,
            status_forcelist=[]
        )
        adapter = HTTPAdapter(
            pool_connections=1,
            pool_maxsize=1,
            max_retries=retry_strategy,
            pool_block=False
        )
        self.session.mount("http://", adapter)
        
        # 性能统计
        self.success_count = 0
        self.error_count = 0
        self.last_success_time = 0
        
    def run(self):
        """线程运行函数 - 优化版，减少卡顿"""
        self.running = True
        consecutive_errors = 0
        max_errors = 3  # 减少最大错误次数，更快恢复
        skip_count = 0  # 跳过次数，用于降级策略
        
        while self.running:
            request_start = time.time()
            
            try:
                # 使用session发送请求（连接复用，性能更好）
                response = self.session.get(
                    self.url, 
                    timeout=self.timeout,
                    headers={'Connection': 'keep-alive'}  # 保持连接
                )
                
                if response.status_code == 200:
                    # 解析JSON数据
                    data = response.json()
                    
                    # 验证数据完整性
                    required_keys = ['accelX', 'accelY', 'accelZ', 
                                   'gyroX', 'gyroY', 'gyroZ', 'temperature']
                    
                    if all(key in data for key in required_keys):
                        # 发送数据信号
                        self.data_received.emit(data)
                        
                        # 成功统计
                        self.success_count += 1
                        self.last_success_time = time.time()
                        
                        # 重置错误计数和跳过次数
                        if consecutive_errors > 0 or skip_count > 0:
                            consecutive_errors = 0
                            skip_count = 0
                            current_time = time.time()
                            if current_time - self.last_status_time >= self.status_update_interval:
                                self.connection_status.emit(True, "✓ 连接正常")
                                self.last_status_time = current_time
                    else:
                        raise ValueError("数据格式不完整")
                else:
                    raise Exception(f"HTTP {response.status_code}")
                    
            except requests.exceptions.Timeout:
                consecutive_errors += 1
                self.error_count += 1
                
                # 超时后不立即报错，静默跳过
                if consecutive_errors == 1:
                    skip_count += 1
                    # 第一次超时不报错，继续尝试
                    pass
                elif consecutive_errors >= 2:
                    # 连续超时才报错
                    current_time = time.time()
                    if current_time - self.last_status_time >= self.status_update_interval:
                        self.connection_status.emit(False, f"✗ 连接不稳定")
                        self.last_status_time = current_time
                
            except requests.exceptions.ConnectionError:
                consecutive_errors += 1
                self.error_count += 1
                current_time = time.time()
                if current_time - self.last_status_time >= self.status_update_interval:
                    self.connection_status.emit(False, f"✗ 设备离线")
                    self.last_status_time = current_time
                
            except Exception as e:
                consecutive_errors += 1
                self.error_count += 1
                # 其他错误静默处理，不频繁报错
                
            # 智能等待策略
            request_time = time.time() - request_start
            
            if consecutive_errors >= max_errors:
                # 连续错误过多，增加等待时间
                time.sleep(1.0)
                consecutive_errors = max_errors - 1
            else:
                # 动态调整等待时间，补偿请求耗时
                wait_time = max(0, self.interval - request_time)
                if wait_time > 0:
                    time.sleep(wait_time)
    
    def stop(self):
        """停止数据获取"""
        self.running = False
        
        # 关闭session
        if hasattr(self, 'session'):
            self.session.close()
        
        self.wait()  # 等待线程结束
        
        # 打印统计信息
        if self.success_count > 0:
            success_rate = self.success_count / (self.success_count + self.error_count) * 100
            print(f"📊 数据获取统计: 成功 {self.success_count} 次, 失败 {self.error_count} 次, 成功率 {success_rate:.1f}%")
    
    def set_url(self, url):
        """更新URL地址"""
        self.url = url
    
    def set_interval(self, interval):
        """
        更新数据获取间隔
        
        Args:
            interval: 间隔时间(毫秒)
        """
        self.interval = interval / 1000.0


# 测试代码
if __name__ == '__main__':
    from PyQt5.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    
    def on_data(data):
        print(f"加速度: X={data['accelX']:.3f} Y={data['accelY']:.3f} Z={data['accelZ']:.3f}")
        print(f"陀螺仪: X={data['gyroX']:.2f} Y={data['gyroY']:.2f} Z={data['gyroZ']:.2f}")
        print(f"温度: {data['temperature']:.2f}°C")
        
        # 如果有MT6701角度数据，也打印出来
        if 'angle' in data and data.get('angleValid', False):
            print(f"角度: {data['angle']:.2f}° (原始: {data.get('angleRaw', 0)}/16383)")
        print()
    
    def on_status(connected, message):
        print(f"连接状态: {message}")
    
    fetcher = DataFetcher("http://192.168.3.57/data")
    fetcher.data_received.connect(on_data)
    fetcher.connection_status.connect(on_status)
    fetcher.start()
    
    print("正在获取数据... (按Ctrl+C退出)")
    sys.exit(app.exec_())


import sys
import subprocess
import numpy as np
import json
import os
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, QLabel, QFrame, QTabWidget, QScrollArea, QGroupBox, QRadioButton, QTimeEdit, QButtonGroup, QDialog, QGridLayout, QFileDialog, QProgressBar
from PyQt5.QtGui import QFont, QColor, QPainter, QImage, QPixmap, QPen, QBrush, QPolygonF
from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSlot, QThread, pyqtSignal, QTime

from chat_ui import ChatDialog

class ProcessReaderThread(QThread):
    output_signal = pyqtSignal(str)

    def __init__(self, process):
        super().__init__()
        self.process = process
        self.running = True

    def run(self):
        while self.running and self.process and self.process.poll() is None:
            line = self.process.stdout.readline()
            if line:
                self.output_signal.emit(line.strip())
            else:
                break

    def stop(self):
        self.running = False

# 尝试导入底层的 ROS Worker
try:
    from ros_worker import ROSWorker
except ImportError as e:
    print(f"[System] Warning: Failed to import ROSWorker. Error: {e}")
    ROSWorker = None

class MapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.map_data = None
        self.map_info = None # (width, height, resolution, origin_x, origin_y)
        self.robot_pose = None # (x, y, yaw)
        self.map_qimage = None
        self.setStyleSheet("background-color: #11111b; border-radius: 10px;")

    def update_map(self, map_tuple):
        data, width, height, resolution, origin_x, origin_y = map_tuple
        self.map_info = (width, height, resolution, origin_x, origin_y)
        
        # 将 numpy 数组转为 RGB 图像格式
        # -1 (未知) -> 灰色, 0 (空闲) -> 浅色, 100 (障碍) -> 黑色
        img_array = np.zeros((height, width, 3), dtype=np.uint8)
        img_array[data == -1] = [80, 80, 90]   # Unknown: 科技灰
        img_array[data == 0] = [200, 200, 220] # Free: 亮灰白
        img_array[data >= 50] = [10, 10, 20]   # Occupied: 极暗黑
        
        # 翻转 y 轴 (ROS 的原点在左下角，Qt 在左上角)
        img_array = np.flipud(img_array)

        # 转换为 QImage (修复 PyQt5 不接受 memoryview 的问题)
        height, width, channel = img_array.shape
        bytesPerLine = 3 * width
        self.map_qimage = QImage(img_array.tobytes(), width, height, bytesPerLine, QImage.Format_RGB888).copy()
        
        self.update() # 触发重绘

    def update_pose(self, x, y, yaw):
        self.robot_pose = (x, y, yaw)
        self.update() # 仅重绘小车

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(20, 20, 20)) # 深灰色背景
        
        if self.map_qimage:
            # 保持原始比例缩放地图
            scaled_img = self.map_qimage.scaled(self.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
            
            # 计算居中偏移量
            x_offset = (self.width() - scaled_img.width()) // 2
            y_offset = (self.height() - scaled_img.height()) // 2
            
            # 绘制地图
            painter.drawImage(x_offset, y_offset, scaled_img)
            
            # 如果有小车坐标，叠加在地图上
            if self.robot_pose and self.map_info:
                x, y, yaw = self.robot_pose
                width, height, resolution, origin_x, origin_y = self.map_info

                # 将 ROS 世界坐标转换为 Qt 像素坐标
                # 1. 计算在栅格地图上的坐标 (原点在左下角)
                grid_x = (x - origin_x) / resolution
                grid_y = (y - origin_y) / resolution

                # 2. 转换到 QImage 坐标系 (因为翻转了 y 轴)
                img_x = grid_x
                img_y = height - grid_y

                # 3. 转换到缩放后的窗口坐标系
                scale_factor = scaled_img.width() / width
                screen_x = x_offset + img_x * scale_factor
                screen_y = y_offset + img_y * scale_factor

                # 画小车 (绿色三角形)
                painter.translate(screen_x, screen_y)
                # 注意：Qt的Y轴朝下，而ROS的世界坐标里Yaw是相对于X轴。因此绘画时旋转需考虑Y翻转。
                # 另外，我们只做视觉示意，顺时针为正。
                painter.rotate(math.degrees(-yaw)) 
                
                # 把原本的 12 改成 30，让红色图标变大两倍以上
                car_size = 30
                polygon = QPolygonF([
                    QPointF(car_size, 0),
                    QPointF(-car_size/2, car_size/2),
                    QPointF(-car_size/2, -car_size/2)
                ])
                
                painter.setBrush(QBrush(QColor(255, 50, 50))) # 醒目红
                painter.setPen(QPen(QColor(255, 255, 255), 1)) # 加一圈白色描边更明显
                painter.drawPolygon(polygon)
        else:
            painter.setPen(QColor(150, 150, 150))
            painter.setFont(QFont("Microsoft YaHei", 14))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待加载 SLAM 地图...")

import math

class PowerConfigDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("巡检后电源策略配置")
        self.setFixedSize(800, 500)
        self.setStyleSheet("""
            QDialog {
                background-color: rgba(20, 20, 25, 240);
                border: 2px solid #45a29e;
                border-radius: 15px;
            }
            QLabel {
                color: #c5c6c7;
            }
            QPushButton {
                border-radius: 8px;
                font-weight: bold;
            }
        """)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        
        self.selected_mode = None
        self.hour = 8
        self.minute = 0
        
        self.init_ui()
        
    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(30, 30, 30, 30)
        main_layout.setSpacing(20)
        
        title = QLabel("请选择巡检完成后的电源策略")
        title.setFont(QFont("Microsoft YaHei", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #66fcf1; border: none;")
        main_layout.addWidget(title)
        
        # Keep On Button
        self.btn_keep_on = QPushButton("🔋 保持开机待命 (不断电)")
        self.btn_keep_on.setFixedHeight(60)
        self.btn_keep_on.setFont(QFont("Microsoft YaHei", 16, QFont.Bold))
        self.btn_keep_on.setCheckable(True)
        self.btn_keep_on.clicked.connect(self.on_keep_on_clicked)
        
        # Sleep Button
        self.btn_sleep = QPushButton("💤 关机休眠，定时唤醒")
        self.btn_sleep.setFixedHeight(60)
        self.btn_sleep.setFont(QFont("Microsoft YaHei", 16, QFont.Bold))
        self.btn_sleep.setCheckable(True)
        self.btn_sleep.clicked.connect(self.on_sleep_clicked)
        
        self.btn_group = QButtonGroup(self)
        self.btn_group.addButton(self.btn_keep_on)
        self.btn_group.addButton(self.btn_sleep)
        
        # Default style for buttons
        self.btn_style = """
            QPushButton {
                background-color: #1f2833;
                color: #c5c6c7;
                border: 2px solid #1f2833;
            }
            QPushButton:checked {
                background-color: #2c3e50;
                color: #66fcf1;
                border: 2px solid #66fcf1;
            }
        """
        self.btn_keep_on.setStyleSheet(self.btn_style)
        self.btn_sleep.setStyleSheet(self.btn_style)
        
        main_layout.addWidget(self.btn_keep_on)
        main_layout.addWidget(self.btn_sleep)
        
        # Time Picker Container
        self.time_container = QWidget()
        self.time_container.setStyleSheet("border: none; background-color: transparent;")
        time_layout = QHBoxLayout(self.time_container)
        time_layout.setContentsMargins(0, 0, 0, 0)
        
        # Hour controls
        hour_layout = QVBoxLayout()
        btn_h_up = QPushButton("▲")
        btn_h_up.setFixedSize(120, 80)
        btn_h_up.setFont(QFont("Arial", 36))
        btn_h_up.clicked.connect(lambda: self.adjust_time('h', 1))
        
        self.lbl_hour = QLabel(f"{self.hour:02d}")
        self.lbl_hour.setFont(QFont("Arial", 60, QFont.Bold))
        self.lbl_hour.setAlignment(Qt.AlignCenter)
        self.lbl_hour.setStyleSheet("border: none;")
        
        btn_h_down = QPushButton("▼")
        btn_h_down.setFixedSize(120, 80)
        btn_h_down.setFont(QFont("Arial", 36))
        btn_h_down.clicked.connect(lambda: self.adjust_time('h', -1))
        
        hour_layout.addWidget(btn_h_up, 0, Qt.AlignCenter)
        hour_layout.addWidget(self.lbl_hour, 0, Qt.AlignCenter)
        hour_layout.addWidget(btn_h_down, 0, Qt.AlignCenter)
        
        # Colon
        lbl_colon = QLabel(":")
        lbl_colon.setFont(QFont("Arial", 60, QFont.Bold))
        lbl_colon.setAlignment(Qt.AlignCenter)
        lbl_colon.setStyleSheet("border: none;")
        
        # Minute controls
        min_layout = QVBoxLayout()
        btn_m_up = QPushButton("▲")
        btn_m_up.setFixedSize(120, 80)
        btn_m_up.setFont(QFont("Arial", 36))
        btn_m_up.clicked.connect(lambda: self.adjust_time('m', 5))
        
        self.lbl_min = QLabel(f"{self.minute:02d}")
        self.lbl_min.setFont(QFont("Arial", 60, QFont.Bold))
        self.lbl_min.setAlignment(Qt.AlignCenter)
        self.lbl_min.setStyleSheet("border: none;")
        
        btn_m_down = QPushButton("▼")
        btn_m_down.setFixedSize(120, 80)
        btn_m_down.setFont(QFont("Arial", 36))
        btn_m_down.clicked.connect(lambda: self.adjust_time('m', -5))
        
        min_layout.addWidget(btn_m_up, 0, Qt.AlignCenter)
        min_layout.addWidget(self.lbl_min, 0, Qt.AlignCenter)
        min_layout.addWidget(btn_m_down, 0, Qt.AlignCenter)
        
        # Add to time_container
        time_layout.addStretch()
        time_layout.addLayout(hour_layout)
        time_layout.addWidget(lbl_colon)
        time_layout.addLayout(min_layout)
        time_layout.addStretch()
        
        # Style the adjustment buttons
        adj_btn_style = """
            QPushButton {
                background-color: #2b2b2b;
                color: #45a29e;
                border: 2px solid #2b2b2b;
            }
            QPushButton:pressed {
                background-color: #1a1a1a;
            }
        """
        btn_h_up.setStyleSheet(adj_btn_style)
        btn_h_down.setStyleSheet(adj_btn_style)
        btn_m_up.setStyleSheet(adj_btn_style)
        btn_m_down.setStyleSheet(adj_btn_style)
        
        main_layout.addWidget(self.time_container)
        self.time_container.setVisible(False)
        
        # Bottom Buttons
        btn_layout = QHBoxLayout()
        
        self.btn_cancel = QPushButton("✖ 取消")
        self.btn_cancel.setFixedHeight(60)
        self.btn_cancel.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_cancel.setStyleSheet("""
            QPushButton {
                background-color: #d9534f;
                color: white;
                border: none;
            }
            QPushButton:pressed {
                background-color: #c9302c;
            }
        """)
        self.btn_cancel.clicked.connect(self.reject)
        
        self.btn_confirm = QPushButton("🚀 确认发车")
        self.btn_confirm.setFixedHeight(60)
        self.btn_confirm.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_confirm.setEnabled(False)
        self.btn_confirm.setStyleSheet("""
            QPushButton {
                background-color: #2c3e50;
                color: #7f8c8d;
                border: none;
            }
            QPushButton:enabled {
                background-color: #5bc0de;
                color: white;
            }
            QPushButton:enabled:pressed {
                background-color: #31b0d5;
            }
        """)
        self.btn_confirm.clicked.connect(self.accept)
        
        btn_layout.addWidget(self.btn_cancel)
        btn_layout.addWidget(self.btn_confirm)
        
        main_layout.addLayout(btn_layout)
        
    def on_keep_on_clicked(self):
        self.selected_mode = "keep-on"
        self.time_container.setVisible(False)
        self.btn_confirm.setEnabled(True)
        
    def on_sleep_clicked(self):
        self.selected_mode = "sleep-until"
        self.time_container.setVisible(True)
        self.btn_confirm.setEnabled(True)
        
    def adjust_time(self, unit, amount):
        if unit == 'h':
            self.hour = (self.hour + amount) % 24
            self.lbl_hour.setText(f"{self.hour:02d}")
        elif unit == 'm':
            self.minute = (self.minute + amount) % 60
            self.lbl_min.setText(f"{self.minute:02d}")
            
    def get_power_arg(self):
        if self.selected_mode == "keep-on":
            return "keep-on"
        else:
            return f"sleep-until-{self.hour:02d}:{self.minute:02d}"

class OTAReaderThread(QThread):
    log_signal = pyqtSignal(str)
    progress_signal = pyqtSignal(int, int)
    finished_signal = pyqtSignal(int)

    def __init__(self, process):
        super().__init__()
        self.process = process
        self.running = True

    def run(self):
        buffer = ""
        while self.running and self.process and self.process.poll() is None:
            char = self.process.stdout.read(1)
            if not char:
                break
            if char == '\r' or char == '\n':
                line = buffer.strip()
                if line:
                    if line.startswith("Progress:"):
                        try:
                            parts = line.split()
                            current = int(parts[1])
                            total = int(parts[3])
                            self.progress_signal.emit(current, total)
                        except Exception:
                            self.log_signal.emit(line)
                    else:
                        self.log_signal.emit(line)
                buffer = ""
            else:
                buffer += char
        if buffer:
            self.log_signal.emit(buffer.strip())
        self.process.wait()
        self.finished_signal.emit(self.process.returncode)

    def stop(self):
        self.running = False


class OTADialog(QDialog):
    def __init__(self, parent=None, file_path=""):
        super().__init__(parent)
        self.file_path = file_path
        self.main_window = parent
        self.setWindowTitle("OTA 固件升级")
        self.setFixedSize(700, 500)
        self.setStyleSheet("""
            QDialog {
                background-color: rgba(20, 20, 25, 240);
                border: 2px solid #f0ad4e;
                border-radius: 15px;
            }
            QLabel { color: #c5c6c7; }
            QPushButton { border-radius: 8px; font-weight: bold; }
        """)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        
        self.process = None
        self.reader = None
        self.is_success = False
        self.init_ui()
        self.start_ota()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(30, 30, 30, 30)
        main_layout.setSpacing(15)

        title = QLabel("📡 STM32 固件 OTA 升级")
        title.setFont(QFont("Microsoft YaHei", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #f0ad4e; border: none;")
        main_layout.addWidget(title)

        self.lbl_file = QLabel(f"固件路径: {self.file_path}")
        self.lbl_file.setFont(QFont("Microsoft YaHei", 10))
        self.lbl_file.setStyleSheet("border: none; color: #a9a9a9;")
        self.lbl_file.setWordWrap(True)
        main_layout.addWidget(self.lbl_file)

        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet("""
            QProgressBar { border: 2px solid #444; border-radius: 5px; text-align: center; background-color: #2b2b2b; color: white; font-weight: bold; }
            QProgressBar::chunk { background-color: #f0ad4e; }
        """)
        self.progress_bar.setFixedHeight(30)
        self.progress_bar.setValue(0)
        main_layout.addWidget(self.progress_bar)

        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setFont(QFont("Consolas", 10))
        self.log_area.setStyleSheet("QTextEdit { background-color: #1a1a1a; border: 1px solid #444; border-radius: 5px; color: #00ff00; }")
        main_layout.addWidget(self.log_area)

        self.btn_close = QPushButton("✖ 关闭")
        self.btn_close.setFixedHeight(50)
        self.btn_close.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_close.setEnabled(False)
        self.btn_close.setStyleSheet("""
            QPushButton { background-color: #d9534f; color: white; border: none; }
            QPushButton:pressed { background-color: #c9302c; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self.btn_close.clicked.connect(self.accept)
        main_layout.addWidget(self.btn_close)

    def start_ota(self):
        self.log_area.append("[System] 准备 OTA 烧录。")
        import time
        time.sleep(0.5)

        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        send_script = os.path.join(base_dir, "send.py")
        if not os.path.exists(send_script):
            send_script = "/root/send.py"

        port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
        baudrate = "115200"

        cmd = ["python3", send_script, self.file_path, "--port", port, "--baudrate", baudrate]
        self.log_area.append(f"[System] 执行命令: {' '.join(cmd)}\n")

        try:
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            self.reader = OTAReaderThread(self.process)
            self.reader.log_signal.connect(self.on_log)
            self.reader.progress_signal.connect(self.on_progress)
            self.reader.finished_signal.connect(self.on_finished)
            self.reader.start()
        except Exception as e:
            self.log_area.append(f"[Error] OTA 进程启动失败: {e}")
            self.btn_close.setEnabled(True)

    @pyqtSlot(str)
    def on_log(self, text):
        self.log_area.append(text)
        self.log_area.verticalScrollBar().setValue(self.log_area.verticalScrollBar().maximum())

    @pyqtSlot(int, int)
    def on_progress(self, current, total):
        if total > 0:
            self.progress_bar.setMaximum(total)
            self.progress_bar.setValue(current)

    @pyqtSlot(int)
    def on_finished(self, returncode):
        if returncode == 0:
            self.is_success = True
            self.log_area.append("\n[Success] OTA 固件烧录成功！关闭窗口后将自动重启 ROS 系统。")
            self.progress_bar.setStyleSheet("""
                QProgressBar { border: 2px solid #444; border-radius: 5px; text-align: center; background-color: #2b2b2b; color: white; font-weight: bold; }
                QProgressBar::chunk { background-color: #5cb85c; }
            """)
        else:
            self.log_area.append(f"\n[Error] OTA 失败，退出码: {returncode}")
            self.progress_bar.setStyleSheet("""
                QProgressBar { border: 2px solid #444; border-radius: 5px; text-align: center; background-color: #2b2b2b; color: white; font-weight: bold; }
                QProgressBar::chunk { background-color: #d9534f; }
            """)
        self.btn_close.setEnabled(True)

    def closeEvent(self, event):
        if self.process and self.process.poll() is None:
            self.process.terminate()
        if self.reader:
            self.reader.stop()
        super().closeEvent(event)
        # 通知主窗口自动重启底盘驱动和导航
        if self.is_success and self.main_window:
            self.main_window.restart_base_driver()


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AI 巡检边缘终端")
        # 移除固定的 1024x600，让系统接管真实分辨率
        # 设置无边框，防止桌面管理器强制加标题栏
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setStyleSheet("background-color: #0b0c10; color: #c5c6c7;")

        self.patrol_process = None
        self.nav_process = None
        self.nav_reader = None

        self.init_ui()
        self.init_ros()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        h_layout = QHBoxLayout(main_widget)
        h_layout.setContentsMargins(10, 10, 10, 10)
        h_layout.setSpacing(15)

        # 左侧：地图导航区 (600px 宽)
        left_panel = QFrame()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        map_title = QLabel("SLAM 实时导航")
        map_title.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        map_title.setStyleSheet("color: #66fcf1;")
        
        self.map_widget = MapWidget()
        
        left_layout.addWidget(map_title)
        left_layout.addWidget(self.map_widget)
        # 移除固定宽度，让布局自适应

        # 右侧：控制与日志区 (余下宽度)
        right_panel = QFrame()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        # 顶部：机器人状态
        self.status_label = QLabel("🔋 电量: --% (--V)  |  🌡️ CPU: --°C")
        self.status_label.setStyleSheet("color: #66fcf1; font-size: 16px; font-weight: bold; padding: 5px; background: #1f2833; border-radius: 5px;")
        self.status_label.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.status_label)
        
        # (原先的配置区已移除，由发车时的触控弹窗代替)
        
        # 替换原有的单一文本框，加入 Tab 切换
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabBar::tab {
                background: #2a2a2a;
                color: #c5c6c7;
                padding: 10px 20px;
                font-size: 14px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background: #1f2833;
                color: #66fcf1;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #45a29e;
                background: #1f2833;
                border-radius: 8px;
            }
        """)

        # Tab 1: 图文巡检流 (Feed)
        self.feed_scroll = QScrollArea()
        self.feed_scroll.setWidgetResizable(True)
        self.feed_scroll.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")
        
        self.feed_container = QWidget()
        self.feed_container.setStyleSheet("background-color: transparent;")
        self.feed_layout = QVBoxLayout(self.feed_container)
        self.feed_layout.setContentsMargins(10, 10, 10, 10)
        self.feed_layout.setSpacing(15)
        self.feed_layout.addStretch() # 把卡片往上顶
        self.feed_scroll.setWidget(self.feed_container)

        # Tab 2: 原始文本日志 (Log)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setFont(QFont("Microsoft YaHei", 12))
        self.log_area.setStyleSheet("QTextEdit { background-color: transparent; border: none; color: #ffffff; }")
        self.log_area.append("[System] 系统初始化完成，等待发车。")

        # Tab 3: 导航与底层日志 (Nav Log)
        self.nav_log_area = QTextEdit()
        self.nav_log_area.setReadOnly(True)
        self.nav_log_area.setFont(QFont("Microsoft YaHei", 10))
        self.nav_log_area.setStyleSheet("QTextEdit { background-color: transparent; border: none; color: #00ffcc; }")
        self.nav_log_area.append("[System] 等待导航模块启动...")

        self.tab_widget.addTab(self.feed_scroll, "📸 巡检图文记录")
        self.tab_widget.addTab(self.log_area, "📝 原始通信日志")
        self.tab_widget.addTab(self.nav_log_area, "🗺️ 导航与底层日志")
        
        # 启动巡检结果的后台读取定时器
        self.known_count = 0
        self.feed_timer = QTimer(self)
        self.feed_timer.timeout.connect(self.poll_inspection_history)
        self.feed_timer.start(1000)

        # 底部按钮区
        btn_layout = QHBoxLayout()
        
        self.btn_nav = QPushButton("🗺️ 启动导航")
        self.btn_nav.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_nav.setFixedHeight(80)
        self.btn_nav.setStyleSheet("""
            QPushButton {
                background-color: #5bc0de;
                color: #ffffff;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #31b0d5;
            }
        """)
        self.btn_nav.clicked.connect(self.on_start_nav)

        self.btn_start = QPushButton("🚀 开始自动巡检(等待导航...)")
        self.btn_start.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_start.setFixedHeight(80)
        self.btn_start.setEnabled(False)
        self.btn_start.setStyleSheet("""
            QPushButton {
                background-color: #2c3e50;
                color: #7f8c8d;
                border-radius: 10px;
            }
        """)
        self.btn_start.clicked.connect(self.on_start_patrol)

        self.btn_ota = QPushButton("🔄 固件OTA")
        self.btn_ota.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_ota.setFixedHeight(80)
        self.btn_ota.setStyleSheet("""
            QPushButton {
                background-color: #f0ad4e;
                color: #ffffff;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #ec971f;
            }
        """)
        self.btn_ota.clicked.connect(self.on_start_ota)

        self.btn_chat = QPushButton("💬 大模型交互")
        self.btn_chat.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_chat.setFixedHeight(80)
        self.btn_chat.setStyleSheet("""
            QPushButton {
                background-color: #8a2be2;
                color: #ffffff;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #7b2cbf;
            }
        """)
        self.btn_chat.clicked.connect(self.on_start_chat)

        self.btn_stop = QPushButton("🛑 紧急停止")
        self.btn_stop.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.btn_stop.setFixedHeight(80)
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #d9534f;
                color: #ffffff;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #c9302c;
            }
        """)
        self.btn_stop.clicked.connect(self.on_stop_patrol)

        btn_layout.addWidget(self.btn_nav)
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_chat)
        btn_layout.addWidget(self.btn_ota)
        btn_layout.addWidget(self.btn_stop)

        right_layout.addWidget(self.tab_widget)
        right_layout.addLayout(btn_layout)

        # 核心：使用权重划分比例 (左边占60%，右边占40%)，完美铺满任何长宽比屏幕
        h_layout.addWidget(left_panel, 6)
        h_layout.addWidget(right_panel, 4)

    def init_ros(self):
        if ROSWorker:
            self.ros_worker = ROSWorker()
            self.ros_worker.map_signal.connect(self.map_widget.update_map)
            self.ros_worker.pose_signal.connect(self.map_widget.update_pose)
            self.ros_worker.log_signal.connect(self.append_ai_log)
            self.ros_worker.status_signal.connect(self.update_status)
            self.ros_worker.start()
        else:
            self.log_area.append("[Error] 无法加载 ROS 依赖，仅为 UI 演示模式。")

    def update_status(self, status_dict):
        if not hasattr(self, 'robot_status_data'):
            self.robot_status_data = {"voltage": 0.0, "percentage": 0.0, "temp": 0.0}
            
        if status_dict.get("type") == "battery":
            self.robot_status_data["voltage"] = status_dict["voltage"]
            self.robot_status_data["percentage"] = status_dict["percentage"]
        elif status_dict.get("type") == "temp":
            self.robot_status_data["temp"] = status_dict["value"]
            
        status_text = f"🔋 电量: {self.robot_status_data['percentage']}% ({self.robot_status_data['voltage']}V)  |  🌡️ CPU: {self.robot_status_data['temp']}°C"
        self.status_label.setText(status_text)

    def poll_inspection_history(self):
        jsonl_path = '/tmp/inspection_history.jsonl'
        if not os.path.exists(jsonl_path):
            return
            
        try:
            with open(jsonl_path, 'r', encoding='utf-8') as f:
                lines = [line.strip() for line in f.readlines() if line.strip()]
                
            if len(lines) > self.known_count:
                for i in range(self.known_count, len(lines)):
                    node_data = json.loads(lines[i])
                    self.add_feed_card(i + 1, node_data)
                self.known_count = len(lines)
        except Exception as e:
            pass

    def add_feed_card(self, index, data):
        card = QFrame()
        card.setStyleSheet("""
            QFrame {
                background-color: #121212;
                border: 1px solid #444444;
                border-radius: 8px;
            }
        """)
        card_layout = QVBoxLayout(card)
        
        # Header
        header = QLabel(f"巡检节点 #{index}")
        header.setStyleSheet("color: #90caf9; font-size: 14px; font-weight: bold; border: none; padding-bottom: 5px;")
        card_layout.addWidget(header)
        
        # Image
        img_path = data.get("image_path", "")
        if img_path and os.path.exists(img_path):
            img_label = QLabel()
            pixmap = QPixmap(img_path)
            scaled_pixmap = pixmap.scaledToWidth(350, Qt.SmoothTransformation)
            img_label.setPixmap(scaled_pixmap)
            img_label.setAlignment(Qt.AlignCenter)
            img_label.setStyleSheet("border: none; padding: 5px; background-color: #000000;")
            card_layout.addWidget(img_label)
            
        # Description
        desc_text = data.get("description", "")
        desc_text = desc_text.replace("正常", "<span style='color:#69f0ae; font-weight:bold;'>正常</span>")
        desc_text = desc_text.replace("异常", "<span style='color:#ff5252; font-weight:bold;'>异常</span>")
        
        desc_label = QLabel(desc_text)
        desc_label.setWordWrap(True)
        desc_label.setStyleSheet("color: #e0e0e0; font-size: 14px; border: none; margin-top: 5px; line-height: 1.5;")
        card_layout.addWidget(desc_label)
        
        self.feed_layout.insertWidget(0, card)

    @pyqtSlot(str)
    def append_ai_log(self, text):
        self.log_area.append(f"\n[AI]: {text}")
        # 自动滚动到底部
        self.log_area.verticalScrollBar().setValue(self.log_area.verticalScrollBar().maximum())

    @pyqtSlot(str)
    def append_nav_log(self, text):
        self.nav_log_area.append(text)
        self.nav_log_area.verticalScrollBar().setValue(self.nav_log_area.verticalScrollBar().maximum())
        
        if "Managed nodes are active" in text and not self.btn_start.isEnabled():
            self.log_area.append("\n[System] ✅ 检测到 Nav2 节点已全部激活，巡检功能已解锁！")
            self.nav_log_area.append("\n[System] ✅ Nav2 启动完成，可以开始巡检！")
            self.btn_start.setText("🚀 开始自动巡检")
            self.btn_start.setEnabled(True)
            self.btn_start.setStyleSheet("""
                QPushButton {
                    background-color: #45a29e;
                    color: #0b0c10;
                    border-radius: 10px;
                }
                QPushButton:pressed {
                    background-color: #66fcf1;
                }
            """)

    def on_start_nav(self):
        self.log_area.append("\n[System] 正在启动 Nav2 导航模块...")
        self.tab_widget.setCurrentIndex(2) # 自动切换到第三个Tab (索引2)
        if self.nav_process is None or self.nav_process.poll() is not None:
            try:
                # 动态获取当前脚本所在的项目根目录 (向上跳两级: ui -> src -> 根目录)
                base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                nav_src = os.path.join(base_dir, "my_nav_launch.py")
                if not os.path.exists(nav_src):
                    nav_src = "/root/my_nav_launch.py"
                    
                nav_dst = nav_src.replace(".py", ".launch.py")
                
                cmd = ["bash", "-c", f"source /opt/ros/humble/setup.bash && source {base_dir}/install/setup.bash && cp -f '{nav_src}' '{nav_dst}' && ros2 launch '{nav_dst}'"]
                self.nav_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
                
                self.nav_reader = ProcessReaderThread(self.nav_process)
                self.nav_reader.output_signal.connect(self.append_nav_log)
                self.nav_reader.start()

                self.log_area.append("[System] Nav2 导航模块已成功启动！")
            except Exception as e:
                self.log_area.append(f"[Error] Nav2 启动失败: {str(e)}")
        else:
            self.log_area.append("[Warning] Nav2 导航模块已经在运行中！")

    def restart_base_driver(self):
        self.log_area.append("\n[System] 正在重新拉起底盘与雷达驱动...")
        self.tab_widget.setCurrentIndex(2)
        try:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            avoid_src = os.path.join(base_dir, "stm32_sllidar_avoid_launch.py")
            if not os.path.exists(avoid_src):
                avoid_src = "/root/stm32_sllidar_avoid_launch.py"
                
            avoid_dst = avoid_src.replace(".py", ".launch.py")
            cmd = ["bash", "-c", f"source /opt/ros/humble/setup.bash && source {base_dir}/install/setup.bash && cp -f '{avoid_src}' '{avoid_dst}' && ros2 launch '{avoid_dst}'"]
            
            # 使用 hasattr 防止报错
            if hasattr(self, 'base_process') and self.base_process and self.base_process.poll() is None:
                self.base_process.terminate()
                
            self.base_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            self.base_reader = ProcessReaderThread(self.base_process)
            self.base_reader.output_signal.connect(self.append_nav_log)
            self.base_reader.start()
            
            self.log_area.append("[System] 底盘与雷达驱动已后台重新拉起，您可以随时手动点击“启动导航”！")
        except Exception as e:
            self.log_area.append(f"[Error] 底盘驱动重启失败: {str(e)}")

    def on_start_patrol(self):
        # 弹出纯触控优化的电源策略选择框
        dialog = PowerConfigDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            self.log_area.append("\n[System] 发车已取消。")
            return
            
        power_arg = dialog.get_power_arg()

        self.log_area.append("\n[System] 正在启动主巡检脚本...")
        if self.patrol_process is None or self.patrol_process.poll() is not None:
            try:
                # 动态获取项目根目录
                base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                patrol_script = os.path.join(base_dir, "patrol_commander.py")
                if not os.path.exists(patrol_script):
                    patrol_script = "/root/patrol_commander.py"
                    
                cmd = ["python3", patrol_script, "--power-mode", power_arg]

                self.patrol_process = subprocess.Popen(cmd)
                self.log_area.append(f"[System] patrol_commander.py 已成功启动！(参数: {power_arg})")
            except Exception as e:
                self.log_area.append(f"[Error] 启动失败: {str(e)}")
        else:
            self.log_area.append("[Warning] 巡检任务已经在运行中！")

    def on_stop_patrol(self):
        self.log_area.append("\n[System] 触发紧急停止！尝试终止巡检任务...")
        if self.patrol_process:
            self.patrol_process.terminate()
            self.patrol_process = None
            self.log_area.append("[System] patrol_commander.py 已被终止。")
        else:
            # 也可以暴力 kill 整个 python3 进程，防患未然
            subprocess.run(["pkill", "-9", "-f", "patrol_commander.py"])
            self.log_area.append("[System] 发送全局强制终止信号。")
        
        # 提示用户底层可能需要用三连杀
        self.log_area.append("[Notice] 若底层底盘未停下，请手动控制接管！")

    def on_start_chat(self):
        self.log_area.append("\n[System] 正在唤起全屏智能交互界面...")
        dialog = ChatDialog(self, getattr(self, 'ros_worker', None))
        dialog.exec_()

    def on_start_ota(self):
        file_path = "/root/app.bin"
        if not os.path.exists(file_path):
            self.log_area.append(f"\n[Error] 找不到固件文件: {file_path}，请确保固件已就绪！")
            return
            
        self.log_area.append("\n[System] 正在准备 OTA，停止所有后台 ROS 进程...")
        self.tab_widget.setCurrentIndex(1)
        
        if self.patrol_process and self.patrol_process.poll() is None:
            self.patrol_process.terminate()
            self.patrol_process = None
            
        if self.nav_process and self.nav_process.poll() is None:
            self.nav_process.terminate()
            self.nav_process = None

        subprocess.run(["pkill", "-9", "-f", "patrol_commander.py"])
        subprocess.run(["pkill", "-9", "-f", "stm32_sllidar_avoid"])
        subprocess.run(["pkill", "-9", "-f", "stm32_ros2_bridge.py"])
        subprocess.run(["pkill", "-9", "-f", "sllidar_node"])
        subprocess.run(["pkill", "-9", "-f", "ros2 run tf2_ros static_transform_publisher"])
        
        # 为了保证串口彻底释放，多等一秒
        QTimer.singleShot(1000, lambda: self._show_ota_dialog(file_path))
        
    def _show_ota_dialog(self, file_path):
        dialog = OTADialog(self, file_path)
        dialog.exec_()

    def closeEvent(self, event):
        if hasattr(self, 'ros_worker') and self.ros_worker:
            self.ros_worker.stop()
        if self.patrol_process:
            self.patrol_process.terminate()
        if hasattr(self, 'nav_reader') and self.nav_reader:
            self.nav_reader.stop()
        if self.nav_process:
            self.nav_process.terminate()
        super().closeEvent(event)


if __name__ == "__main__":
    import os
    import glob
    import subprocess
    
    # 针对现代 Ubuntu (Wayland) 桌面极其严苛的 root X11 拦截，直接切换到 Wayland 原生模式
    if os.path.exists("/run/user/1000/wayland-0"):
        os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"
        os.environ["WAYLAND_DISPLAY"] = "wayland-0"
        # 强制 Qt 使用 Wayland 引擎，绕过 X11 的 Authorization 检查
        os.environ["QT_QPA_PLATFORM"] = "wayland"
        print("🚀 [System] Auto-configured for Wayland Native Display. Bypassing X11 auth...")
    else:
        if "DISPLAY" not in os.environ:
            os.environ["DISPLAY"] = ":0"
    
    # 终极解决方案：从系统进程中强行抓取 X server 的真实授权文件路径
    try:
        ps_output = subprocess.check_output("ps aux | grep -E 'Xorg|Xwayland' | grep -v grep", shell=True).decode()
        if "-auth " in ps_output:
            real_auth = ps_output.split("-auth ")[1].split()[0]
            os.environ["XAUTHORITY"] = real_auth
            print(f"[System] Auto-detected XAUTHORITY: {real_auth}")
        else:
            auth_files = glob.glob("/home/*/.Xauthority") + glob.glob("/run/user/*/.Xauthority") + glob.glob("/var/run/lightdm/root/:0")
            if auth_files:
                os.environ["XAUTHORITY"] = auth_files[0]
    except Exception as e:
        print(f"[System] Auth detection warning: {e}")
            
    app = QApplication(sys.argv)
    window = Dashboard()
    window.showFullScreen()
    sys.exit(app.exec_())

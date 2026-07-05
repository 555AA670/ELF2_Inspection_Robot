import sys
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QScrollArea, QLineEdit, QDialog, QFrame, QSizePolicy)
from PyQt5.QtGui import QFont, QColor, QPainter, QPainterPath
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QSize, QTimer

class ClickableLineEdit(QLineEdit):
    clicked = pyqtSignal()
    
    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self.clicked.emit()

class ChatBubble(QWidget):
    def __init__(self, text, is_user=False):
        super().__init__()
        self.is_user = is_user
        self.text = text
        self.init_ui()
        
    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 10, 15, 10)
        
        # User Avatar
        self.avatar = QLabel()
        self.avatar.setFixedSize(45, 45)
        self.avatar.setAlignment(Qt.AlignCenter)
        self.avatar.setFont(QFont("Segoe UI Emoji", 24))
        
        # Message bubble
        self.label = QLabel(self.text)
        self.label.setFont(QFont("Microsoft YaHei", 14))
        self.label.setWordWrap(True)
        # Limit max width so bubble doesn't span entire screen
        self.label.setMaximumWidth(650)
        
        if self.is_user:
            self.avatar.setText("👤")
            self.label.setStyleSheet("background-color: #95ec69; color: black; border-radius: 10px; padding: 15px;")
            layout.addStretch()
            layout.addWidget(self.label)
            layout.addSpacing(10)
            layout.addWidget(self.avatar)
        else:
            self.avatar.setText("🤖")
            self.label.setStyleSheet("background-color: #ffffff; color: black; border-radius: 10px; padding: 15px;")
            layout.addWidget(self.avatar)
            layout.addSpacing(10)
            layout.addWidget(self.label)
            layout.addStretch()

class VirtualKeyboard(QWidget):
    keyPressed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.is_uppercase = False
        self.is_symbols = False
        self.setFixedHeight(280)
        self.setStyleSheet("background-color: #d1d5db; border-top: 1px solid #9ca3af;")
        self.init_ui()
        
    def init_ui(self):
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(5, 10, 5, 10)
        self.main_layout.setSpacing(8)
        self.create_keys()
        
    def create_keys(self):
        # Clear existing keys
        while self.main_layout.count():
            item = self.main_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
            elif item.layout():
                while item.layout().count():
                    subitem = item.layout().takeAt(0)
                    if subitem.widget():
                        subitem.widget().deleteLater()
                item.layout().deleteLater()
                
        if not self.is_symbols:
            rows = [
                ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'],
                ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l'],
                ['SHIFT', 'z', 'x', 'c', 'v', 'b', 'n', 'm', 'BACK'],
                ['123', ',', 'SPACE', '.', 'ENTER', 'HIDE']
            ]
        else:
            rows = [
                ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
                ['-', '/', ':', ';', '(', ')', '$', '&', '@', '"'],
                ['ABC', '[', ']', '{', '}', '#', '%', '^', 'BACK'],
                ['123', '_', 'SPACE', '=', 'ENTER', 'HIDE']
            ]
            
        for row_idx, row in enumerate(rows):
            h_layout = QHBoxLayout()
            h_layout.setSpacing(8)
            h_layout.setContentsMargins(0, 0, 0, 0)
            
            if row_idx == 1:
                h_layout.addSpacing(30)
            
            for key in row:
                display_text = key
                if not self.is_symbols and len(key) == 1 and self.is_uppercase:
                    display_text = key.upper()
                    
                btn = QPushButton(display_text)
                btn.setFont(QFont("Arial", 16, QFont.Bold))
                btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                
                style = """
                    QPushButton {
                        background-color: #ffffff;
                        color: #000000;
                        border-radius: 8px;
                        border-bottom: 3px solid #9ca3af;
                    }
                    QPushButton:pressed {
                        background-color: #e5e7eb;
                        border-bottom: 1px solid #9ca3af;
                        margin-top: 2px;
                    }
                """
                
                if key in ['SHIFT', 'BACK', '123', 'ABC', 'ENTER', 'HIDE']:
                    style = """
                        QPushButton {
                            background-color: #9ca3af;
                            color: #000000;
                            border-radius: 8px;
                            border-bottom: 3px solid #6b7280;
                        }
                        QPushButton:pressed {
                            background-color: #d1d5db;
                            border-bottom: 1px solid #6b7280;
                            margin-top: 2px;
                        }
                    """
                    if key == 'SHIFT':
                        btn.setText('⬆' if not self.is_uppercase else '⬇')
                    elif key == 'BACK':
                        btn.setText('⌫ 退格')
                    elif key == 'ENTER':
                        btn.setText('发送')
                        style = """
                            QPushButton {
                                background-color: #10b981;
                                color: #ffffff;
                                border-radius: 8px;
                                border-bottom: 3px solid #059669;
                            }
                            QPushButton:pressed { background-color: #34d399; border-bottom: 1px solid #059669; margin-top: 2px;}
                        """
                    elif key == 'HIDE':
                        btn.setText('⬇ 收起')
                elif key == 'SPACE':
                    btn.setText('空格')
                    
                btn.setStyleSheet(style)
                btn.clicked.connect(lambda checked, k=key: self.on_key_clicked(k))
                h_layout.addWidget(btn)
                
            if row_idx == 1:
                h_layout.addSpacing(30)
                
            self.main_layout.addLayout(h_layout)

    def on_key_clicked(self, key):
        if key == 'SHIFT':
            self.is_uppercase = not self.is_uppercase
            self.create_keys()
        elif key == '123':
            self.is_symbols = True
            self.create_keys()
        elif key == 'ABC':
            self.is_symbols = False
            self.create_keys()
        else:
            if not self.is_symbols and len(key) == 1 and self.is_uppercase:
                self.keyPressed.emit(key.upper())
            else:
                self.keyPressed.emit(key)


class ChatDialog(QDialog):
    def __init__(self, parent=None, ros_worker=None):
        super().__init__(parent)
        self.ros_worker = ros_worker
        self.setWindowTitle("大模型智能助手")
        self.setFixedSize(1024, 600)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        self.setStyleSheet("background-color: #ededed;")
        
        self.init_ui()
        
        if self.ros_worker:
            self.ros_worker.chat_signal.connect(self.on_llm_response)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 1. Top Bar
        top_bar = QFrame()
        top_bar.setFixedHeight(60)
        top_bar.setStyleSheet("background-color: #f3f3f3; border-bottom: 1px solid #d1d1d1;")
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(15, 0, 15, 0)
        
        btn_back = QPushButton("🔙 返回")
        btn_back.setFixedSize(120, 40)
        btn_back.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        btn_back.setStyleSheet("""
            QPushButton { background-color: #ffffff; border: 1px solid #d1d1d1; border-radius: 8px; color: #333; }
            QPushButton:pressed { background-color: #e5e5e5; }
        """)
        btn_back.clicked.connect(self.close)
        
        lbl_title = QLabel("大模型智能交互")
        lbl_title.setFont(QFont("Microsoft YaHei", 18, QFont.Bold))
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setStyleSheet("color: #000000; border: none;")
        
        top_layout.addWidget(btn_back)
        top_layout.addWidget(lbl_title, 1)
        top_layout.addSpacing(120) # Balance title
        
        main_layout.addWidget(top_bar)
        
        # 2. Chat Area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet("QScrollArea { border: none; background-color: #ededed; }")
        
        self.chat_container = QWidget()
        self.chat_container.setStyleSheet("background-color: transparent;")
        self.chat_layout = QVBoxLayout(self.chat_container)
        self.chat_layout.setContentsMargins(15, 15, 15, 15)
        self.chat_layout.setSpacing(15)
        self.chat_layout.addStretch() # Push messages up
        
        self.scroll_area.setWidget(self.chat_container)
        main_layout.addWidget(self.scroll_area, 1)
        
        # 3. Input Area
        input_bar = QFrame()
        input_bar.setFixedHeight(80)
        input_bar.setStyleSheet("background-color: #f3f3f3; border-top: 1px solid #d1d1d1;")
        input_layout = QHBoxLayout(input_bar)
        input_layout.setContentsMargins(20, 15, 20, 15)
        input_layout.setSpacing(15)
        
        self.input_box = ClickableLineEdit()
        self.input_box.setFont(QFont("Microsoft YaHei", 16))
        self.input_box.setStyleSheet("QLineEdit { background-color: #ffffff; border: 1px solid #d1d1d1; border-radius: 8px; padding: 5px 15px; color: #000; }")
        self.input_box.setPlaceholderText("触摸此处打开键盘...")
        self.input_box.clicked.connect(self.show_keyboard)
        
        self.btn_send = QPushButton("发送")
        self.btn_send.setFixedSize(100, 50)
        self.btn_send.setFont(QFont("Microsoft YaHei", 16, QFont.Bold))
        self.btn_send.setStyleSheet("""
            QPushButton { background-color: #07c160; color: white; border-radius: 8px; border: none; }
            QPushButton:pressed { background-color: #06ad56; }
        """)
        self.btn_send.clicked.connect(self.send_message)
        
        input_layout.addWidget(self.input_box, 1)
        input_layout.addWidget(self.btn_send)
        
        main_layout.addWidget(input_bar)
        
        # 4. Virtual Keyboard
        self.keyboard = VirtualKeyboard()
        self.keyboard.keyPressed.connect(self.handle_key)
        self.keyboard.setVisible(False)
        main_layout.addWidget(self.keyboard)

        # 初始打个招呼
        QTimer.singleShot(500, lambda: self.on_llm_response("你好！我是您的随车大模型智能助手，有什么我可以帮您的吗？"))

    def show_keyboard(self):
        self.keyboard.setVisible(True)
        QTimer.singleShot(100, self.scroll_to_bottom)

    def hide_keyboard(self):
        self.keyboard.setVisible(False)

    def handle_key(self, key):
        if key == 'HIDE':
            self.hide_keyboard()
        elif key == 'ENTER':
            self.send_message()
            self.hide_keyboard()
        elif key == 'BACK':
            current_text = self.input_box.text()
            self.input_box.setText(current_text[:-1])
        elif key == 'SPACE':
            self.input_box.insert(" ")
        else:
            self.input_box.insert(key)

    def send_message(self):
        text = self.input_box.text().strip()
        if not text:
            return
            
        self.input_box.clear()
        
        # UI Add user bubble
        bubble = ChatBubble(text, is_user=True)
        self.chat_layout.insertWidget(self.chat_layout.count() - 1, bubble)
        QTimer.singleShot(50, self.scroll_to_bottom)
        
        # Send via ROS
        if self.ros_worker:
            self.ros_worker.send_prompt(text)

    @pyqtSlot(str)
    def on_llm_response(self, text):
        # 过滤多余标签
        clean_text = text.replace("<final>", "").replace("</final>", "").strip()
        if not clean_text or "<tool_call>" in clean_text:
            return
            
        bubble = ChatBubble(clean_text, is_user=False)
        self.chat_layout.insertWidget(self.chat_layout.count() - 1, bubble)
        QTimer.singleShot(50, self.scroll_to_bottom)

    def scroll_to_bottom(self):
        scrollbar = self.scroll_area.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

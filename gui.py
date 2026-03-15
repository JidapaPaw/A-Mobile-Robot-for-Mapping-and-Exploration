import sys
import json
import os
from PyQt5.QtWidgets import (
    QApplication, QDialog, QFrame, QTabWidget, QWidget, QPushButton,
    QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QScrollArea, QSizePolicy, QStyle,
    QInputDialog, QMessageBox, QLineEdit
)
from PyQt5.QtGui import QIcon, QFont, QColor, QPalette, QMovie
from PyQt5.QtCore import Qt, QSize, QTimer, QEvent

# --- Stylesheets ---
STYLES = {
    "main": """
        QDialog {
            background-color: #1e1e1e;
            color: #ffffff;
            font-family: "Segoe UI", Arial, sans-serif;
        }
        QLabel { color: #e0e0e0; }
        QPushButton {
            background-color: #333333;
            border: 1px solid #444444;
            border-radius: 6px;
            color: white;
            padding: 8px 15px;
            font-size: 14px;
        }
        QPushButton:hover {
            background-color: #444444;
            border: 1px solid #0078d7;
        }
        QPushButton:pressed { background-color: #0078d7; }
        QScrollBar:vertical {
            border: none;
            background: #2b2b2b;
            width: 10px;
            margin: 0px;
        }
        QScrollBar::handle:vertical {
            background: #555;
            min-height: 20px;
            border-radius: 5px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
    """,
    "tab_widget": """
        QTabWidget::pane {
            border: 1px solid #444;
            background-color: #2b2b2b;
            border-radius: 5px;
        }
        QTabBar::tab {
            background-color: #3a3a3a;
            color: #aaa;
            padding: 10px 20px;
            border-top-left-radius: 5px;
            border-top-right-radius: 5px;
            margin-right: 2px;
            font-weight: bold;
        }
        QTabBar::tab:selected {
            background-color: #2b2b2b;
            color: #0078d7;
            border-bottom: 2px solid #0078d7;
        }
        QTabBar::tab:hover { background-color: #4a4a4a; }
    """,
    "map_frame": """
        QFrame#frame {
            background-color: #000000;
            border: 3px dashed #0078d7;
            border-radius: 8px;
            background-image:
                linear-gradient(to right, rgba(255,255,255,0.1) 1px, transparent 1px),
                linear-gradient(to bottom, rgba(255,255,255,0.1) 1px, transparent 1px);
            background-size: 40px 40px;
        }
    """,
    "dpad_btn": """
        QPushButton {
            background-color: #3a3a3a;
            border: 2px solid #555555;
            border-radius: 15px;
            font-size: 36px;
            padding: 15px;
        }
        QPushButton:hover {
            background-color: #0078d7;
            border-color: #00aaff;
        }
        QPushButton:pressed { background-color: #005a9e; }
    """
}

class ScreensaverWidget(QWidget):
    def __init__(self, parent=None, gif_path=""):
        super().__init__(parent)
        self.setStyleSheet("background-color: black;")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.label)
        
        self.movie = QMovie(gif_path)
        if self.movie.isValid():
            self.label.setMovie(self.movie)
            self.movie.start()
            self.label.setScaledContents(True) # ให้ GIF ขยายเต็มจอ

class AMRControlDialog(QDialog):
    def __init__(self):
        super().__init__()
        self._init_window()
        self._init_data()
        self._init_ui()
        self._init_screensaver()

    def _init_data(self):
        self.config_file = '/home/senseri/ros2_ws/config.json'
        self.locations = []
        self.load_config()

    def _init_screensaver(self):
        self.screensaver = ScreensaverWidget(self, "/home/senseri/ros2_ws/frogui.gif")
        self.screensaver.hide()
        
        self.inactivity_timer = QTimer(self)
        self.inactivity_timer.setInterval(60000) 
        self.inactivity_timer.timeout.connect(self.show_screensaver)
        self.inactivity_timer.start()
        
        QApplication.instance().installEventFilter(self)
        self.show_screensaver()

    def _init_window(self):
        self.setWindowTitle("AMR Control Center")
        self.setWindowState(Qt.WindowMaximized)
        self.setWindowFlags(self.windowFlags() | Qt.WindowMinMaxButtonsHint)
        self.setStyleSheet(STYLES["main"])

    def resizeEvent(self, event):
        """ปรับขนาด Screensaver ให้เต็มจอเสมอเมื่อมีการย่อ/ขยายหน้าต่าง"""
        self.screensaver.resize(self.size())
        super().resizeEvent(event)

    def eventFilter(self, source, event):
        """ดักจับ Event เพื่อรีเซ็ตเวลา หรือปิด Screensaver"""
        if event.type() in (QEvent.MouseMove, QEvent.MouseButtonPress, QEvent.KeyPress, QEvent.Wheel):
            if self.screensaver.isVisible():
                # ถ้า Screensaver เปิดอยู่ และมีการคลิกเมาส์ -> ปิด Screensaver
                if event.type() == QEvent.MouseButtonPress:
                    self.screensaver.hide()
                    self.reset_inactivity_timer()
                    return True # Consume event (ไม่ส่งต่อให้ปุ่มข้างล่างทำงาน)
                return True # Block event อื่นๆ ระหว่างที่ Screensaver บังอยู่
            else:
                # ถ้าใช้งานปกติ -> รีเซ็ตเวลานับถอยหลัง
                self.reset_inactivity_timer()
        
        return super().eventFilter(source, event)

    def reset_inactivity_timer(self):
        self.inactivity_timer.start(60000)

    def show_screensaver(self):
        self.screensaver.raise_() # ดึงมาไว้ชั้นบนสุด
        self.screensaver.resize(self.size())
        self.screensaver.show()

    def load_config(self):
        """โหลดข้อมูล Location จากไฟล์ JSON"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    data = json.load(f)
                    self.locations = data.get('locations', [])
            except Exception as e:
                print(f"Error loading config: {e}")
                self.locations = []
        else:
            self.locations = []

    def save_config(self):
        """บันทึกข้อมูล Location ลงไฟล์ JSON"""
        data = {'locations': self.locations}
        try:
            with open(self.config_file, 'w') as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Could not save config: {e}")

    def _init_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        content_widget = QWidget()
        main_layout = QHBoxLayout(content_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(20)

        # --- Left Panel ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)

        # 1.1 ปุ่มควบคุมแผนที่ (แยกออกมาอยู่นอกกรอบ Map ด้านบน)
        map_controls_layout = QHBoxLayout()
        map_controls_layout.setAlignment(Qt.AlignLeft)
        
        self.btn_save_map = QPushButton("Save Map")
        self.btn_save_map.setObjectName("pushsave")
        self.btn_save_map.setFixedSize(120, 40)
        self.btn_save_map.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        
        self.btn_load_map = QPushButton("Load Map")
        self.btn_load_map.setObjectName("pushload")
        self.btn_load_map.setFixedSize(120, 40)
        self.btn_load_map.setIcon(self.style().standardIcon(QStyle.SP_DialogOpenButton))

        map_controls_layout.addWidget(self.btn_save_map)
        map_controls_layout.addWidget(self.btn_load_map)
        left_layout.addLayout(map_controls_layout)

        # Map Frame
        self.map_frame = QFrame()
        self.map_frame.setObjectName("frame")
        self.map_frame.setFrameShape(QFrame.StyledPanel)
        self.map_frame.setFrameShadow(QFrame.Raised)
        self.map_frame.setStyleSheet(STYLES["map_frame"])
        self.map_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        map_inner_layout = QVBoxLayout(self.map_frame)
        self.lbl_map_placeholder = QLabel("WAITING FOR MAP DATA...\n(ROS 2 Topic: /map)")
        self.lbl_map_placeholder.setAlignment(Qt.AlignCenter)
        self.lbl_map_placeholder.setStyleSheet("color: rgba(255, 255, 255, 0.3); font-size: 20px; font-weight: bold;")
        map_inner_layout.addWidget(self.lbl_map_placeholder)

        left_layout.addWidget(self.map_frame)
        main_layout.addWidget(left_panel, stretch=2)

        # --- Right Panel (Tabs) ---
        self.tab_widget = QTabWidget()
        self.tab_widget.setObjectName("tabWidget")
        self.tab_widget.setStyleSheet(STYLES["tab_widget"])

        self.tab_manual = QWidget()
        self.setup_manual_tab()
        self.tab_widget.addTab(self.tab_manual, "MANUAL CONTROL")

        # ตั้งค่า Tab 2: AUTOMATION [cite: 3]
        self.tab_auto = QWidget()
        self.setup_automation_tab()
        self.tab_widget.addTab(self.tab_auto, "AUTOMATION")
        
        main_layout.addWidget(self.tab_widget, stretch=1)
        root_layout.addWidget(content_widget)

        # --- Status Bar ---
        self.status_bar = QFrame()
        self.status_bar.setFixedHeight(30)
        self.status_bar.setStyleSheet("background-color: #0078d7; color: white; font-weight: bold;")
        status_layout = QHBoxLayout(self.status_bar)
        status_layout.setContentsMargins(10, 0, 10, 0)
        
        self.lbl_status = QLabel("STATUS: READY")
        self.lbl_battery = QLabel("BATTERY: 100%")
        status_layout.addWidget(self.lbl_status)
        status_layout.addStretch()
        status_layout.addWidget(self.lbl_battery)
        
        root_layout.addWidget(self.status_bar)

    def setup_manual_tab(self):
        layout = QVBoxLayout(self.tab_manual)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(30)

        # หัวข้อ
        title_label = QLabel("MANUAL CONTROL")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        dpad_layout = QGridLayout()
        dpad_layout.setSpacing(5)

        dpad_btn_style = STYLES["dpad_btn"]
        
        stop_btn_style = dpad_btn_style + """
            QPushButton {
                background-color: #d9534f;
                border-radius: 30px;
                font-size: 20px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #c9302c; }
        """

        btn_up = QPushButton("↑"); btn_up.setStyleSheet(dpad_btn_style)
        btn_left = QPushButton("←"); btn_left.setStyleSheet(dpad_btn_style)
        btn_stop = QPushButton("STOP"); btn_stop.setFixedSize(80, 80); btn_stop.setStyleSheet(stop_btn_style)
        btn_right = QPushButton("→"); btn_right.setStyleSheet(dpad_btn_style)
        btn_down = QPushButton("↓"); btn_down.setStyleSheet(dpad_btn_style)

        dpad_layout.addWidget(btn_up, 0, 1)
        dpad_layout.addWidget(btn_left, 1, 0)
        dpad_layout.addWidget(btn_stop, 1, 1)
        dpad_layout.addWidget(btn_right, 1, 2)
        dpad_layout.addWidget(btn_down, 2, 1)

        layout.addLayout(dpad_layout)

        speed_layout = QVBoxLayout()
        speed_layout.setSpacing(10)
        btn_inc_speed = QPushButton("Increase Speed")
        btn_dec_speed = QPushButton("Decrease Speed")
        
        speed_btn_style = """
            QPushButton {
                padding: 12px;
                font-size: 18px;
                border-radius: 20px;
                border: 1px solid #0078d7;
            }
            QPushButton:hover { background-color: rgba(0, 120, 215, 0.2); }
        """
        btn_inc_speed.setStyleSheet(speed_btn_style)
        btn_dec_speed.setStyleSheet(speed_btn_style)
        speed_layout.addWidget(btn_inc_speed)
        speed_layout.addWidget(btn_dec_speed)

        layout.addLayout(speed_layout)
        layout.addStretch() # ดันทุกอย่างขึ้นด้านบน


    def setup_automation_tab(self):
        layout = QVBoxLayout(self.tab_auto)
        layout.setSpacing(20)
        layout.setContentsMargins(20, 20, 20, 20)

        # หัวข้อ
        title_label = QLabel("AUTOMATION")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        self.btn_save_loc = QPushButton("+  SAVE CURRENT LOCATION AS...")
        self.btn_save_loc.setStyleSheet("""
            QPushButton {
                background-color: #0078d7;
                color: white;
                border: none;
                border-radius: 25px;
                font-size: 14px;
                font-weight: bold;
                padding: 15px;
            }
            QPushButton:hover { background-color: #00aaff; }
            QPushButton:pressed { background-color: #005a9e; }
        """)
        self.btn_save_loc.clicked.connect(self.add_location)
        layout.addWidget(self.btn_save_loc)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")

        self.location_list_widget = QWidget()
        self.location_list_widget.setStyleSheet("background-color: transparent;")
        self.location_list_layout = QVBoxLayout(self.location_list_widget)
        self.location_list_layout.setSpacing(10)
        self.location_list_layout.setAlignment(Qt.AlignTop)

        self.refresh_location_list()

        scroll_area.setWidget(self.location_list_widget)
        layout.addWidget(scroll_area)

    def refresh_location_list(self):
        """สร้างรายการปุ่ม Location ใหม่จาก self.locations"""
        # ลบ Widget เก่าทิ้งให้หมด
        for i in reversed(range(self.location_list_layout.count())):
            widget = self.location_list_layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()

        pin_icon = self.style().standardIcon(QStyle.SP_FileIcon)
        trash_icon = self.style().standardIcon(QStyle.SP_TrashIcon)

        for loc in self.locations:
            loc_name = loc['name']
            
            # สร้าง Container แนวนอนสำหรับแต่ละแถว
            row_widget = QWidget()
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(5)

            # ปุ่มไปที่ Location (ปุ่มหลัก)
            btn_go = QPushButton(f"  {loc_name}")
            btn_go.setIcon(pin_icon)
            btn_go.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            btn_go.setStyleSheet("""
                QPushButton {
                    background-color: #333333;
                    border: 1px solid #555;
                    border-radius: 10px;
                    text-align: left;
                    padding: 10px 15px;
                    font-size: 14px;
                    color: white;
                }
                QPushButton:hover { background-color: #444; border-color: #0078d7; }
            """)
            # TODO: เชื่อมต่อฟังก์ชันเดินรถจริงที่นี่
            btn_go.clicked.connect(lambda checked, n=loc_name: print(f"Going to {n}"))

            # ปุ่มลบ (ปุ่มเล็กขวาสุด)
            btn_del = QPushButton()
            btn_del.setIcon(trash_icon)
            btn_del.setFixedSize(40, 40)
            btn_del.setStyleSheet("""
                QPushButton {
                    background-color: #d9534f;
                    border-radius: 10px;
                }
                QPushButton:hover { background-color: #c9302c; }
            """)
            btn_del.clicked.connect(lambda checked, n=loc_name: self.delete_location(n))

            row_layout.addWidget(btn_go)
            row_layout.addWidget(btn_del)
            
            self.location_list_layout.addWidget(row_widget)

    def add_location(self):
        text, ok = QInputDialog.getText(self, 'Save Location', 'Enter location name:')
        if ok and text:
            # TODO: ดึงพิกัดจริงจาก Robot Odometry
            new_loc = {"name": text, "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            self.locations.append(new_loc)
            self.save_config()
            self.refresh_location_list()

    def delete_location(self, name):
        reply = QMessageBox.question(self, 'Confirm Delete', 
                                     f"Are you sure you want to delete '{name}'?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.locations = [loc for loc in self.locations if loc['name'] != name]
            self.save_config()
            self.refresh_location_list()

if __name__ == '__main__':
    from PyQt5.QtWidgets import QApplication, QStyle
    app = QApplication(sys.argv)
    dialog = AMRControlDialog()
    dialog.show()
    sys.exit(app.exec_())
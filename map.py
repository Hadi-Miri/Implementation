from PyQt5 import QtWidgets, QtGui, QtCore
import sys
import os

class DrivingScreen(QtWidgets.QWidget):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack
        self.dark_mode = False
        self.init_ui()
        self.setFixedSize(1000,750)

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        
        # TOP BAR SETUP
        top_bar = QtWidgets.QHBoxLayout()
        
        self.home_button = QtWidgets.QPushButton()
        self.home_button.setIcon(self.load_icon("home.png"))
        self.home_button.setIconSize(QtCore.QSize(32,32))
        self.home_button.setFixedSize(40,40)
        self.home_button.setStyleSheet("border: none; background: transparent;")
        self.home_button.clicked.connect(self.go_home)
        
        self.exit_button = QtWidgets.QPushButton("Exit the Platoon")
        self.exit_button.setFixedSize(200,40)
        self.exit_button.setStyleSheet("background-color: red; font-size: 16px; font-weight: bold; color: white;")
        
        self.stop_button = QtWidgets.QPushButton()
        self.stop_button.setIcon(self.load_icon("stop.png"))
        self.stop_button.setFixedSize(32,32)
        self.stop_button.setStyleSheet("border: none; background: transparent; margin-right: 5px;")
        self.stop_button.clicked.connect(self.stop_action)
        
        self.theme_button = QtWidgets.QPushButton()
        self.theme_button.setIcon(self.load_icon("b.png"))
        self.theme_button.setFixedSize(32,32)
        self.theme_button.setStyleSheet("border: none; background: transparent;")
        self.theme_button.clicked.connect(self.toggle_theme)
        
        top_bar.addWidget(self.home_button)
        top_bar.addStretch()
        top_bar.addWidget(self.exit_button)
        top_bar.addStretch()
        top_bar.addWidget(self.stop_button)
        top_bar.addWidget(self.theme_button)
        layout.addLayout(top_bar)
        
        # LOGO
        logo_label = QtWidgets.QLabel()
        logo_pixmap = self.load_pixmap("logo.png",200,200)
        logo_label.setPixmap(logo_pixmap)
        logo_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(logo_label)
        
        # TITLE LABEL WITH ANIMATION
        self.title_label = QtWidgets.QLabel("Driving to the Platoon")
        self.title_label.setAlignment(QtCore.Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 26px; font-weight: bold; background: transparent;")
        layout.addWidget(self.title_label)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_title_animation)
        self.timer.start(500)
        self.dot_count = 0

        # MAP CONTAINER SETUP
        # Instead of a plain QLabel, we create a container widget where we overlay the marker.
        self.map_container = QtWidgets.QWidget()
        self.map_container.setFixedSize(400,400)
        self.map_container.setStyleSheet("background: transparent;")
        
        # Create a label to show the map
        self.map_label = QtWidgets.QLabel(self.map_container)
        map_pixmap = self.load_pixmap("map.png",400,400)
        self.map_label.setPixmap(map_pixmap)
        self.map_label.setGeometry(0, 0, 400, 400)
        
        # Create the marker label (initially labeled "Coburg")
        self.marker_label = QtWidgets.QLabel(self.map_container)
        self.marker_label.setText("Coburg")
        self.marker_label.setStyleSheet("""
            background-color: rgba(255,255,255,0.8);
            border: 1px solid black;
            border-radius: 5px;
            padding: 2px;
            """)
        self.marker_label.adjustSize()
        # Set an initial dummy position (for example, 100,100)
        self.marker_label.move(100, 100)
        
        # Add the map container to the main layout
        layout.addWidget(self.map_container, alignment=QtCore.Qt.AlignCenter)
        
        self.setLayout(layout)
        self.setWindowTitle("RestLink Driving Page")
        self.apply_theme()
        
        # --- Removed the dummy marker timer ---
        # Previously, a timer called update_marker() every 500ms was used to simulate movement.
        # That logic is now removed in favor of real /ego_pose updates via update_position().

    def update_title_animation(self):
        dots = "." * ((self.dot_count % 3) + 1)
        self.title_label.setText(f"Driving to the Platoon{dots}")
        self.dot_count += 1

    @QtCore.pyqtSlot(float, float)
    def update_position(self, real_x, real_y):
        """
        Slot to update the marker's position using real x,y coordinates.
        Only updates when this layout is active.
        """
        # Check if this widget is currently visible (i.e. active layout)
        if not self.isVisible():
            return

        # Convert real-world coordinates to map pixel coordinates.
        pixel_x, pixel_y = self.transform_coordinates(real_x, real_y)
        self.marker_label.move(pixel_x, pixel_y)
        self.marker_label.setText(f"🚗 ({pixel_x}, {pixel_y})")
        self.marker_label.adjustSize()

    def transform_coordinates(self, real_x, real_y):
        """
        Converts real-world coordinates (assumed within an 800x800 unit space)
        to pixel coordinates on the map image (400x400 pixels). Also inverts the Y-axis.
        """
        scale_factor = 400 / 800  # i.e. 0.5
        pixel_x = real_x * scale_factor
        # Invert the y-axis: assuming real_y=0 is at the bottom and pixel_y=0 is at the top.
        pixel_y = 400 - (real_y * scale_factor)
        return int(pixel_x), int(pixel_y)

    def go_home(self):
        print("Home button clicked")

    def toggle_theme(self):
        self.dark_mode = not self.dark_mode
        self.apply_theme()

    def stop_action(self):
        print("Stop button clicked")

    def apply_theme(self):
        if self.dark_mode:
            self.stack.setStyleSheet("background: #222; color: white;")
        else:
            self.stack.setStyleSheet("background: white; color: black;")
        self.theme_button.setIcon(self.load_icon("b.png"))

    def load_icon(self, filename):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(base_dir, filename)
        return QtGui.QIcon(full_path) if os.path.exists(full_path) else QtGui.QIcon()

    def load_pixmap(self, filename, width, height):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(base_dir, filename)
        pixmap = QtGui.QPixmap(full_path)
        if pixmap.isNull():
            print(f"Warning: Could not load pixmap from {full_path}")
        else:
            pixmap = pixmap.scaled(width, height, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        return pixmap

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    stack = QtWidgets.QStackedWidget()
    driving_screen = DrivingScreen(stack)
    stack.addWidget(driving_screen)
    stack.show()
    sys.exit(app.exec_())


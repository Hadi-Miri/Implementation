from PyQt5 import QtWidgets, QtGui, QtCore
import sys
import os

class MainMenu(QtWidgets.QWidget):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack
        self.dark_mode = False  # Theme state
        self.init_ui()
        self.setFixedSize(1000, 750)  # Further increased window size for better spacing

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Top-right theme toggle button
        self.theme_button = QtWidgets.QPushButton()
        self.theme_button.setIcon(self.load_icon("b.png"))  # Load icon safely
        self.theme_button.setFixedSize(32, 32)
        self.theme_button.setStyleSheet("border: none; background: transparent;")
        self.theme_button.clicked.connect(self.toggle_theme)

        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addStretch()
        top_bar.addWidget(self.theme_button)
        layout.addLayout(top_bar)

        # Title Label
        label = QtWidgets.QLabel("Welcome to RestLink\nPlease Choose Your Status")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-size: 26px; font-weight: bold; background: transparent;")
        layout.addWidget(label)

        # Logo
        logo_label = QtWidgets.QLabel()
        logo_pixmap = self.load_pixmap("logo.png", 350, 350)  # Moved logo up and increased size
        logo_label.setPixmap(logo_pixmap)
        logo_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(logo_label)

        # Buttons Layout with Images
        button_layout = ButtonLayoutWidget(self.stack, self)
        layout.addWidget(button_layout)

        self.setLayout(layout)
        self.setWindowTitle("RestLink Main Menu")
        self.apply_theme()

    def toggle_theme(self):
        """Toggle dark/light mode for the whole UI."""
        self.dark_mode = not self.dark_mode
        self.apply_theme()

    def apply_theme(self):
        """Applies the current theme to all elements properly."""
        if self.dark_mode:
            self.setStyleSheet("background: #222; color: white;")
        else:
            self.setStyleSheet("background: white; color: black;")
        self.theme_button.setIcon(self.load_icon("b.png"))

    def load_icon(self, filename):
        """Load an icon safely, returning an empty icon if the file is missing."""
        return QtGui.QIcon(filename) if os.path.exists(filename) else QtGui.QIcon()

    def load_pixmap(self, filename, width, height):
        """Load an image safely, returning a blank pixmap if missing."""
        pixmap = QtGui.QPixmap()
        if os.path.exists(filename):
            pixmap.load(filename)
            return pixmap.scaled(width, height, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        return QtGui.QPixmap()

class ButtonLayoutWidget(QtWidgets.QWidget):
    """Separate widget for button layout with larger images."""
    def __init__(self, stack, parent):
        super().__init__()
        self.stack = stack
        self.parent = parent
        self.setStyleSheet("background: transparent;")

        button_layout = QtWidgets.QHBoxLayout(self)

        # Follower Vehicle
        self.follower_layout = QtWidgets.QVBoxLayout()
        self.follower_image = QtWidgets.QLabel()
        self.follower_image.setPixmap(self.load_pixmap("following_vehicle.png", 350, 250))  # Enlarged image
        self.follower_image.setAlignment(QtCore.Qt.AlignCenter)
        self.follower_button = QtWidgets.QPushButton("Follower Vehicle")
        self.follower_button.setFixedHeight(60)
        self.follower_button.setStyleSheet("background-color: grey; font-size: 18px;")
        self.follower_button.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        self.follower_layout.addWidget(self.follower_image)
        self.follower_layout.addWidget(self.follower_button)

        # Lead Vehicle
        self.lead_layout = QtWidgets.QVBoxLayout()
        self.lead_image = QtWidgets.QLabel()
        self.lead_image.setPixmap(self.load_pixmap("lead_vehicle.png", 350, 250))  # Enlarged image
        self.lead_image.setAlignment(QtCore.Qt.AlignCenter)
        self.lead_button = QtWidgets.QPushButton("Lead Vehicle")
        self.lead_button.setFixedHeight(60)
        self.lead_button.setStyleSheet("background-color: grey; font-size: 18px;")
        self.lead_button.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        self.lead_layout.addWidget(self.lead_image)
        self.lead_layout.addWidget(self.lead_button)

        button_layout.addLayout(self.follower_layout)
        button_layout.addLayout(self.lead_layout)
        self.setLayout(button_layout)

    def load_pixmap(self, filename, width, height):
        """Load an image safely, returning a blank pixmap if missing."""
        pixmap = QtGui.QPixmap()
        if os.path.exists(filename):
            pixmap.load(filename)
            return pixmap.scaled(width, height, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        return QtGui.QPixmap()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    stack = QtWidgets.QStackedWidget()
    main_menu = MainMenu(stack)
    stack.addWidget(main_menu)

    stack.show()
    sys.exit(app.exec_())

#!/usr/bin/env python3
import sys
import threading
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from custom_msgs.msg import Platoon
from geometry_msgs.msg import Pose, Quaternion, PoseStamped  # Added PoseStamped

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QPropertyAnimation, QEasingCurve, QTimer

# -- Layout Classes --
from restlink_ui.app.app import ClickableRestLinkUI
from restlink_ui.home.home import MainMenu
from restlink_ui.destination.destination import DestinationMenu
from restlink_ui.lead_vehicle.lead_vehicle import TruckJoinRequestScreen
from restlink_ui.loading.loading import LoadingPage
from restlink_ui.platoon.platoon import PlatoonMenu
from restlink_ui.map.map import DrivingScreen
from restlink_ui.exit.exit import ExitingScreen
from restlink_ui.done.done import OutOfPlatoonScreen
from restlink_ui.acceptance.acceptance import AcceptanceScreen
from restlink_ui.accepted.accepted import AcceptedScreen

# --------------------------------------------------------------------
# Create a Qt signal emitter to safely send ROS data to the UI thread.
class PoseSignalEmitter(QtCore.QObject):
    new_pose_signal = QtCore.pyqtSignal(float, float)  # x and y coordinates

    def __init__(self, parent=None):
        super().__init__(parent)

# --------------------------------------------------------------------
class RestLinkUINode(Node):
    """
    ROS2 node for the RestLink UI.
    Publishes topics like /hv_destination, /platoon_id, etc.
    Subscribes to topics like /platoons_list, /lv_acceptance.
    **New:** Subscribes to /ego_pose (PoseStamped) and emits Qt signals with real pose.
    """
    def __init__(self):
        super().__init__('restlink_ui_node')

        # Publishers
        self.destination_publisher = self.create_publisher(Pose, '/hv_destination', 10)
        self.destination_pose_publisher = self.create_publisher(Pose, '/destination_pose', 10)
        self.is_destination_publisher = self.create_publisher(Bool, '/is_destination', 10)
        self.platoon_found_publisher = self.create_publisher(Bool, '/is_platoon_found', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)
        self.exit_platoon_publisher = self.create_publisher(Bool, '/exit_platoon', 10)
        self.platoon_id_publisher = self.create_publisher(Int32, '/platoon_id', 10)

        # Subscribers (existing)
        self.create_subscription(Platoon, '/platoons_list', self.platoon_list_callback, 10)
        self.create_subscription(Bool, '/lv_acceptance', self.lv_acceptance_callback, 10)

        # New subscriber for /ego_pose
        self.create_subscription(PoseStamped, '/ego_pose', self.ego_pose_callback, 10)

        # Internal states
        self.platoons_received = False
        self.platoon_list = None
        self.lv_acceptance_value = None

        # A dictionary of valid city -> (x,y,z) coordinates
        self.valid_destinations = {
            "kronach":      (0.5, 6.0, 0.0),
            "wurzburg":     (0.5, 1.5, 0.0),
            "bamberg":      (0.5, 2.0, 0.0),
            "coburg":       (6.5, 2.5, 0.0),
            "lichtenfels":  (3.0, 6.0, 0.0),
        }

    def platoon_list_callback(self, msg):
        self.platoon_list = msg
        self.platoons_received = True
        self.get_logger().info(f"Received platoons: {self.platoon_list}")

    def lv_acceptance_callback(self, msg):
        self.lv_acceptance_value = msg.data
        self.get_logger().info(f"Received /lv_acceptance: {self.lv_acceptance_value}")

    def publish_message(self, publisher, message):
        self.get_logger().info(f"Publishing: {message} to topic: {publisher.topic_name}")
        publisher.publish(message)

    def ego_pose_callback(self, msg):
        """
        Callback for /ego_pose messages.
        Extracts x and y from msg.pose.position and emits them via a Qt signal.
        """
        real_x = msg.pose.position.x
        real_y = msg.pose.position.y
        self.get_logger().info(f"Received /ego_pose: x={real_x}, y={real_y}")
        if hasattr(self, 'pose_emitter'):
            self.pose_emitter.new_pose_signal.emit(real_x, real_y)
        else:
            self.get_logger().warning("pose_emitter not set; cannot emit new pose signal.")

# --------------------------------------------------------------------
class MainApplication(QtWidgets.QMainWindow):
    """
    The main UI application that orchestrates all layouts and ROS communication.
    """
    def __init__(self, ros_node: RestLinkUINode):
        super(MainApplication, self).__init__()
        self.ros_node = ros_node
        self.last_pose = None

        self.stack = QtWidgets.QStackedWidget()
        self.setCentralWidget(self.stack)
        self.setWindowTitle("RestLink UI")

        self.last_layout_key = None
        self.launch_started = False
        self.animations = []
        self.animation_duration = 500

        # Timer for continuous platoon_id publishing (not started by default)
        self.platoon_id_timer = QTimer(self)
        self.platoon_id_timer.timeout.connect(self.publish_platoon_id)

        # New timer for continuously publishing /is_destination and /is_platoon_found as False
        self.find_new_platoon_timer = QTimer(self)
        self.find_new_platoon_timer.timeout.connect(self.publish_find_new_platoon_msgs)

        # Create pages
        self.pages = {
            "app":          ClickableRestLinkUI(self.stack),
            "home":         MainMenu(self.stack),
            "destination":  DestinationMenu(self.stack),
            "lead_vehicle": TruckJoinRequestScreen(self.stack),
            "loading":      LoadingPage(self.stack),
            "platoon":      PlatoonMenu(self.stack),
            "map":          DrivingScreen(self.stack),  # Map UI with updated marker logic
            "exit":         ExitingScreen(self.stack),
            "done":         OutOfPlatoonScreen(self.stack),
            "acceptance":   AcceptanceScreen(self.stack),
            "accepted":     AcceptedScreen(self.stack),
        }

        for p in self.pages.values():
            self.stack.addWidget(p)
        self.stack.setCurrentWidget(self.pages["app"])

        # Navigation signals
        self.pages["app"].restlink_button.clicked.connect(self.handle_restlink_click)

        if hasattr(self.pages["home"], "follower_button"):
            self.pages["home"].follower_button.clicked.connect(lambda: self.switch_layout("destination"))
        if hasattr(self.pages["home"], "lead_button"):
            self.pages["home"].lead_button.clicked.connect(lambda: self.switch_layout("lead_vehicle"))

        self.pages["destination"].confirm_button.clicked.connect(self.handle_destination_confirm)
        self.pages["platoon"].join_button.clicked.connect(self.handle_join_platoon)
        self.pages["platoon"].find_button.clicked.connect(self.handle_find_new_platoon)
        self.pages["map"].exit_button.clicked.connect(self.handle_exit_sequence)

        # Special rule: lead_vehicle layout => home button goes to "home"
        for page_key, page_obj in self.pages.items():
            if hasattr(page_obj, "home_button"):
                try:
                    page_obj.home_button.clicked.disconnect()
                except Exception:
                    pass
                if page_key == "lead_vehicle":
                    page_obj.home_button.clicked.connect(lambda: self.switch_layout("home"))
                else:
                    page_obj.home_button.clicked.connect(self.handle_go_to_app_layout)

        # Stop buttons => emergency stop
        for page_key, page_obj in self.pages.items():
            if hasattr(page_obj, "stop_button"):
                try:
                    page_obj.stop_button.clicked.disconnect()
                except Exception:
                    pass
                page_obj.stop_button.clicked.connect(self.handle_emergency_stop)

        # Timers
        self.platoon_check_timer = QTimer(self)
        self.platoon_check_timer.timeout.connect(self.check_platoons_received)
        self.platoon_check_timer.start(1000)

        self.acceptance_check_timer = QTimer(self)
        self.acceptance_check_timer.timeout.connect(self.check_lv_acceptance)
        self.acceptance_check_timer.start(1000)

        self.platoon_info_timer = QTimer(self)
        self.platoon_info_timer.timeout.connect(self.update_platoon_display)
        self.platoon_info_timer.start(1000)

    def check_platoons_received(self):
        if self.stack.currentWidget() == self.pages["loading"]:
            if self.ros_node.platoon_list and self.ros_node.platoon_list.id != 0:
                print("Platoon ID is nonzero => go 'platoon'")
                self.switch_layout("platoon")

    def check_lv_acceptance(self):
        if self.stack.currentWidget() == self.pages["acceptance"] and self.ros_node.lv_acceptance_value is not None:
            if self.ros_node.lv_acceptance_value:
                print("lv_acceptance = TRUE => accepted => 3s => map")
                self.switch_layout("accepted")
                QTimer.singleShot(3000, lambda: self.switch_layout("map"))
            else:
                print("lv_acceptance = FALSE => back to 'platoon'")
                self.switch_layout("platoon")
            self.ros_node.lv_acceptance_value = None

    def update_platoon_display(self):
        if self.stack.currentWidget() == self.pages["platoon"]:
            platoon_page = self.pages["platoon"]
            if hasattr(platoon_page, "platoon_list"):
                platoon_page.platoon_list.clear()
                if not self.ros_node.platoon_list:
                    platoon_page.platoon_list.addItem("No platoons received yet.")
                    return
                msg = self.ros_node.platoon_list
                platoon_page.platoon_list.addItem(f"ID: {msg.id}")
                platoon_page.platoon_list.addItem(f"Name: {msg.name}")
                platoon_page.platoon_list.addItem(f"Destination: {msg.destination}")

    def handle_restlink_click(self):
        if not self.launch_started:
            print("Launching rl_launch.py for the first time...")
            subprocess.Popen(["ros2", "launch", "rl_launch", "rl_launch.py"])
            self.launch_started = True
            self.switch_layout("home")
        else:
            if self.last_layout_key and self.last_layout_key in self.pages:
                self.switch_layout(self.last_layout_key)
            else:
                self.switch_layout("home")

    def handle_go_to_app_layout(self):
        current_key = self.get_current_layout_key()
        if current_key != "app":
            self.last_layout_key = current_key
        self.switch_layout("app")

    def handle_destination_confirm(self):
        dest = self.pages["destination"].text_input.text().strip().lower()
        if dest in self.ros_node.valid_destinations:
            print(f"Destination Confirmed: {dest}")
            if self.find_new_platoon_timer.isActive():
                self.find_new_platoon_timer.stop()
                print("Stopped continuous publishing for find new platoon due to destination confirmation")
            self.ros_node.publish_message(
                self.ros_node.is_destination_publisher,
                Bool(data=True)
            )
            x, y, z = self.ros_node.valid_destinations[dest]
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            pose_msg.orientation = Quaternion(w=1.0)
            self.ros_node.publish_message(
                self.ros_node.destination_publisher,
                pose_msg
            )
            self.last_pose = pose_msg
            self.switch_layout("loading")
        else:
            print("Invalid destination. Please enter a valid city.")

    def handle_join_platoon(self):
        print("Join platoon => /is_platoon_found = TRUE => 'acceptance'")
        self.ros_node.publish_message(
            self.ros_node.platoon_found_publisher,
            Bool(data=True)
        )
        if self.ros_node.platoon_list is not None:
            if not self.platoon_id_timer.isActive():
                self.platoon_id_timer.start(500)  # Publish every 500ms.
                print("Started continuous publishing of platoon id.")
        else:
            print("No platoon list available to publish platoon id.")
        self.switch_layout("acceptance")

    def publish_platoon_id(self):
        if self.ros_node.platoon_list is not None:
            platoon_id = self.ros_node.platoon_list.id
            self.ros_node.publish_message(
                self.ros_node.platoon_id_publisher,
                Int32(data=platoon_id)
            )
            print(f"Continuously published platoon id: {platoon_id}")
        else:
            print("No platoon list available for continuous publish.")

    def publish_find_new_platoon_msgs(self):
        self.ros_node.publish_message(
            self.ros_node.is_destination_publisher,
            Bool(data=False)
        )
        self.ros_node.publish_message(
            self.ros_node.platoon_found_publisher,
            Bool(data=False)
        )
        print("Continuously publishing /is_destination and /is_platoon_found as False")

    def stop_platoon_id_timer(self):
        if self.platoon_id_timer.isActive():
            self.platoon_id_timer.stop()
            print("Stopped continuous publishing of platoon id.")

    def handle_find_new_platoon(self):
        print("Find new platoon => reset /is_destination and /is_platoon_found to False and switch to 'destination'")
        if not self.find_new_platoon_timer.isActive():
            self.find_new_platoon_timer.start(500)
            print("Started continuous publishing for finding new platoon")
        self.stop_platoon_id_timer()
        self.switch_layout("destination")

    def handle_exit_sequence(self):
        print("Exit platoon => /exit_platoon=TRUE => 'exit'")
        self.ros_node.publish_message(
            self.ros_node.exit_platoon_publisher,
            Bool(data=True)
        )
        self.ros_node.publish_message(
            self.ros_node.is_destination_publisher,
            Bool(data=False)
        )
        self.stop_platoon_id_timer()
        if self.last_pose:
            print("Publishing /destination_pose with last known city coords")
            self.ros_node.publish_message(
                self.ros_node.destination_pose_publisher,
                self.last_pose
            )
        else:
            print("No stored pose => can't publish /destination_pose")
        self.switch_layout("exit")
        QTimer.singleShot(3000, self.go_to_done_layout)

    def go_to_done_layout(self):
        self.switch_layout("done")
        QTimer.singleShot(5000, lambda: self.switch_layout("home"))

    def handle_emergency_stop(self):
        print("Stop => /emergency_stop=TRUE => 5s => 'app'")
        self.ros_node.publish_message(
            self.ros_node.emergency_stop_publisher,
            Bool(data=True)
        )
        QTimer.singleShot(5000, self.reset_ui_after_emergency)

    def reset_ui_after_emergency(self):
        self.switch_layout("app")
        self.last_layout_key = None
        self.stop_platoon_id_timer()
        if self.find_new_platoon_timer.isActive():
            self.find_new_platoon_timer.stop()
        print("UI memory reset after emergency stop. Next restlink click => 'home'")

    def get_current_layout_key(self):
        current_widget = self.stack.currentWidget()
        for k, w in self.pages.items():
            if w == current_widget:
                return k
        return None

    def switch_layout(self, page_key):
        if page_key not in self.pages:
            print(f"Page '{page_key}' not found.")
            return
        current_widget = self.stack.currentWidget()
        next_widget = self.pages[page_key]
        if current_widget == next_widget:
            return
        if page_key != "app":
            self.last_layout_key = page_key
        self.fade_transition(current_widget, next_widget)
        print(f"Switched to {page_key} page.")
        if page_key != "acceptance":
            self.stop_platoon_id_timer()
        if page_key != "destination" and self.find_new_platoon_timer.isActive():
            self.find_new_platoon_timer.stop()

    def fade_transition(self, current_widget, next_widget):
        fade_effect = QtWidgets.QGraphicsOpacityEffect(current_widget)
        current_widget.setGraphicsEffect(fade_effect)
        fade_out = QPropertyAnimation(fade_effect, b"opacity")
        fade_out.setDuration(self.animation_duration)
        fade_out.setStartValue(1.0)
        fade_out.setEndValue(0.0)
        fade_out.setEasingCurve(QEasingCurve.InOutQuad)
        fade_out.finished.connect(lambda: self.fade_in_transition(next_widget))
        self.animations.append(fade_out)
        fade_out.start()

    def fade_in_transition(self, next_widget):
        self.stack.setCurrentWidget(next_widget)
        fade_effect = QtWidgets.QGraphicsOpacityEffect(next_widget)
        next_widget.setGraphicsEffect(fade_effect)
        fade_in = QPropertyAnimation(fade_effect, b"opacity")
        fade_in.setDuration(self.animation_duration)
        fade_in.setStartValue(0.0)
        fade_in.setEndValue(1.0)
        fade_in.setEasingCurve(QEasingCurve.InOutQuad)
        fade_in.finished.connect(lambda: self.animations.remove(fade_in))
        self.animations.append(fade_in)
        fade_in.start()

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    ros_node = RestLinkUINode()

    # Create and attach the Qt signal emitter for pose updates
    ros_node.pose_emitter = PoseSignalEmitter()

    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    main_window = MainApplication(ros_node)
    main_window.show()

    # Connect the new pose signal to the DrivingScreen's update_position slot.
    # This ensures that whenever a new /ego_pose is received, the marker is updated.
    ros_node.pose_emitter.new_pose_signal.connect(main_window.pages["map"].update_position)

    exit_code = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()


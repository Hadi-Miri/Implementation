# Import necessary libraries
import rclpy  # ROS2 Python client library
from rclpy.node import Node  # Base class for ROS2 nodes
import numpy as np  # Numerical computations
import gtsam  # Graph-Based SLAM optimization library
import matplotlib.pyplot as plt  # For 2D visualization of the SLAM trajectory
import open3d as o3d  # For 3D point cloud visualization
import csv  # For exporting pose data to CSV files
from sensor_msgs.msg import LaserScan, Image  # ROS2 message types for LIDAR and camera data
from nav_msgs.msg import Odometry  # ROS2 message type for odometry data
from cv_bridge import CvBridge  # Converts ROS2 image messages to OpenCV format
import cv2  # OpenCV library for image processing

# Define the Graph SLAM Node
class GraphSLAMNode(Node):
    def __init__(self):
        super().__init__('graph_slam_recorder')  # Initialize the node with a name
        
        # Subscribe to LIDAR topic
        self.subscription_lidar = self.create_subscription(
            LaserScan, '/lidar_scan', self.lidar_callback, 10)
        
        # Subscribe to odometry topic
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Subscribe to camera image topic
        self.subscription_camera = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Create a graph for pose optimization
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        
        # Store pose history (x, y, theta)
        self.pose_history = []
        self.node_id = 0  # Counter for graph nodes
        
        self.get_logger().info("Graph SLAM Recorder Node Started - Recording...")

    # Callback function for odometry data
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x  # Extract x position
        y = msg.pose.pose.position.y  # Extract y position
        theta = np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2  # Compute orientation
        
        self.pose_history.append((x, y, theta))  # Store pose in history
        
        if self.node_id == 0:
            # Add a prior factor for the initial pose
            self.graph.add(gtsam.PriorFactorPose2(
                self.node_id, gtsam.Pose2(x, y, theta), 
                gtsam.noiseModel.Isotropic.Sigma(3, 0.1)))
        else:
            # Add odometry constraints between consecutive poses
            prev_x, prev_y, prev_theta = self.pose_history[-2]
            odometry_factor = gtsam.BetweenFactorPose2(
                self.node_id - 1, self.node_id, 
                gtsam.Pose2(x - prev_x, y - prev_y, theta - prev_theta), 
                gtsam.noiseModel.Isotropic.Sigma(3, 0.1))
            self.graph.add(odometry_factor)
        
            # Loop closure detection (checks if the robot revisits a previous place)
            for i in range(len(self.pose_history) - 2):
                loop_x, loop_y, _ = self.pose_history[i]
                if np.linalg.norm([x - loop_x, y - loop_y]) < 0.5:  # Loop detected within 0.5m radius
                    loop_closure = gtsam.BetweenFactorPose2(
                        i, self.node_id, gtsam.Pose2(0, 0, 0), 
                        gtsam.noiseModel.Isotropic.Sigma(3, 0.05))
                    self.graph.add(loop_closure)
                    self.get_logger().info(f"Loop closure detected at node {self.node_id}")
        
        self.node_id += 1  # Increment node counter

    def lidar_callback(self, msg):
        # Process LIDAR data for 3D mapping (to be implemented)
        pass

    def camera_callback(self, msg):
        # Convert ROS2 Image message to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(f'camera_frame_{self.node_id}.jpg', image)  # Save image

    def save_results(self):
        # Save recorded poses to CSV file
        with open('poses.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "theta"])
            writer.writerows(self.pose_history)
        
        # Optimize the graph
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
        optimized_values = optimizer.optimize()
        
        # Save optimized trajectory as an image
        x_vals, y_vals = zip(*[(pose.x(), pose.y()) for i, pose in optimized_values.items() if isinstance(pose, gtsam.Pose2)])
        plt.plot(x_vals, y_vals, 'ro-')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Graph SLAM Path')
        plt.savefig('graph_slam_map.png')
        
        # Export 3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(self.pose_history))
        o3d.io.write_point_cloud('graph_slam_map.ply', pcd)
        
        self.get_logger().info("Recording Stopped - Data Exported Successfully.")

    def stop_recording(self):
        self.save_results()
        rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    node = GraphSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_recording()
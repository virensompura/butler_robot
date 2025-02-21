#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import json
import os
import math
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Int32
import tf2_ros
import tf_transformations

# Import the basic navigator from Nav2 simple commander
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

##########################################
# RobotPosition Node: Monitors TF and logs deviations
##########################################
class RobotPosition(Node):
    def __init__(self):
        super().__init__('robot_position')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.goal_pose = None  # expected as a dictionary with keys "pos" and "orient"

    def calculate_deviation(self, current, goal):
        dx = current['x'] - goal['pos']['x']
        dy = current['y'] - goal['pos']['y']
        return math.sqrt(dx * dx + dy * dy)

    def calculate_angular_deviation(self, current, goal):
        # Extract yaw from current and goal quaternions
        current_quat = [
            current['orient']['x'],
            current['orient']['y'],
            current['orient']['z'],
            current['orient']['w']
        ]
        goal_quat = [
            goal['orient']['x'],
            goal['orient']['y'],
            goal['orient']['z'],
            goal['orient']['w']
        ]
        _, _, current_yaw = tf_transformations.euler_from_quaternion(current_quat)
        _, _, goal_yaw = tf_transformations.euler_from_quaternion(goal_quat)
        return math.degrees(abs(current_yaw - goal_yaw))

    def timer_callback(self):
        try:
            # Look up transform from 'odom' to 'base_link'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            current_pose = {
                'x': trans.transform.translation.x,
                'y': trans.transform.translation.y,
                'orient': {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w
                }
            }
            if self.goal_pose is not None:
                linear_dev = self.calculate_deviation(current_pose, self.goal_pose)
                angular_dev = self.calculate_angular_deviation(current_pose, self.goal_pose)
                self.get_logger().info(f'Linear deviation: {linear_dev:.2f} m')
                self.get_logger().info(f'Angular deviation: {angular_dev:.2f} deg')
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f"TF Error: {ex}")

##########################################
# LocationNavigator Node: Listens for location commands and navigates
##########################################
class LocationNavigator(Node):
    def __init__(self):
        super().__init__('location_navigator')

        # Subscribe to location commands (e.g., from voice command)
        self.create_subscription(String, '/location_select', self.location_callback, 10)
        # Subscribe to order status (order ready = True means the order is ready in kitchen)
        self.create_subscription(Bool, '/order_ready', self.order_ready_callback, 10)

        # Publisher to publish navigation feedback (optional)
        self.feedback_pub = self.create_publisher(String, '/navigation_feedback', 10)

        # Instantiate the navigator (from nav2_simple_commander)
        self.navigator = BasicNavigator()

        # Load locations from JSON file
        self.locations = self.load_locations("/home/virensompura/goat_ws/src/butler_robot/scripts/location.txt")
        if "home" not in self.locations:
            self.get_logger().error("Home location not found in file!")
            rclpy.shutdown()

        # Initialize order ready flag (default: False)
        self.order_ready = False

        # Create an instance of RobotPosition (so we can update its goal_pose)
        self.pose_monitor = RobotPosition()

        # Wait until Nav2 is active (blocking call)
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

    def load_locations(self, file_path):
        if os.path.exists(file_path):
            with open(file_path, "r") as f:
                try:
                    return json.load(f)
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"JSON decode error: {e}")
                    return {}
        else:
            self.get_logger().error(f"Location file {file_path} not found!")
            return {}

    def order_ready_callback(self, msg: Bool):
        self.order_ready = msg.data
        status = "ready" if self.order_ready else "not ready"
        self.get_logger().info(f"Order status updated: {status}")

    def location_callback(self, msg: String):
        selected_location = msg.data.strip().lower()
        self.get_logger().info(f"Received location command: {selected_location}")

        # For example, if the location is "kitchen", check if the order is ready
        if selected_location == "kitchen" and not self.order_ready:
            self.get_logger().info("Order in kitchen is not ready. Please wait...")
            self.publish_feedback("Order in kitchen not ready")
            return

        if selected_location not in self.locations:
            self.get_logger().warn(f"Location '{selected_location}' not found in file.")
            self.publish_feedback(f"Location '{selected_location}' not found")
            return

        # Set goal from location file
        location = self.locations[selected_location]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = location["pos_x"]
        goal_pose.pose.position.y = location["pos_y"]
        goal_pose.pose.orientation.z = location["orient_z"]
        goal_pose.pose.orientation.w = location["orient_w"]

        # Update the RobotPosition node's goal_pose for deviation feedback.
        self.pose_monitor.goal_pose = {
            "pos": {"x": location["pos_x"], "y": location["pos_y"]},
            "orient": {
                "x": 0.0, "y": 0.0, "z": location["orient_z"], "w": location["orient_w"]
            }
        }

        # Start navigation to the goal pose.
        self.get_logger().info(f"Navigating to {selected_location}...")
        self.navigator.goToPose(goal_pose)

        # Wait until navigation completes.
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f"Estimated time remaining: {eta:.0f} seconds")
            time.sleep(1)

        # Check result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Successfully arrived at {selected_location}")
            self.publish_feedback(f"Arrived at {selected_location}")
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f"Navigation to {selected_location} was canceled")
            self.publish_feedback(f"Navigation to {selected_location} canceled")
        else:
            self.get_logger().info(f"Navigation to {selected_location} failed. Returning home.")
            self.publish_feedback("Navigation failed, returning home")
            self.navigate_home()

    def navigate_home(self):
        if "home" not in self.locations:
            self.get_logger().error("Home location not found in locations file!")
            return
        home = self.locations["home"]
        home_pose = PoseStamped()
        home_pose.header.frame_id = "map"
        home_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        home_pose.pose.position.x = home["pos_x"]
        home_pose.pose.position.y = home["pos_y"]
        home_pose.pose.orientation.z = home["orient_z"]
        home_pose.pose.orientation.w = home["orient_w"]

        self.navigator.goToPose(home_pose)
        while not self.navigator.isTaskComplete():
            time.sleep(1)
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info("Successfully returned to home")
            self.publish_feedback("Returned to home")
        else:
            self.get_logger().error("Failed to return home")
            self.publish_feedback("Failed to return home")

    def publish_feedback(self, message: str):
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)

##########################################
# Main: Spin both nodes concurrently.
##########################################
def main(args=None):
    rclpy.init(args=args)

    # Instantiate both nodes
    location_navigator = LocationNavigator()
    robot_position = location_navigator.pose_monitor  # same instance created inside LocationNavigator

    # Use a MultiThreadedExecutor to run both nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(location_navigator)
    executor.add_node(robot_position)

    try:
        executor.spin()
    except KeyboardInterrupt:
        location_navigator.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        location_navigator.destroy_node()
        robot_position.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy, JoyFeedback, JoyFeedbackArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import json
import os

class JoyGoalNode(Node):
    def __init__(self):
        super().__init__('joy_goal_node')
        
        # Load locations from file
        self.LOCATIONS = self.load_locations('/home/virensompura/goat_ws/src/butler_robot/scripts/location.txt')
        
        # Button-to-location mapping (skip button index 2)
        self.BUTTON_TO_LOCATION = {
            0: 'home',
            1: 'kitchen',
            3: 'table_1',
            4: 'table_2'
        }

        # Create subscription to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        # Create action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for haptics feedback
        self.haptics_pub = self.create_publisher(JoyFeedbackArray, '/joy/set_feedback', 10)
        
        # Previous button states
        self.prev_buttons = None
         
        # Wait for action server
        self.get_logger().info('Waiting for navigation action server...')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
        else:
            self.get_logger().info('Navigation action server connected!')
    
    def load_locations(self, file_path):
        """Load locations from a JSON file."""
        if not os.path.exists(file_path):
            self.get_logger().error(f'File not found: {file_path}')
            return {}
        
        with open(file_path, 'r') as file:
            try:
                locations_data = json.load(file)
                self.get_logger().info('Locations loaded successfully')
                return locations_data
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse locations file: {e}')
                return {}
    
    def joy_callback(self, msg):
        # Initialize previous button states if not set
        if self.prev_buttons is None:
            self.prev_buttons = msg.buttons
            return
            
        # Check for button press events (transition from 0 to 1)
        for button_idx, location_name in self.BUTTON_TO_LOCATION.items():
            if (msg.buttons[button_idx] == 1 and 
                self.prev_buttons[button_idx] == 0):
                self.send_goal(location_name)
                
        # Update previous button states
        self.prev_buttons = msg.buttons
        
    def send_goal(self, location_name):
        if location_name not in self.LOCATIONS:
            self.get_logger().error(f'Unknown location: {location_name}')
            return
        
        location = self.LOCATIONS[location_name]
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = location['pos_x']
        goal_msg.pose.pose.position.y = location['pos_y']
        goal_msg.pose.pose.position.z = location['pos_z']
        
        # Set orientation
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = location['orient_z']
        goal_msg.pose.pose.orientation.w = location['orient_w']
        
        # Send the goal
        self.get_logger().info(f'Sending navigation goal to {location_name} position: '
                              f'x={location["pos_x"]}, y={location["pos_y"]}')
        
        # Send goal and setup callbacks
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.send_haptics(0.5)  # Half intensity for failure
            self.create_timer(1.0, lambda: self.send_haptics(0.0))  # Stop after 1 second
            return

        self.get_logger().info('Goal accepted')
        future = goal_handle.get_result_async()
        future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
            self.send_haptics(1.0)  # Full intensity for success
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
            self.send_haptics(0.5)  # Half intensity for failure
        
        # Stop haptics after 1 second
        self.create_timer(1.0, lambda: self.send_haptics(0.0))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can add custom feedback handling here if needed
        pass

    def send_haptics(self, intensity):
        """Send haptic feedback to the joystick."""
        msg = JoyFeedbackArray()
        
        # Create feedback for left motor
        left_feedback = JoyFeedback()
        left_feedback.type = JoyFeedback.TYPE_RUMBLE
        left_feedback.id = 0
        left_feedback.intensity = intensity
        
        # Create feedback for right motor
        right_feedback = JoyFeedback()
        right_feedback.type = JoyFeedback.TYPE_RUMBLE
        right_feedback.id = 1
        right_feedback.intensity = intensity
        
        # Add both feedback messages to the array
        msg.array = [left_feedback, right_feedback]
        
        # Publish the feedback
        self.haptics_pub.publish(msg)

def main():
    rclpy.init()
    node = JoyGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

class InteractiveOrderPublisher(Node):
    def __init__(self):
        super().__init__('interactive_order_publisher')
        self.location_pub = self.create_publisher(String, '/location_select', 10)
        self.order_pub = self.create_publisher(Bool, '/order_ready', 10)

    def publish_location(self, location: str):
        msg = String()
        msg.data = location
        self.location_pub.publish(msg)
        self.get_logger().info(f"Published location command: '{msg.data}'")

    def publish_order_status(self, ready: bool):
        msg = Bool()
        msg.data = ready
        self.order_pub.publish(msg)
        status_str = "ready" if ready else "not ready"
        self.get_logger().info(f"Published order status: {status_str}")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveOrderPublisher()

    # Interactive input from the user.
    order_ready_input = input("Is the order ready? (true/false): ").strip().lower()
    order_ready = True if order_ready_input == "true" else False

    # Publish order status.
    node.publish_order_status(order_ready)

    # If order is not ready, exit.
    if not order_ready:
        node.get_logger().info("Order is not ready. Exiting without navigation commands.")
        rclpy.spin_once(node, timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()
        return

    # Ask which table to deliver the order.
    table = input("Enter the table where the order should be delivered (e.g. table_2): ").strip()

    # Step 1: Navigate to kitchen (order pick-up).
    node.get_logger().info("Order is ready. Navigating from home to kitchen to pick up the order.")
    node.publish_location("kitchen")
    # Allow time for navigation and order pickup.
    time.sleep(5)

    # Step 2: Navigate from kitchen to the specified table.
    node.get_logger().info(f"Navigating from kitchen to {table} to deliver the order.")
    node.publish_location(table)
    time.sleep(5)

    # Optionally, return to home.
    node.get_logger().info("Returning to home position.")
    node.publish_location("home")
    time.sleep(2)

    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


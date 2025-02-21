#!/usr/bin/env python3

import rclpy 
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy.time
import tf2_ros
import tf2_geometry_msgs
import json
import os


class RobotPosition(Node):
    def __init__(self):
        super().__init__("robot_position")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
        self.timer  = self.create_timer(1.0,self.timer_callback)
        self.main_pos = {}
        self.categories = ["home","kitchen", "table_1","table_2","table_3", "table_4", "table_5"]
        self.file_path = "/home/virensompura/goat_ws/src/butler_robot/scripts/location.txt"

    def load_positions(self):
        if os.path.exists(self.file_path):
            with open(self.file_path,"r") as f:     
                try:
                    self.main_pos = json.load(f)
                except json.JSONDecodeError:
                    self.main_pos = {}

    def save_positions(self):
        with open(self.file_path,"w") as f:
            json.dump(self.main_pos,f,indent=4)
    

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("map","base_link",rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.1))
            pos = trans.transform.translation
            orient = trans.transform.rotation
            self.get_logger().info(
                f"Robot position: x={pos.x}, y={pos.y}, z={pos.z} | "
                f"orientation: x={orient.x}, y={orient.y}, z={orient.z}, w={orient.w}"
            )
            take_input = int(input("Select location (0 for Home, 1 for kitchen, 2 for table_1, 3 for table_2, 4 for table_3, 5 for table_4, 6 for table_5): "))   
            if take_input <=6 :
                self.load_positions()
                self.main_pos[self.categories[take_input]] = {
                    "pos_x" : pos.x,
                    "pos_y" : pos.y,
                    "pos_z" : pos.z,
                    "orient_z" : orient.z,
                    "orient_w" : orient.w
                }
                self.save_positions()
        except tf2_ros.LookupException:
            self.get_logger().warn('Transform not available')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'ExtrapolationException: {str(e)}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f'ConnectivityException: {str(e)}')
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'TransformException: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPosition()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

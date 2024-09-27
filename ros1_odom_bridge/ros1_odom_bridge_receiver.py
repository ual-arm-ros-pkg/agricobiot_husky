#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import yaml

# Set up socket connection
HOST = '192.168.131.2'  # This computer's IP
PORT = 5000  # Same port as in the sender script

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odometry', 10)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)  # Listen for 1 connection

        self.get_logger().info(f"Waiting for ROS1 odometry data on {HOST}:{PORT}")

        # Run socket listening in a separate thread
        self.run_socket()

    def run_socket(self):
        """Wait for a connection and process the incoming data."""
        while True:
            conn, addr = self.server_socket.accept()
            self.get_logger().info(f"Connection from {addr}")
            with conn:
                data = conn.recv(4096)
                if not data:
                    break
                self.publish_odom(data.decode('utf-8'))

    def publish_odom(self, odom_yaml):
        """Deserialize YAML and publish odometry data."""
        try:
            odom_dict = yaml.safe_load(odom_yaml)

            # Create an Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = odom_dict['header']['frame_id']
            odom_msg.pose.pose.position.x = odom_dict['pose']['position']['x']
            odom_msg.pose.pose.position.y = odom_dict['pose']['position']['y']
            odom_msg.pose.pose.position.z = odom_dict['pose']['position']['z']
            odom_msg.pose.pose.orientation.x = odom_dict['pose']['orientation']['x']
            odom_msg.pose.pose.orientation.y = odom_dict['pose']['orientation']['y']
            odom_msg.pose.pose.orientation.z = odom_dict['pose']['orientation']['z']
            odom_msg.pose.pose.orientation.w = odom_dict['pose']['orientation']['w']
            odom_msg.twist.twist.linear.x = odom_dict['twist']['linear']['x']
            odom_msg.twist.twist.linear.y = odom_dict['twist']['linear']['y']
            odom_msg.twist.twist.linear.z = odom_dict['twist']['linear']['z']
            odom_msg.twist.twist.angular.x = odom_dict['twist']['angular']['x']
            odom_msg.twist.twist.angular.y = odom_dict['twist']['angular']['y']
            odom_msg.twist.twist.angular.z = odom_dict['twist']['angular']['z']

            # Publish the message
            self.publisher_.publish(odom_msg)
            self.get_logger().info("Published odometry data to ROS2 topic")

        except yaml.YAMLError as e:
            self.get_logger().error(f"Failed to parse YAML: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish odometry data: {e}")

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



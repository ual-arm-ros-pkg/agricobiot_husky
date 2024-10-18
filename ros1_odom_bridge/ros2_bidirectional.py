#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import socket
import yaml
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Socket connection details
ODOM_PORT = 5000
CMD_VEL_HOST = '192.168.131.1'  # IP of ROS1 machine
CMD_VEL_PORT = 5001
TF_PORT = 5002
BUFFER_SIZE = 4096

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('ros2_bidirectional')

        # Create ROS2 publishers
        self.odom_publisher = self.create_publisher(Odometry, '/odometry', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create sockets once for odometry and TF reception
        self.odom_socket = self.create_socket(ODOM_PORT)
        self.tf_socket = self.create_socket(TF_PORT)

        # Create timers to periodically check for odometry and TF data
        self.create_timer(0.1, self.receive_odometry)
        self.create_timer(0.05, self.receive_tf)

        # Subscribe to /cmd_vel and send the data to ROS1
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def create_socket(self, port):
        """Create a socket for receiving data on a specified port."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('0.0.0.0', port))
        s.listen(1)
        s.setblocking(False)  # Non-blocking mode for efficient polling
        return s

    def receive_odometry(self):
        """Receive and process odometry data from ROS1."""
        try:
            conn, _ = self.odom_socket.accept()
            data = conn.recv(BUFFER_SIZE)
            conn.close()
            if data:
                odom_dict = yaml.safe_load(data)
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

                self.odom_publisher.publish(odom_msg)
        except socket.error:
            pass  # No data received, continue without blocking

    def receive_tf(self):
        """Receive and broadcast TF data from ROS1."""
        try:
            conn, _ = self.tf_socket.accept()
            data = conn.recv(BUFFER_SIZE)
            conn.close()
            if data:
                tf_dict = yaml.safe_load(data)
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = tf_dict['header']['frame_id']
                tf_msg.child_frame_id = tf_dict['child_frame_id']
                tf_msg.transform.translation.x = tf_dict['transform']['translation']['x']
                tf_msg.transform.translation.y = tf_dict['transform']['translation']['y']
                tf_msg.transform.translation.z = tf_dict['transform']['translation']['z']
                tf_msg.transform.rotation.x = tf_dict['transform']['rotation']['x']
                tf_msg.transform.rotation.y = tf_dict['transform']['rotation']['y']
                tf_msg.transform.rotation.z = tf_dict['transform']['rotation']['z']
                tf_msg.transform.rotation.w = tf_dict['transform']['rotation']['w']

                self.tf_broadcaster.sendTransform(tf_msg)
        except socket.error:
            pass  # No data received, continue without blocking

    def cmd_vel_callback(self, msg):
        """Send /cmd_vel from ROS2 to ROS1 via socket."""
        cmd_vel_dict = {
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z
            }
        }
        cmd_vel_yaml = yaml.dump(cmd_vel_dict)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((CMD_VEL_HOST, CMD_VEL_PORT))
            s.sendall(cmd_vel_yaml.encode('utf-8'))
            s.close()
        except Exception as e:
            self.get_logger().error(f"Failed to send /cmd_vel data to ROS1: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

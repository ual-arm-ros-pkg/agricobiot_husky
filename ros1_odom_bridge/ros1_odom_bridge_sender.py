#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
import socket
import yaml

# Set the IP and port of the receiving computer
HOST = '192.168.131.2'  # IP of the second computer
PORT = 5000  # Arbitrary port for communication

def callback(odom_data):
    """Callback function to send odometry data via socket."""
    try:
        # Convert ROS message to a dictionary (could also serialize in other formats)
        odom_dict = {
            'header': {
                'seq': odom_data.header.seq,
                'stamp': str(odom_data.header.stamp),
                'frame_id': odom_data.header.frame_id
            },
            'pose': {
                'position': {
                    'x': odom_data.pose.pose.position.x,
                    'y': odom_data.pose.pose.position.y,
                    'z': odom_data.pose.pose.position.z
                },
                'orientation': {
                    'x': odom_data.pose.pose.orientation.x,
                    'y': odom_data.pose.pose.orientation.y,
                    'z': odom_data.pose.pose.orientation.z,
                    'w': odom_data.pose.pose.orientation.w
                }
            },
            'twist': {
                'linear': {
                    'x': odom_data.twist.twist.linear.x,
                    'y': odom_data.twist.twist.linear.y,
                    'z': odom_data.twist.twist.linear.z
                },
                'angular': {
                    'x': odom_data.twist.twist.angular.x,
                    'y': odom_data.twist.twist.angular.y,
                    'z': odom_data.twist.twist.angular.z
                }
            }
        }

        # Convert the dictionary to YAML format
        odom_yaml = yaml.dump(odom_dict)

        # Open socket and send the data
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.sendall(odom_yaml.encode('utf-8'))
        rospy.loginfo("Sent odometry data to ROS2 computer")
        s.close()

    except Exception as e:
        rospy.logerr("Failed to send odometry data: %s" % str(e))

def listener():
    """ROS1 node to subscribe to /odometry/filtered."""
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()



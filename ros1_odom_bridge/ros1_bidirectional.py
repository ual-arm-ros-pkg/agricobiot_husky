#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import socket
import yaml
import tf2_ros
import geometry_msgs.msg
import time

# Socket connection details
ODOM_HOST = '192.168.131.2'  # IP of ROS2 machine
ODOM_PORT = 5000
CMD_VEL_PORT = 5001
TF_PORT = 5002

def send_socket_data(data, host, port):
    """Helper function to send data via socket."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall(data.encode('utf-8'))
        s.close()
    except Exception as e:
        rospy.logerr("Socket send failed: %s" % str(e))

def odom_callback(odom_data):
    """Send /odometry/filtered data to ROS2."""
    try:
        odom_dict = {
            'header': {
                'seq': odom_data.header.seq,
                'stamp': str(odom_data.header.stamp),
                'frame_id': odom_data.header.frame_id
            },
            'child_frame_id': odom_data.child_frame_id,
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
        odom_yaml = yaml.dump(odom_dict)
        send_socket_data(odom_yaml, ODOM_HOST, ODOM_PORT)
    except Exception as e:
        rospy.logerr("Failed to process odometry data: %s" % str(e))

def cmd_vel_receiver():
    """Receive /cmd_vel from ROS2 and publish it to ROS1."""
    rospy.loginfo("Listening for /cmd_vel from ROS2")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('0.0.0.0', CMD_VEL_PORT))
    s.listen(1)
    
    while not rospy.is_shutdown():
        conn, addr = s.accept()
        data = conn.recv(4096)
        conn.close()
        if data:
            cmd_vel_dict = yaml.safe_load(data)
            cmd_msg = Twist()
            cmd_msg.linear.x = cmd_vel_dict['linear']['x']
            cmd_msg.linear.y = cmd_vel_dict['linear']['y']
            cmd_msg.linear.z = cmd_vel_dict['linear']['z']
            cmd_msg.angular.x = cmd_vel_dict['angular']['x']
            cmd_msg.angular.y = cmd_vel_dict['angular']['y']
            cmd_msg.angular.z = cmd_vel_dict['angular']['z']
            pub.publish(cmd_msg)
            rospy.loginfo("Republished /cmd_vel from ROS2")

def tf_broadcaster():
    """Send the TF transform between base_link and odom to ROS2."""
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(20)  # 20 Hz

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0))
            tf_dict = {
                'header': {
                    'stamp': str(trans.header.stamp),
                    'frame_id': trans.header.frame_id
                },
                'child_frame_id': trans.child_frame_id,
                'transform': {
                    'translation': {
                        'x': trans.transform.translation.x,
                        'y': trans.transform.translation.y,
                        'z': trans.transform.translation.z
                    },
                    'rotation': {
                        'x': trans.transform.rotation.x,
                        'y': trans.transform.rotation.y,
                        'z': trans.transform.rotation.z,
                        'w': trans.transform.rotation.w
                    }
                }
            }
            tf_yaml = yaml.dump(tf_dict)
            send_socket_data(tf_yaml, ODOM_HOST, TF_PORT)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup TF")
        rate.sleep()

def listener():
    """Main function to start listeners and callbacks."""
    rospy.init_node('ros1_bidirectional', anonymous=True)
    
    # Subscribe to /odometry/filtered
    rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)
    
    # Start a thread to receive /cmd_vel from ROS2
    rospy.Timer(rospy.Duration(0.1), lambda event: cmd_vel_receiver())  # Start cmd_vel listener in background
    
    # Start TF broadcaster
    tf_broadcaster()

    rospy.spin()

if __name__ == '__main__':
    listener()

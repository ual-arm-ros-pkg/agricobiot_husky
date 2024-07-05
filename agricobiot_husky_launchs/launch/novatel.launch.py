import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Set the path to different files and folders.
    pkg_agrico = get_package_share_directory('agricobiot_husky_launchs')

    novatel_node = Node(
        package='mrpt_sensor_gnss_novatel',
        executable='mrpt_sensor_gnss_novatel_node',
        name='mrpt_sensor_gnss_novatel_node',
        output='screen',
        parameters=[
            {
                "publish_topic": "/gps_novatel",
                "publish_mrpt_obs_topic": "/gps_novatel_mrpt",
                "sensor_frame_id": "novatel",
                "sensor_label": "novatel",
                "novatel_main_serial_port": "/dev/serial/by-id/usb-Novatel_Inc._Novatel_GPS_Receiver_BJYA15400456J-if00-port0",
                "novatel_ntrip_serial_port": "/dev/serial/by-id/usb-Novatel_Inc._Novatel_GPS_Receiver_BJYA15400456J-if00-port2",
                "ntrip_server": "www.euref-ip.net",
                "ntrip_port": "2101",
                "ntrip_mount_point": "ALME00ESP0",
                "ntrip_user": os.environ["RTK_USER"],
                "ntrip_password": os.environ["RTK_PASS"],
                "novatel_init_azimuth": "0.0 25.0",
            }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    #ld.add_action(declare_map_yaml_cmd)

    ld.add_entity(novatel_node)

    return ld

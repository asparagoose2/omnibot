from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name="ROS_LOG_DIR", value='/home/working_dir/logs'
        ),
        Node(
            package='omnibot',
            executable='twist2serial.py',
        )
    ])
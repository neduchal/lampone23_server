import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution 



def generate_launch_description():


    return LaunchDescription([
        Node(
            package='lampone23_server',
            namespace='lampone23',
            executable='image_grabber',
            name='lampone23_image_grabber'
        ),
       Node(
            package='lampone23_server',
            namespace='lampone23',
            executable='server',
            name='lampone23_server'
        ),
       Node(
            package='lampone23_server',
            namespace='lampone23',
            executable='controller',
            name='lampone23_controller'
        )        
    ])

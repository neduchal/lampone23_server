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
            node_namespace='lampone23',
            node_executable='image_grabber',
            node_name='lampone23_image_grabber'
        ),
       Node(
            package='lampone23_server',
            node_namespace='lampone23',
            node_executable='server',
            node_name='lampone23_server'
        ),
       Node(
            package='lampone23_server',
            node_namespace='lampone23',
            node_executable='controller',
            node_name='lampone23_controller'
        )        
    ])

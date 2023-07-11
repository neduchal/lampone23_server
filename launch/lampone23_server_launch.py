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

    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')

    realsense_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')),
        #launch_arguments={'camera_name':'home_navigation'}.items()
    )

    scan_height_arg = DeclareLaunchArgument(
        'scan_height', default_value=TextSubstitution(text='100')
    )
    range_max_arg = DeclareLaunchArgument(
        'range_max', default_value=TextSubstitution(text='3')
    )    

    return LaunchDescription([
        realsense_launch,
        scan_height_arg,
        range_max_arg,
        Node(
            package='depthimage_to_laserscan',
            namespace='home_navigation',
            executable='depthimage_to_laserscan_node',
            name='sim',
            remappings=[
                ('/home_navigation/depth', '/camera/depth/image_rect_raw'),
                ('/home_navigation/depth_camera_info', '/camera/depth/camera_info'),
            ],
            parameters=[
                {'scan_height': 50},
                {'range_max': 5.0}
            ]
        )
    ])

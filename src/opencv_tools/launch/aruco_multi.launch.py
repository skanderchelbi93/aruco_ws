import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    opencv_tools_dir = get_package_share_directory('opencv_tools')

    # Arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.05',
        description='Default size of the ArUco markers in meters'
    )
    
    marker_sizes_arg = DeclareLaunchArgument(
        'marker_sizes', default_value='{}',
        description='JSON string mapping marker IDs to sizes, e.g., \'{"0": 0.05, "1": 0.1}\''
    )

    dictionary_arg = DeclareLaunchArgument(
        'dictionary', default_value='DICT_ARUCO_ORIGINAL',
        description='ArUco dictionary name'
    )

    # Realsense Launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_pointcloud': 'false',
            'ordered_pc': 'false',
        }.items()
    )

    # ArUco Processor Node
    aruco_processor_node = Node(
        package='opencv_tools',
        executable='aruco_processor',
        name='aruco_processor',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_sizes': LaunchConfiguration('marker_sizes'),
            'dictionary': LaunchConfiguration('dictionary'),
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }],
        output='screen'
    )

    return LaunchDescription([
        marker_size_arg,
        marker_sizes_arg,
        dictionary_arg,
        realsense_launch,
        aruco_processor_node
    ])

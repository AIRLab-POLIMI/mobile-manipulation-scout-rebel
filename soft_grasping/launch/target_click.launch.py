# ROS2 imports
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():

    aruco_params_file = os.path.join(
        get_package_share_directory('aruco_pose_estimation'),
        'config',
        'aruco_parameters.yaml'
    )

    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config = config["/aruco_node"]["ros__parameters"]

    image_topic_arg = DeclareLaunchArgument(
        name='image_topic',
        default_value=config['image_topic'],
        description='Name of the image RGB topic to subscribe to',
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        name='depth_image_topic',
        default_value=config['depth_image_topic'],
        description='Name of the depth image topic to subscribe to',
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        name='camera_info_topic',
        default_value=config['camera_info_topic'],
        description='Name of the camera info topic to subscribe to',
    )

    camera_frame_arg = DeclareLaunchArgument(
        name='camera_frame',
        default_value=config['camera_frame'],
        description='Name of the camera frame where the estimated pose will be',
    )

    target_click_node = Node(
        package='soft_grasping',
        executable='target_click.py',
        parameters=[{
            "image_topic": LaunchConfiguration('image_topic'),
            "depth_image_topic": LaunchConfiguration('depth_image_topic'),
            "camera_info_topic": LaunchConfiguration('camera_info_topic'),
            "camera_frame": LaunchConfiguration('camera_frame'),
        }],
        output='screen',
        emulate_tty=True
    )

    # launch realsense camera node
    cam_feed_launch_file = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    camera_feed_depth_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_feed_launch_file),
        launch_arguments={
            "pointcloud.enable": "true",
            "enable_rgbd": "true",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
        }.items(),
    )

    rviz_file = PathJoinSubstitution([
        FindPackageShare('soft_grasping'),
        'rviz',
        'target_click.rviz'
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        # Arguments
        image_topic_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,
        camera_frame_arg,
        # Nodes
        camera_feed_depth_node,
        target_click_node,
        rviz2_node
    ])

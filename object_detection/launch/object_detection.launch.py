
# ROS2 imports
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

# Python imports
import os
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():

    # Load camera parameters from the config file
    camera_params_file = os.path.join(
        get_package_share_directory('soft_grasping'),
        'config',
        'camera.yaml'
    )

    with open(camera_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config = config["/**"]["ros__parameters"]

    rgb_topic_arg = DeclareLaunchArgument(
        name='rgb_topic',
        default_value=config['rgb_topic'],
        description='Name of the image RGB topic provided by the camera node',
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        name='depth_topic',
        default_value=config['depth_topic'],
        description='Name of the depth image topic provided by the camera node',
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        name='pointcloud_topic',
        default_value=config['pointcloud_topic'],
        description='Name of the pointcloud topic provided by the camera node',
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        name='camera_info_topic',
        default_value=config['camera_info_topic'],
        description='Name of the camera info topic provided by the camera node',
    )

    camera_rgb_frame_arg = DeclareLaunchArgument(
        name='camera_rgb_frame',
        default_value=config['camera_rgb_frame'],
        description='Camera frame of reference for rgb images',
    )

    camera_depth_frame_arg = DeclareLaunchArgument(
        name='camera_depth_frame',
        default_value=config['camera_depth_frame'],
        description='Camera frame of reference for depth images',
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

    # launch target click node: click on the target on the RGB image window to get the pixel coordinates
    # of the target in the camera frame, for testing purposes
    yolov8_detector_node = Node(
        name="yolov8_detector",
        package='object_detection',
        executable='yolov8_detector.py',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
        }],
        output='screen',
        emulate_tty=True
    )

    # delay start of the grasping pose estimator node until the realsense camera node is up and running
    ball_detector_node_delayed = TimerAction(
        period=1.0,
        actions=[
            LaunchDescription([
                yolov8_detector_node
            ])
        ]
    )

    return LaunchDescription([
        # Arguments
        rgb_topic_arg,
        pointcloud_topic_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,
        camera_rgb_frame_arg,
        camera_depth_frame_arg,
        # Nodes
        camera_feed_depth_node,
        ball_detector_node_delayed,
    ])

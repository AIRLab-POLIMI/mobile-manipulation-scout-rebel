
# ROS2 imports
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

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

    # start the demo autonomy task
    dataset_collector_node = Node(
        name="dataset_collector",
        package="object_detection",
        executable="dataset_collector.py",
        emulate_tty=True,
        output="screen",
        parameters=[{
             'rgb_topic': LaunchConfiguration('rgb_topic'),
        }],
    )

    # launch realsense camera node
    cam_feed_launch_file = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    camera_feed_depth_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_feed_launch_file),
        launch_arguments={
            "enable_color": "true",
            "camera_namespace": ""
        }.items(),
    )

    # delay start of the grasping pose estimator node until the realsense camera node is up and running
    dataset_collector_node_delayed = TimerAction(
        period=1.0,
        actions=[
            LaunchDescription([
                dataset_collector_node
            ])
        ]
    )

    return LaunchDescription([
        rgb_topic_arg,
        camera_feed_depth_node,
        dataset_collector_node_delayed
    ])

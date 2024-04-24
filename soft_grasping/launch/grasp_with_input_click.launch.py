
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

# load moveit controllers and parameters from yaml and robot description files
from moveit_launch import moveit_loader


def generate_launch_description():

    # Load camera parameters from the config file
    camera_params_file = os.path.join(
        get_package_share_directory('soft_grasping'),
        'config',
        'params.yaml'
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

    # declare launch arguments from moveit loader
    args = moveit_loader.declare_arguments()
    # read camera frame from aruco_pose_estimation config file
    camera_frame_arg = moveit_loader.load_camera_frame_arg()

    args.append(camera_frame_arg)

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
    target_click_node = Node(
        name="target_click",
        package='soft_grasping',
        executable='target_click.py',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
        }],
        output='screen',
        emulate_tty=True
    )

    # delay start of the grasping pose estimator node until the realsense camera node is up and running
    target_click_node_delayed = TimerAction(
        period=1.0,
        actions=[
            LaunchDescription([
                target_click_node
            ])
        ]
    )

    # launch grasping pose estimator node
    grasp_with_input_click_node = Node(
        name='grasp_with_input_click',
        package='soft_grasping',
        executable='grasp_with_input_click',
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [{
            # parameters for moveit2_apis library
            "camera_frame": LaunchConfiguration('camera_rgb_frame'),
            "load_base": LaunchConfiguration("load_base"),
            # parameters for the grasping pose estimator node
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'camera_rgb_frame': LaunchConfiguration('camera_rgb_frame'),
            'camera_depth_frame': LaunchConfiguration('camera_depth_frame'),
        }],
        output='screen',
        emulate_tty=True
    )

    # delay start of the grasping pose estimator node until the realsense camera node is up and running
    grasp_with_input_click_node_delayed = TimerAction(
        period=1.0,
        actions=[
            LaunchDescription([
                grasp_with_input_click_node
            ])
        ]
    )

    # launch rviz2 with the target click rviz configuration file
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

    return LaunchDescription(args + [
        # Arguments
        rgb_topic_arg,
        pointcloud_topic_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,
        camera_rgb_frame_arg,
        camera_depth_frame_arg,
        # Nodes
        camera_feed_depth_node,
        target_click_node_delayed,
        grasp_with_input_click_node_delayed,
        rviz2_node
    ])

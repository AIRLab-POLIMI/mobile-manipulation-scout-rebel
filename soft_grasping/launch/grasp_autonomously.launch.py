
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

    # launch grasping autonomously node: using the detected object's bounding box and the depth map to
    # segment the point cloud and estimate the grasping pose effectively and autonomously
    grasp_autonomous_node = Node(
        name='grasp_autonomous',
        package='soft_grasping',
        executable='grasp_autonomous',
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [{
            # parameters for moveit2_apis library
            "camera_frame": LaunchConfiguration('camera_rgb_frame'),
            "load_base": LaunchConfiguration("load_base"),
            # parameters for the grasping pose estimator node
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'camera_rgb_frame': LaunchConfiguration('camera_rgb_frame'),
            'camera_depth_frame': LaunchConfiguration('camera_depth_frame'),
        }],
        output='screen',
        emulate_tty=True
    )

    # delay start of the grasp_autonomous node until the object_detection node is up and running
    grasp_autonomous_node_delayed = TimerAction(
        period=3.0,
        actions=[
            LaunchDescription([
                grasp_autonomous_node
            ])
        ]
    )

    # launch rviz2 with the target autonomous rviz configuration file
    rviz_file = PathJoinSubstitution([
        FindPackageShare('soft_grasping'),
        'rviz',
        'target_detection.rviz'
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
        grasp_autonomous_node_delayed,
        rviz2_node
    ])

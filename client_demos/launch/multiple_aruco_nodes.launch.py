# ROS2 imports
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

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

    # declare configuration parameters
    marker_size_arg = DeclareLaunchArgument(
        name='marker_size',
        default_value=str(config['marker_size']),
        description='Size of the aruco marker in meters',
    )

    aruco_dictionary_id_arg = DeclareLaunchArgument(
        name='aruco_dictionary_id',
        default_value=config['aruco_dictionary_id'],
        description='ID of the aruco dictionary to use',
    )

    use_depth_input_arg = DeclareLaunchArgument(
        name='use_depth_input',
        default_value=str(config['use_depth_input']),
        description='Use depth input for pose estimation',
        choices=['true', 'false', 'True', 'False']
    )

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

    detected_markers_topic_arg = DeclareLaunchArgument(
        name='detected_markers_topic',
        default_value=config['detected_markers_topic'],
        description='Name of the topic to publish the detected markers messages',
    )

    markers_visualization_topic_arg = DeclareLaunchArgument(
        name='markers_visualization_topic',
        default_value=config['markers_visualization_topic'],
        description='Name of the topic to publish the pose array for visualization of the markers',
    )

    output_image_topic_arg = DeclareLaunchArgument(
        name='output_image_topic',
        default_value=config['output_image_topic'],
        description='Name of the topic to publish the image with the detected markers',
    )

    # big aruco pose estimation node
    big_aruco_node = Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        parameters=[{
            "marker_size": 0.185,
            "aruco_dictionary_id": "DICT_4X4_50",
            "use_depth_input": False,
            "image_topic": LaunchConfiguration('image_topic'),
            "depth_image_topic": LaunchConfiguration('depth_image_topic'),
            "camera_info_topic": LaunchConfiguration('camera_info_topic'),
            "camera_frame": LaunchConfiguration('camera_frame'),
            "detected_markers_topic": "/aruco/markers/big",
            "markers_visualization_topic": "/aruco/poses/big",
            "output_image_topic": "/aruco/image/big",
        }],
        output='screen',
        emulate_tty=True
    )

    # small aruco pose estimation node
    small_aruco_node = Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        parameters=[{
            "marker_size": 0.033,
            "aruco_dictionary_id": "DICT_4X4_50",
            "use_depth_input": False,
            "image_topic": LaunchConfiguration('image_topic'),
            "depth_image_topic": LaunchConfiguration('depth_image_topic'),
            "camera_info_topic": LaunchConfiguration('camera_info_topic'),
            "camera_frame": LaunchConfiguration('camera_frame'),
            "detected_markers_topic": "/aruco/markers/small",
            "markers_visualization_topic": "/aruco/poses/small",
            "output_image_topic": "/aruco/image/small",
        }],
        output='screen',
        emulate_tty=True
    )

    # multi aruco plane detection node
    multi_aruco_plane_node = Node(
        package="multi_aruco_plane_detection",
        executable="multi_aruco_plane_detection",
        name="multi_aruco_plane_detection",
        parameters=[{
            "camera_frame": LaunchConfiguration("camera_frame"),
        }],
        remappings=[("/aruco/markers", "/aruco/markers/small")],
        output="screen",
        emulate_tty=True,
    )

    # launch realsense camera node
    cam_feed_launch_file = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    camera_feed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_feed_launch_file),
        launch_arguments={
            "pointcloud.enable": "true",
            "enable_color": "true",
            "camera_namespace": ""
        }.items(),
    )

    rviz_file = PathJoinSubstitution([
        FindPackageShare('aruco_pose_estimation'),
        'rviz',
        'cam_detect.rviz'
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        marker_size_arg,
        aruco_dictionary_id_arg,
        use_depth_input_arg,
        image_topic_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,
        camera_frame_arg,
        detected_markers_topic_arg,
        markers_visualization_topic_arg,
        output_image_topic_arg,

        multi_aruco_plane_node,
        small_aruco_node,
        big_aruco_node,
        camera_feed_node,
        rviz2_node
    ])

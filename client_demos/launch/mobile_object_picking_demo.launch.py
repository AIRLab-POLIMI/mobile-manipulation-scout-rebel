""" Mobile Object Picking Demo

Rotate the arm with the camera on top, and spin around itself, until the target aruco spot is located.
Once the aruco is located, it will publish the estimated location on /target_pose pose topic.
Given the target pose, it computes the parking position.
Then it navigates to the computed parking pose.
Once the robot is successfully parked, it starts the button presser demo.
It finds the button setup box nearby, and once the box is located, it presses the buttons one at a time.

Architecture

demo C++ client:
- client sends request of high-level motion commands to the C++ object picker and dropper servers
- client sends request of high-level parking and navigation commands to the python parking server

parking python server:
- handle request for computing parking position and navigation to the target goal using NAV2,
- returns once the target goal is reached or the navigation aborts

Object Picking C++ action server:
- handle request to find objects in a reachable range and pick them up
- returns once the object is picked up or if there are no objects in the reachable range to pick up

Object Dropping C++ action server:
- handle request to drop the object in a predefined location
- returns once the object is dropped and the robot arm is back to its parked position

"""

# ros2 python imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

# Python imports
import os
from ament_index_python.packages import get_package_share_directory
import yaml

# load moveit controllers and parameters from yaml and robot description files
from moveit_launch import moveit_loader

# remember to start the moveit_controllers.launch.py file from the igus_rebel_moveit_config package first
# it will start the necessary controllers and the moveit dependencies
# start it with the correct gripper and hardware protocol arguments, and leaving rviz_file:=none (default)

# launches only the URDF version 2 robot description

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

    # read nav2_servers config waypoints file
    nav2_waypoints_file = os.path.join(
        get_package_share_directory('nav2_servers'),
        'config',
        'waypoints.yaml'
    )

    waypoints_config_arg = DeclareLaunchArgument(
        name='waypoints_config',
        default_value=nav2_waypoints_file,
        description='Path to the waypoints config file',
    )

    with open(nav2_waypoints_file, 'r') as file:
        waypoints_config = yaml.safe_load(file)

    # choose the demo to test
    thread_demo_arg = DeclareLaunchArgument(
        name='thread_demo',
        default_value='demov2',
        description='Choose the thread to run in mobile object picking demo',
        choices=["picking", "dropping", "parking", "demov1", "demov2"]
    )

    # declare launch arguments from moveit loader
    args = moveit_loader.declare_arguments()
    # read camera frame from aruco_pose_estimation config file
    camera_frame_arg = moveit_loader.load_camera_frame_arg()

    args.append(camera_frame_arg)

    rviz_file_name = "target_detection.rviz"

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("soft_grasping"), "rviz", rviz_file_name]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_loader.load_robot_description(),
            moveit_loader.load_robot_description_semantic(),
        ],
    )

    picking_dropping_action_servers_node = Node(
        package="soft_grasping",
        executable="grasp_action_servers",
        name="grasp_action_servers_node",
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [{
            # parameters for moveit2_apis library
            "camera_frame": LaunchConfiguration("camera_frame"),
            "load_base": LaunchConfiguration("load_base"),
            # parameters for the grasping pose estimator node
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'camera_rgb_frame': LaunchConfiguration('camera_rgb_frame'),
            'camera_depth_frame': LaunchConfiguration('camera_depth_frame'),
        }],
        output="screen",
    )

    robot_parking_action_server = Node(
        package='nav2_servers',
        executable='robot_parking_action_server.py',
        name='robot_parking_action_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'waypoints_config': LaunchConfiguration('waypoints_config')
        }]
    )

    mobile_object_picking_client_node = Node(
        package='client_demos',
        executable='mobile_object_picking',
        name='mobile_object_picking',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # waypoints names list
            'waypoints': waypoints_config["waypoints"],
            "thread_demo": LaunchConfiguration('thread_demo'),
        }]
    )

    # aruco pose estimation launch
    aruco_pose_estimator_node = Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        parameters=[{
            "marker_size": 0.095,
            "aruco_dictionary_id": "DICT_7X7_50",
            "use_depth_input": False,
            "image_topic": LaunchConfiguration('rgb_topic'),
            "depth_image_topic": LaunchConfiguration('depth_topic'),
            "camera_info_topic": LaunchConfiguration('camera_info_topic'),
            "camera_frame": LaunchConfiguration('camera_rgb_frame'),
            "detected_markers_topic": "/aruco/markers",
            "markers_visualization_topic": "/aruco/poses",
            "output_image_topic": "/aruco/image",
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription(args + [
        # camera arguments
        rgb_topic_arg,
        pointcloud_topic_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,
        camera_rgb_frame_arg,
        camera_depth_frame_arg,
        waypoints_config_arg,
        thread_demo_arg,

        # rviz2_node,
        robot_parking_action_server,
        aruco_pose_estimator_node,
        picking_dropping_action_servers_node,
        mobile_object_picking_client_node
    ])

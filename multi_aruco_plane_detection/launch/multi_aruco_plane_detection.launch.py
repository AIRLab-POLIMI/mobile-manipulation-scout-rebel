# ROS2 imports
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # include launch description from aruco_pose_estimation package
    aruco_pose_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_pose_estimation"),
                "launch",
                "aruco_pose_estimation.launch.py"])
        ),
        launch_arguments={
            "marker_size": "0.033",
            "aruco_dictionary_id": "DICT_4X4_50",
        }.items(),
    )

    # launch multi aruco plane detection node
    multi_aruco_node = Node(
        package="multi_aruco_plane_detection",
        executable="multi_aruco_plane_detection",
        name="multi_aruco_plane_detection",
        parameters=[{
            "camera_frame": LaunchConfiguration("camera_frame"),
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        aruco_pose_estimation_launch,
        multi_aruco_node
    ])

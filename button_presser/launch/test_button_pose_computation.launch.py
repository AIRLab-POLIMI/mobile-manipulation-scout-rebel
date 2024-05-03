# ros2 python imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# load moveit controllers and parameters from yaml and robot description files
from moveit_launch import moveit_loader


def generate_launch_description():

    # declare launch arguments from moveit loader
    args = moveit_loader.declare_arguments()
    # read camera frame from aruco_pose_estimation config file
    camera_frame_arg = moveit_loader.load_camera_frame_arg()

    args.append(camera_frame_arg)

    # create node for goal pose publisher
    test_button_pose_computation_node = Node(
        package="button_presser",
        executable="test_button_pose_computation",
        name="test_button_pose_computation_node",
        output="screen",
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [{
                "camera_frame": LaunchConfiguration("camera_frame"),
                "load_base": LaunchConfiguration("load_base"),
        }],
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("button_presser"), "rviz", "button_presser.rviz"]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
    )

    return LaunchDescription(args + [test_button_pose_computation_node, rviz2_node])

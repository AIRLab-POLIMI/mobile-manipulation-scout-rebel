
# ros2 python imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# load moveit controllers and parameters from yaml and robot description files
from moveit_launch import moveit_loader

# remember to start the moveit_controllers.launch.py file from the igus_rebel_moveit_config package first
# it will start the necessary controllers and the moveit dependencies
# start it with the correct gripper and hardware protocol arguments, and leaving rviz_file:=none (default)

def generate_launch_description():
    # declare launch arguments from moveit loader
    args = moveit_loader.declare_arguments()
    # read camera frame from aruco_pose_estimation config file
    camera_frame_arg = moveit_loader.load_camera_frame_arg()

    args.append(camera_frame_arg)

    button_presser_demo_node = Node(
        package="button_presser",
        executable="button_presser_demo",
        name="button_presser_demo_node",
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [{
                "camera_frame": LaunchConfiguration("camera_frame"),
                "load_base": LaunchConfiguration("load_base"),
        }],
    )

    rviz_file_name = "button_presser.rviz"

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("button_presser"), "rviz", rviz_file_name]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_loader.load_robot_description(),
            moveit_loader.load_robot_description_semantic(),
        ],
    )

    return LaunchDescription(args + [button_presser_demo_node, rviz2_node])

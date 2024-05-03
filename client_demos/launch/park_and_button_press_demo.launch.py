""" Navigate and button press demo

Rotate the arm with the camera on top, and spin around itself, until the target aruco spot is located.
Once the aruco is located, it will publish the estimated location on /target_pose pose topic.
Given the target pose, it computes the parking position.
Then it navigates to the computed parking pose.
Once the robot is successfully parked, it starts the button presser demo.
It finds the button setup box nearby, and once the box is located, it presses the buttons one at a time.

Architecture

demo C++ client:
- client sends request of high-level motion commands to the C++ button presser and button finder servers
- client sends request of high-level parking and navigation commands to the python parking server

button presser C++ server:
- handle request for looking nearby for aruco markers and presses the buttons once the arucos are found
- provides feedback on the buttons being found and pressed
- final result includes percentage of completion in linear motions and total number of successful pose goals reached

button finder C++ server:
- handles request for looking around in the surroundings for a single unique aruco marker
- provides feedback when marker is found and returns as a result the estimated pose
- after aruco is found, the robot arm parks itself to the parking position

parking python server:
- handle request for computing parking position and navigation to the target goal using NAV2,
- returns once the target goal is reached or the navigation aborts

"""

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

# launches only the URDF version 2 robot description
def generate_launch_description():
    # declare launch arguments from moveit loader
    args = moveit_loader.declare_arguments()
    # read camera frame from aruco_pose_estimation config file
    camera_frame_arg = moveit_loader.load_camera_frame_arg()

    args.append(camera_frame_arg)

    rviz_file_name = "button_presser.rviz"

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("button_presser"), "rviz", rviz_file_name]
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

    button_presser_action_servers = Node(
        package="button_presser",
        executable="button_presser_action_servers",
        name="button_presser_action_servers_node",
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [{
                "camera_frame": LaunchConfiguration("camera_frame"),
                "load_base": LaunchConfiguration("load_base"),
        }],
        remappings=[("/aruco/markers", "/aruco/markers/big")],
        output="screen",
    )

    robot_parking_action_server = Node(
        package='nav2_servers',
        executable='robot_parking_action_server.py',
        name='robot_parking_action_server',
        output='screen',
        emulate_tty=True,
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    park_and_button_press_client = Node(
        package='client_demos',
        executable='park_and_button_press',
        name='Park_and_button_press_client',
        output='screen',
        emulate_tty=True,
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(args + [
        button_presser_action_servers,
        rviz2_node,
        robot_parking_action_server,
        park_and_button_press_client
    ])

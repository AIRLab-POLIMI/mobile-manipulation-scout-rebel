#!/usr/bin/env python3


"""
Code to park the robot next to the target pose, according to a ranking algorithm based on the costmap and the robot footprint.

Author: Simone Giampà
University: Politecnico di Milano
Master Degree: Computer Science Engineering
Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

"""

# python imports
import numpy as np
import time
import os

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32, Polygon
from tf_transformations import euler_from_quaternion
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory

# NAV2 control API imports (ROS2-Iron Python bindings required)
from nav2_servers.costmap_2d import PyCostmap2D
from nav2_servers.footprint_collision_checker import FootprintCollisionChecker
from nav2_servers.robot_navigator import BasicNavigator, TaskResult


class RobotParking(Node):
    """
    ROS2 node to park the robot next to the target pose, according to the following algorithm:

    0. given a goal pose (x y theta)
    1. random sample n possible x,y positions in a circle of radius r and centered in the goal pose
    2. save the n coordinates in a vector and order them by lowest cartesian distance from the starting point
    2. random sample m possible theta orientations, with pi < |theta| < pi/2
    3. save the m orientations in a vector and order them by lowest distance in absolute valoue from pi (meaning that pi comes first and then the others
    4. Create vector with all n x m possible combinations, while respecting the previous order. Filter out the parking poses colliding with any walls or obstacles.
    5. Compute the cost for all possible targets in the costmap and filter out the targets having the robot footprint with a cost higher than a set threshold t. Then sort the vector by lowest cost (ascending)
    6. iterate for all possible combinations n x m until a feasible parking pose is found:

        for every parking(x,y, theta): // ordered by path cost

            viable = plan_path( kinematics, start, park)
            if (viable) save parking and planning

        - pick the viable path having the lowest path plan cost (first one in the vector of possible parkings)

    7. check if a feasible path exist from the starting pose to the parking pose. Iterate the steps 1-4 until a target pose with viable traversable path is found. If a new iteration of the algorithm is necessary, then change the value of r making it a little bit closer to the goal pose. Consider the skid steering kinematics while checking the parking paths.
    8. once a viable parking pose is found, navigate the robot to the parking pose. If no viable parking poses are found, return an error.

    Attributes:
    ----------
        navigator : nav2_servers.robot_navigator. BasicNavigator
            the NAV2 control API object
        target_radius : float
            radius of the circle centered in the target pose
        xy_samples : int 
            number of random samples for x,y coordinates
        theta_samples : int
            number of random samples for theta orientation
        robot_width : float
            width of the robot footprint
        robot_length : float
            length of the robot footprint
        footprint : geometry_msgs.msg.Polygon 
            footprint of the robot
        target_available : bool
            flag to indicate if a target pose is available
        initial_pose : geometry_msgs.msg.PoseStamped
            starting point of the robot
        robot_link_name : str
            name of the robot base link
        tf_buffer : tf2_ros.buffer.Buffer
            TF2 transform buffer
        tf_listener : tf2_ros.transform_listener.TransformListener
            TF2 transform listener
        map_frame : str
            name of the map frame -> "map

    Methods:
    -------
        callback_target(msg: PoseStamped):
            subscriber callback to target pose
        setup_navigation():
            main function thread
        parking_algorithm():
            apply the parking algorithm to find the optimal parking pose
        get_current_pose():
            get the current pose of the robot from the TF2 transform
        compute_footprint_polygon(): 
            compute the footprint polygon of the robot given the width and length
        sample_positions():
            sample xy_samples random positions in a circle of radius target_radius centered in the target pose
        sample_orientations(goal_x: float, goal_y: float):
            sample theta_samples random orientations in the range -pi /2 < theta < pi/2
        compute_footprint_cost_at_pose(x: float, y: float, theta: float)
            compute the footprint cost at a given pose
        convert_to_pose_stamped(x: float, y: float, theta: float)
            convert x,y,theta coordinates to PoseStamped
        filter_out_colliding_poses(target_candidates): 
            filter out the parking poses colliding with any walls or obstacles
        ranking_targets_by_cost_dist_theta(target_candidates, target_costs)
            apply ranking algorithm: maximize weighted sum of normalized cost, distance and orientation


    """

    # Costmap values constants
    NO_INFORMATION = 255
    LETHAL_OBSTACLE = 254
    INSCRIBED_INFLATED_OBSTACLE = 253
    MAX_NON_OBSTACLE = 252
    FREE_SPACE = 0

    # parking algorithm parameters
    target_radius = 0.4  # meters
    theta_samples = 15  # number of random samples for x,y coordinates, where theta is the orientation from the target
    phi_samples = 15  # number of random samples for phi orientation, where phi is the orientation delta from theta
    theta_delta_max = np.pi / 8.0  # radians, maximum orientation delta from the normal of the target pose
    phi_delta_max = np.pi / 6.0  # radians, maximum orientation variation of the mobile base with respect to the theta angle

    # Footprint dimensions
    robot_width = 0.9  # meters
    robot_length = 0.7  # meters

    def __init__(self, navigator: BasicNavigator):
        """
        Constructor: create a node for the robot parking algorithm
        - subscribe to the target pose topic
        - initialize the NAV2 control API object
        - publish the optimal target goal computed from the target pose

        Parameters:
        ---------- 
            navigator (BasicNavigator): the NAV2 commander API object

        """
        super().__init__("robot_parking_node")

        # Subscriber to target pose topic
        # self.subscription = self.create_subscription(PoseStamped, "/target_pose", callback_target_pose, 10)
        # self.subscription  # prevent unused variable warning

        # Publisher for the optimal target goal computed from the target pose
        self.target_goal_pub = self.create_publisher(PoseStamped, "/target_goal", 10)

        # NAV2 control API
        self.navigator = navigator
        self.checker = FootprintCollisionChecker()

        self.footprint = Polygon()
        self.compute_footprint_polygon()

        self.target_available = False

        # Initial pose = starting point
        self.initial_pose = PoseStamped()

        # Create the TF2 transform listener
        self.map_frame = "map"
        self.robot_link_frame = "rear_mount"
        self.robot_center_frame = "mobile_robot_base_link"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)

        # find full path of custom behavior tree in scout_nav2
        self.bt_xml_path = os.path.join(
            get_package_share_directory("scout_nav2"),
            "params",
            "bt_nav2pose.xml",
        )

        self.get_logger().info("Robot parking node started and initialized")

    def update_target_pose(self, target_xyz: Point32, target_yaw: float):
        """Update the target pose given the target position and orientation

        Parameters:
        ----------
        target_xyz : geometry_msgs.msg.Point32
            the x,y,z coordinates of the target pose
        target_yaw : float
            the orientation angle of the target pose

        """
        self.target_xyz = target_xyz
        self.target_yaw = target_yaw
        self.target_available = True

    def parking_algorithm_then_navigate(self):
        """Main function thread

        Main function of the parking script, which initializes the navigation and parking algorithm.

        """
        # Set initial pose
        self.initial_pose = self.get_current_pose()
        self.navigator.setInitialPose(self.initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # self.navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

        global_costmap = self.navigator.getGlobalCostmap() # only static layer inflated
        costmap = PyCostmap2D(global_costmap)
        self.checker.setCostmap(costmap)

        # wait for first target to be available
        while not self.target_available:
            self.get_logger().info("Waiting for target pose...")
            time.sleep(0.5)
        self.target_available = False

        # compute optimal parking pose
        goal_pose = self.parking_algorithm()

        # publish the goal pose computed from the target pose
        if goal_pose is not None:
            self.target_goal_pub.publish(goal_pose)
        else:
            # iterate again with the next target candidate
            return

        # navigate to the parking pose chosen by the parking algorithm
        self.navigator.goToPose(pose=goal_pose, behavior_tree=self.bt_xml_path)

        # goToPose action definition
        # https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action

        i = 0
        feedback = None
        while not self.navigator.isTaskComplete():
            # Do something with the feedback
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                self.get_logger().info(
                    "Estimated time of arrival: "
                    + "{0:.2f}".format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    )
                    + " seconds."
                )
                self.get_logger().info("Distance remaining = {}".format(feedback.distance_remaining))
            i = i + 1

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")
        else:
            self.get_logger().info("Goal has an unknown return status!")

        self.get_logger().info("Distance remaining = {}".format(feedback.distance_remaining))

        # self.navigator.lifecycleShutdown()

    def parking_algorithm(self) -> PoseStamped:
        """Apply the parking algorithm to find the optimal parking pose

        Main function of the parking script, which computes the optimal parking pose given the target pose.

        Returns:
        -------
        goal_pose : PoseStamped
            the optimal parking pose found by the parking algorithm
        """

        # 1. random sample n possible x,y positions in a circle of radius r and centered in the goal pose
        target_candidates = np.empty((0, 3), float)  # x, y, theta

        position_samples = self.sample_positions()
        for position in position_samples:
            # 2. random sample m possible phi orientations, with pi < |phi| < pi/2
            orientation_samples = self.sample_orientations(position[0], position[1])
            for theta in orientation_samples:
                target_candidates = np.append(
                    target_candidates, [[position[0], position[1], theta]], axis=0
                )

        self.get_logger().info(f"initial target candidates number = : {target_candidates.shape[0]}")

        target_candidates, target_costs = self.filter_out_colliding_poses(target_candidates)

        self.get_logger().info(f"reduced candidates number = {target_candidates.shape[0]}")

        if target_candidates.shape[0] == 0:
            self.get_logger().error("No feasible parking poses found!")
            return None

        # rank the target candidates by lowest cost, lowest distance from the starting point and orientation closest to pi
        ranking_metrics = self.ranking_targets_by_cost_dist_theta(
            target_candidates, target_costs
        )

        # self.get_logger().info(f"computed target metrics:\n{ranking_metrics}")

        # sort the target candidates by the corresponding rank metric
        target_candidates = np.concatenate((target_candidates, target_costs), axis=1)
        target_candidates = np.concatenate((target_candidates, ranking_metrics), axis=1)
        # self.get_logger().debug(f"shape candidates with scores = {target_candidates.shape}")

        sorted_indices = np.argsort(target_candidates[:, -1])[::-1]
        sorted_targets = target_candidates[sorted_indices]
        self.get_logger().debug(f"computed sorted targets:\n{sorted_targets}")

        # iterate the first n target candidates until a feasible parking pose is found
        max_iterations = 10
        i = 0
        goal_pose = None
        while i < max_iterations:
            # use navigator to plan a path from the starting pose to the parking pose
            # and check if the path is feasible
            # if feasible, then navigate the robot to the parking pose
            # if not feasible, then iterate again with the next target candidate
            # if no feasible parking poses are found, return an error
            goal_pose = self.convert_to_pose_stamped(
                sorted_targets[i][0],
                sorted_targets[i][1],
                sorted_targets[i][2],
            )
            path = self.navigator.getPath(self.initial_pose, goal_pose)
            if path is not None:
                self.get_logger().info("Path to goal pose is feasible! Metric rank = {}".format(sorted_targets[i][-1]))
                break
            else:
                self.get_logger().info(f"Attempt {i} in checking path feasibility result in not feasible path!")
                i += 1

        if goal_pose is None:
            self.get_logger().error(f"No feasible paths found within {max_iterations} attempts!")

        return goal_pose

    # ---------------------------------------------------------------------------------------------
    # Helper functions
    # ---------------------------------------------------------------------------------------------

    def compute_footprint_polygon(self):
        """compute the footprint polygon of the robot given the width and length

        Compute the footprint polygon of the robot given the width and length
        """
        self.footprint.points = [Point32(), Point32(), Point32(), Point32()]
        self.footprint.points[0].x = -self.robot_length / 2
        self.footprint.points[0].y = -self.robot_width / 2
        self.footprint.points[1].x = self.robot_length / 2
        self.footprint.points[1].y = -self.robot_width / 2
        self.footprint.points[2].x = self.robot_length / 2
        self.footprint.points[2].y = self.robot_width / 2
        self.footprint.points[3].x = -self.robot_length / 2
        self.footprint.points[3].y = self.robot_width / 2

    def get_current_pose(self) -> PoseStamped:
        """Get the current pose of the robot from the TF2 transform from map -> robot center

        First wait for the listener to connect to the TF2 data, then get the pose of the link frame relative to the base frame.
        It then sets the initial pose of the robot to the computed pose.

        Returns:
        -------
        current_pose : PoseStamped
            the current pose of the robot

        """
        robot_transform = self.get_tf(source_frame=self.robot_center_frame, target_frame=self.map_frame)

        current_pose = PoseStamped()
        time_now = self.navigator.get_clock().now()
        current_pose.header.frame_id = self.map_frame
        current_pose.header.stamp = time_now.to_msg()
        current_pose.pose.position.x = robot_transform.transform.translation.x
        current_pose.pose.position.y = robot_transform.transform.translation.y
        current_pose.pose.position.z = robot_transform.transform.translation.z
        current_pose.pose.orientation = robot_transform.transform.rotation

        # convert initial pose orientation from quaternion to euler angles
        euler = euler_from_quaternion([
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
        ])

        self.get_logger().info(
            f"Current pose = {current_pose.pose.position.x:.3f} {current_pose.pose.position.y:.3f} {euler[2]:.3f}"
        )
        return current_pose

    def sample_positions(self) -> list[float]:
        """ sample xy_samples random positions in a circle of radius target_radius centered in the target pose

        The positions are sampled in a circle arc of radius target_radius centered in the target pose.
        The arc must be in the same direction of target_yaw angle, and spanning theta_delta_max radians.

        Returns:
        -------
        position_samples : list(float)
            the sampled positions in a vector

        """
        position_samples = []

        for i in range(self.theta_samples):
            # sample a random angle theta from -pi/6 to pi/6
            theta = np.random.uniform(- self.theta_delta_max, self.theta_delta_max) + self.target_yaw

            # correct theta angle such that it is in the range -pi to pi
            if theta > np.pi:
                theta -= 2 * np.pi
            elif theta < -np.pi:
                theta += 2 * np.pi

            # TODO: target radius variable with theta angle and z elevation

            # compute x,y coordinates
            x = self.target_xyz.x + self.target_radius * np.cos(theta)
            y = self.target_xyz.y + self.target_radius * np.sin(theta)
            # save the position
            position_samples.append([x, y])

        # return the sampled positions in a vector
        return position_samples

    def sample_orientations(self, goal_x: float, goal_y: float) -> np.array:
        """ random sample m possible phi orientations, with pi < |phi| < pi/2

        Parameters:
        ----------
        goal_x : float
            x coordinate of the target pose
        goal_y : float
            y coordinate of the target pose

        Returns:
        -------
        orientation_samples : np.array
            the sampled orientations in a vector
        """

        # base angle is the angle of the vector starting from the target pose and ending in the given new goal pose
        base_angle = np.arctan2(
            goal_y - self.target_xyz.y, goal_x - self.target_xyz.x
        )

        # sample phi_samples random orientations in the range - pi /2 < phi < pi/2
        orientation_samples = np.random.uniform(
            - self.phi_delta_max, self.phi_delta_max, size=self.phi_samples
        )

        orientation_samples = np.add(orientation_samples, base_angle)

        # return the sampled orientations in a vector
        return orientation_samples

    def compute_footprint_cost_at_pose(self, x: float, y: float, theta: float) -> int:
        """given a position and orientation, compute the footprint coordinates of the robot

        Parameters:
        ----------
        x : float
            x coordinate odometric center of the robot
        y : float
            y coordinate odometric center of the robot
        theta : float
            orientation angle of the robot

        Returns:
        -------
        cost : int
            the cost of the robot footprint at the given pose

        """

        cost = self.checker.footprintCostAtPose(x=x, y=y, theta=theta, footprint=self.footprint)
        if cost >= 252:
            self.get_logger().debug(f"Footprint cost is too high: {cost}")
        return cost

    def get_tf(self, source_frame: str, target_frame: str) -> TransformStamped:
        """Get the transform from the reference frame to the robot center

        Parameters:
        ----------
        source_frame : str
            the source frame from which to compute the transform
        robot_frame : str
            the target frame of interest

        Returns:
        -------
        transform : geometry_msgs.msg.TransformStamped
            the transform from the source frame to the target frame

        """

        # Wait for the listener to connect to the TF2 data
        time_now = rclpy.time.Time()
        while not self.tf_buffer.can_transform(
            source_frame=source_frame,
            target_frame=target_frame,
            time=time_now,
            timeout=rclpy.duration.Duration(seconds=0.1),
        ):
            self.get_logger().debug(
                f"Waiting for transform from map to {target_frame} ..."
            )
            time.sleep(0.2)
            time_now = rclpy.time.Time()

        # Get the pose of the link frame relative to the base frame
        try:
            transform = self.tf_buffer.lookup_transform(
                source_frame=source_frame,
                target_frame=target_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not transform tf: {ex}")
            time.sleep(0.5)

        return transform

    def convert_to_pose_stamped(self, x: float, y: float, theta: float) -> PoseStamped:
        """ convert x,y,theta coordinates to PoseStamped

        Parameters:
        ----------
        x : float
            x coordinate in the robot_link frame
        y : float
            y coordinate in the robot_link frame
        theta : float
            orientation angle

        Returns:
        -------
        pose : PoseStamped
            the pose in PoseStamped format

        """

        # get the transform from the robot chosen frame link to the robot center frame
        tf_mount_robot = self.get_tf(source_frame=self.robot_center_frame, target_frame=self.robot_link_frame)

        # apply the transform to the x,y coordinates, to get the final pose of the robot, in the robot center frame
        x = x + tf_mount_robot.transform.translation.x * \
            np.cos(theta) - tf_mount_robot.transform.translation.y * np.cos(theta)
        y = y + tf_mount_robot.transform.translation.y * \
            np.sin(theta) + tf_mount_robot.transform.translation.x * np.sin(theta)

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(theta / 2.0)
        pose.pose.orientation.w = np.cos(theta / 2.0)
        return pose

    def filter_out_colliding_poses(self, target_candidates: np.array) -> tuple[np.array, np.array]:
        """filters out the parking poses colliding with any walls or obstacles

        removes the rows from the target_candidates corresponding to the colliding poses
        returns the filtered target_candidates and the corresponding target_costs

        Parameters:
        ----------
        target_candidates : np.array
            the target candidates positions and orientations

        Returns:
        -------
        target_candidates : np.array
            the filtered target candidates positions and orientations
        target_costs : np.array
            the corresponding target costs

        """
        # filter out the parking poses colliding with any walls or obstacles.
        target_costs = np.empty((0, 1), dtype=np.float32)
        total = target_candidates.shape[0]
        row = 0
        while row < total:
            x = target_candidates[row][0]
            y = target_candidates[row][1]
            theta = target_candidates[row][2]
            cost = self.compute_footprint_cost_at_pose(x, y, theta)

            if cost > self.INSCRIBED_INFLATED_OBSTACLE:  # collision detected => lethal obstacle
                target_candidates = np.delete(target_candidates, row, axis=0)
                total -= 1
            else:
                # save the cost of the targets computed
                cost = np.array([[cost]], dtype=np.float32)
                target_costs = np.append(target_costs, cost, axis=0)
                row += 1

        return target_candidates, target_costs

    def ranking_targets_by_cost_dist_theta(self, target_candidates: np.array, target_costs: np.array) -> np.array:
        """Ranking algorithm given the target candidates and their costs

        Apply ranking algorithm: maximize weighted sum of normalized cost, distance and orientation
        - lower cost means more distant from the obstacles
        - lower distance means closer to the target pose
        - lower orientation delta means closer to the base angle (more space for the robot arm to move around)
        returns a vector of rank metrics corresponding to the target candidates

        Parameters:
        ----------
        target_candidates : np.array
            the target candidates
        target_costs : np.array
            the corresponding target costs

        Returns:
        -------
        rank_metrics : np.array
            the rank metrics corresponding to the target candidates

        """

        weight_cost = 0.15
        weight_distance = 0.15
        weight_orientation = 0.7
        max_distance = 16.0  # meters

        # zip the target candidates with their costs
        target_candidates = np.concatenate(
            (target_candidates, target_costs), axis=1)
        rank_metrics = np.empty((0, 1), float)

        for x, y, theta, cost in target_candidates:
            cost_norm = 1.0 - cost / 254.0  # lower cost means higher value

            distance = np.sqrt(
                (x - self.target_xyz.x) ** 2 +
                (y - self.target_xyz.y) ** 2
            )
            # 10 meters is the maximum distance from the target pose
            distance_norm = 1.0 - distance / max_distance

            orientation = np.arctan2(y - self.target_xyz.y, x - self.target_xyz.x)
            # delta from the base angle = direction from target pose to the new goal pose
            orientation_delta = np.abs(orientation - theta)
            # pi/2 is the maximum orientation angle delta
            orientation_norm = 1.0 - orientation_delta / (np.pi / 2.0)

            # compute the rank metric as a weighted sum of the normalized cost, distance and orientation
            rank_metric = (
                weight_cost * cost_norm
                + weight_distance * distance_norm
                + weight_orientation * orientation_norm
            )
            rank_metrics = np.append(rank_metrics, [[rank_metric]], axis=0)

        return rank_metrics

    def compute_distance_error(self, pose: PoseStamped, expected_pose: PoseStamped) -> float:
        """Compute the distance error between two poses

        Parameters:
        ----------
        pose : PoseStamped
            the current pose
        expected_pose : PoseStamped
            the expected pose

        Returns:
        -------
        distance : float
            the distance error between the two poses

        """
        distance = np.sqrt(
            (pose.pose.position.x - expected_pose.pose.position.x) ** 2
            + (pose.pose.position.y - expected_pose.pose.position.y) ** 2
        )
        return distance

    def compute_heading_error(self, pose: PoseStamped, expected_pose: PoseStamped) -> float:
        """Compute the heading error between two oriented poses

        Parameters:
        ----------
        pose : PoseStamped
            the current pose
        expected_pose : PoseStamped
            the expected pose given as the target pose

        Returns:
        -------
        heading_delta : float
            the heading error between the two poses orientations

        """
        # compute the heading error from the parking goal, assuming the same reference frame
        yaw_pose = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ])[2]
        yaw_expected = euler_from_quaternion([
            expected_pose.pose.orientation.x,
            expected_pose.pose.orientation.y,
            expected_pose.pose.orientation.z,
            expected_pose.pose.orientation.w,
        ])[2]
        heading_delta = np.abs(yaw_pose - yaw_expected)
        return heading_delta

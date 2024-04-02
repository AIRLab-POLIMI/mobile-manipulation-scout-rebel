#!/usr/bin/env python3

"""
Author: Simone Giampà
University: Politecnico di Milano
Master Degree: Computer Science Engineering
Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

"""

# ROS2 imports
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

# python imports
import cv2
from cv_bridge import CvBridge


class TargetClick(Node):

    def __init__(self):
        super().__init__('target_click_node')

        # create subscription to /camera/color/image_raw topic
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_image_callback, 10)
        self.rgb_sub  # prevent unused variable warning

        self.timer_period = 1.0 / 30.0  # 30Hz [ms]
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # openCV for image visualization
        self.bridge = CvBridge()
        self.window_name = "RGB Feed"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_click_callback)
        self.cv_image = None

    def rgb_image_callback(self, img_msg: Image):
        """Subscriber callback to RGB image

        Parameters:
        ----------
        msg : Image
            the RGB image message received from the topic

        """
        # do something with the image
        self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

    def timer_callback(self):
        """Timer callback

        Callback function for timer event
        """
        #self.get_logger().info(f"displaying image... {self.cv_image is not None}")
        if self.cv_image is not None:
            cv2.imshow(self.window_name, self.cv_image)
            cv2.waitKey(1)

    def mouse_click_callback(self, event, x, y, flags, param):
        """Mouse click callback

        Callback function for mouse click event
        """
        if (event == cv2.EVENT_LBUTTONDOWN):
            self.get_logger().info(f"Mouse click at x: {x}, y: {y}")


def main(args=None):
    rclpy.init(args=args)
    node = TargetClick()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

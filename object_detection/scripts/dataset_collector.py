#!/usr/bin/env python3

"""
Author: Simone Giampà
University: Politecnico di Milano
Master Degree: Computer Science Engineering
Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

"""

# ROS2 python imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV python imports
import cv2
from cv_bridge import CvBridge

# Python imports
import os


class DatasetCollector(Node):

    def __init__(self):
        super().__init__('dataset_collector')

        self.logger_ = self.get_logger()

        # read image topic from the parameter server
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.logger_.info(f"RGB topic: {self.rgb_topic}")

        self.subscription = self.create_subscription(
            Image, self.rgb_topic, self.rgb_img_callback, 10)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Initialize image counting and saving directory
        self.image_count = 0
        self.save_directory = '/home/simongiampa/dataset_apples/'  # Folder to save images
        os.makedirs(self.save_directory, exist_ok=True)  # Create if doesn't exist

        # Find the last image index
        self.image_count = self.get_last_image_count() * 15

    def get_last_image_count(self):
        """Finds the last saved image index (if any)"""
        file_prefix = "data_"
        file_extension = ".png"  # Assuming PNG images

        last_index = -1  # If no images are found
        for filename in os.listdir(self.save_directory):
            if filename.startswith(file_prefix) and filename.endswith(file_extension):
                try:
                    index = int(filename[len(file_prefix):-len(file_extension)])
                    last_index = max(last_index, index)
                except ValueError:
                    pass  # Ignore files with non-numeric names
        
        return last_index + 1

    def rgb_img_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("RGB Feed", cv_image)
        self.image_count += 1
        if (self.image_count % 15) == 0:
            # Save the image
            img_n = self.image_count // 15
            img_name = f"data_{img_n:04d}.png"
            self.logger_.info(f"Saving image {img_name}")
            filename = os.path.join(self.save_directory, f"{img_name}")
            cv2.imwrite(filename, cv_image)

        cv2.waitKey(1)  # Display briefly


def main(args=None):
    rclpy.init(args=args)
    dataset_collector = DatasetCollector()
    rclpy.spin(dataset_collector)
    dataset_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

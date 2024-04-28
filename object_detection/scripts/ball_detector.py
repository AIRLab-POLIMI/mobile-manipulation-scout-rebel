#!/usr/bin/env python3

"""
Author: Simone Giampà
University: Politecnico di Milano
Master Degree: Computer Science Engineering
Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

"""

# ROS2 imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory

# messages imports
from sensor_msgs.msg import Image

# custom ROS2 interfaces
from mobile_manipulation_interfaces.msg import ObjectDetections

# OpenCV python imports
import cv2
from cv_bridge import CvBridge

# Tensorflow imports
import tensorflow as tf
import keras
from keras_cv.models import object_detection

# python imports
import threading
import os
import numpy as np


class BallDetector(Node):

    TEXT_COLOR = (255, 255, 255)  # White
    COLOR_MAPPING = {
        0: (255, 0, 0),  # blue
        1: (0, 255, 0),  # green
        2: (0, 0, 255),  # red
        3: (0, 250, 255),  # yellow
    }

    def __init__(self):
        super().__init__('ball_detector_node')

        self.initialize_parameters()

        # create subscription to /camera/color/image_raw topic
        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_image_callback, 10)
        self.rgb_sub  # prevent unused variable warning

        # refresh image in cv2 window at 30Hz within a timer callback
        self.timer_period = 1.0 / 30.0  # 30Hz [ms]
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # create publisher of target pose coordinates
        self.detections_pub = self.create_publisher(ObjectDetections, '/object_detections', 10)

        self.load_model_inference()

        # openCV for image visualization
        self.bridge = CvBridge()
        self.window_name = "RGB Feed"
        cv2.namedWindow(self.window_name)
        self.cv_image = None
        self.cv_predicted = None

        # create continuously running thread for performing inference with the trained model
        self.inference_thread = threading.Thread(target=self.inference_thread)
        self.inference_thread.start()

    def rgb_image_callback(self, img_msg: Image):
        """Subscriber callback to RGB image

        Parameters:
        ----------
        msg : Image
            the RGB image message received from the topic

        """
        # do something with the image
        self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")

    def load_model_inference(self):
        # class mapping = dictionary from class index to class name
        self.class_mapping = {
            0: "blue_ball",
            1: "green_ball",
            2: "red_ball",
            3: "yellow_ball"
        }

        # print the tensorflow version and the available GPU devices
        self.get_logger().info(f"Tensorflow version: {tf.__version__}")
        self.get_logger().info("GPU is available" if tf.config.list_physical_devices('GPU') else "GPU NOT AVAILABLE")
        self.get_logger().info(f"Keras version: {keras.__version__}")

        # load the trained model for inference
        self.model_path = os.path.join(
            get_package_share_directory("object_detection"),
            "models",
            "yolov8s_600_v2.keras"
        )
        self.yolo_model = tf.keras.models.load_model(self.model_path)
        self.get_logger().info("Model loaded successfully!")
        self.yolo_model.compile(
            classification_loss="binary_crossentropy",  # multi-hot encoded labels require this loss function
            box_loss="ciou",  # complete intersection over union
            # disable just in time compilation with XLA for compatibility issues using GPU acceleration during training,
            # due to CombinedNonMaxSuppression op not currently supported by tensorflow
            jit_compile=False
        )
        self.get_logger().info("Model compiled successfully!")

    def inference_thread(self):
        rate = self.create_rate(1)
        while (rclpy.ok()):
            if self.cv_image is not None:
                # perform inference with the trained model
                # and publish the detected object coordinates
                img = np.expand_dims(self.cv_image, axis=0).astype(np.uint8)

                # measure the inference time
                start_time = cv2.getTickCount()

                # perform inference with the trained model
                predictions = self.yolo_model.predict(img)

                # measure the inference time
                end_time = cv2.getTickCount()
                inference_time = (end_time - start_time) / cv2.getTickFrequency()
                self.get_logger().info(f"Inference time: {inference_time:.3f} s")

                # extract the first index at which -1 is found in the labels list
                # filter out all elements after the first occurrence of -1
                # meaning that the rest of the elements are invalid predictions
                labels = np.array(predictions["classes"][0]).astype(np.int32)
                invalid_index = labels.tolist().index(-1)
                labels = labels[:invalid_index].astype(np.uint16)
                bboxes = np.array(predictions["boxes"][0])[:invalid_index, :]
                confidence = np.array(predictions["confidence"][0])[:invalid_index]

                # visualize the detected object predictions
                self.cv_predicted = self.visualize_single_pred(img, bboxes, labels, confidence)

                # publish the detected object coordinates
                object_detections = ObjectDetections()
                object_detections.header.stamp = self.get_clock().now().to_msg() # current time
                object_detections.header.frame_id = "camera_link" # not actually used
                # linearize the bounding boxes vector
                # and store the detected object coordinates with their respective class labels
                object_detections.bounding_boxes = bboxes.flatten().tolist()
                object_detections.labels = labels.tolist()
                object_detections.confidences = confidence.tolist()

                # publish the detected object coordinates
                self.detections_pub.publish(object_detections)

            else:
                # sleep until a new image is received
                self.get_logger().info("Waiting for a new image...")
                rate.sleep()

    def visualize_single_pred(self, image, bboxes, labels, confidences):
        """Visualizes a batch of images with bounding boxes in a single plot."""
        batched_image = image.astype(np.uint8).copy()
        image = np.squeeze(batched_image, axis=0)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        for bbox, label, confidence in zip(bboxes, labels, confidences):
            class_name = self.class_mapping[label]
            image = self.visualize_bbox(
                image,
                bbox,
                class_name,
                confidence=f"{confidence:.2f}",
                color=self.COLOR_MAPPING[label],
            )
        return image

    def visualize_bbox(self, img, bbox, class_name, color, confidence=""):
        """Visualizes a single bounding box on the image"""
        x_min, y_min, w, h = bbox
        x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)

        # display rectangle with class name and confidence
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color=color, thickness=2)
        display_text = (class_name + " | " + confidence) if confidence != "" else class_name
        ((text_width, text_height), _) = cv2.getTextSize(display_text, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
        cv2.rectangle(img, (x_min, y_min - int(1.3 * text_height)), (x_min + text_width, y_min), color, -1)
        cv2.putText(
            img,
            text=display_text,
            org=(x_min, y_min - int(0.3 * text_height)),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.35,
            color=self.TEXT_COLOR,
            lineType=cv2.LINE_AA,
        )
        return img

    def timer_callback(self):
        """Timer callback

        Callback function for timer event
        """

        # self.get_logger().info(f"displaying image... {self.cv_image is not None}")
        if self.cv_predicted is not None:
            # display processed image with detected object predictions
            cv2.imshow(self.window_name, self.cv_predicted)
            cv2.waitKey(1)
        elif self.cv_image is not None:
            # display raw image
            image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            cv2.imshow(self.window_name, image)
            cv2.waitKey(1)

    def initialize_parameters(self):
        # Declare and read parameters from config/camera.yaml
        self.declare_parameter(
            name="rgb_topic",
            value="/camera/color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="RGB image topic to subscribe to",
            ),
        )

        # read parameters from config/camera.yaml and store them
        self.rgb_topic = (
            self.get_parameter("rgb_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"RGB image topic: {self.rgb_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

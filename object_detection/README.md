# Object Detection with YOLOv8 trained with TensorFlow and Keras3, wrapped in a ROS2 package

This package contains the implementation of training and inference with an object detection neural network: **YOLOv8**.
This neural network is trained using *TensorFlow* and *Keras3*, while the model is provided by *KerasCV*.
The neural network is used to detect the object to be grasped in the RGB image video feed. The network is a trained model
used inside a ROS2 package for inference only. The ball detection node uses Tensorflow to load the trained model and
perform inference on the received images. For each image that the network processes, it publishes on a topic the bounding boxes of the 
detected objects, the predicted classes and the confidence scores. The package is agnostic to the classes of the objects to be detected,
and it can be easily adapted to detect other objects.

#### Contributor
Code developed and tested by: __Simone Giampà__

Project and experimentations conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis Project at Politecnico di Milano, Italy._

## Model Training with Tensorflow and Keras3

The YOLOv8 model is provided by KerasCV, a high-level library for computer vision tasks. The model is trained on a custom dataset
of images containing the objects to be detected. The dataset is augmented with offline data augmentations to increase the
variability of the training data. The training process is done on a Jupyter notebook, which is provided in the `notebooks` folder.
The notebook contains the following steps:
1. Load the dataset of images and annotations
2. Preprocess the dataset: save labels and bounding boxes read from the annotations
3. Convert the bounding boxes to the COCO format for training
4. Split the dataset into training and validation sets
5. Define the offline data augmentations pipeline, using the `Albumentations` library
6. Augment the dataset with the offline data augmentations, and save the augmented images and annotations
7. Load the augmented dataset and annotations
8. Define the YOLOv8 model with the KerasCV library
9. Compile the model with the Adam optimizer and the proper loss functions: CIoU box loss and Binary Cross-Entropy Focal loss for classification
10. Train the model on the augmented dataset
11. Evaluate the model on the validation set
12. Save the trained model to disk and load it for testing the inference step

## Usage

To run the object detection node, it is required to start the ROS2 wrapper for the object detection inference using YOLOv8. 
The node subscribes to the RGB image topic, processes the images with the YOLOv8 model, and publishes the detected objects on a topic.
The node also displays the images with the detected objects and their relative classes in a window using OpenCV.
At each image processed, the node outputs the time taken to execute the inference step.

To run the object detection node, execute the following command:
```bash
$ ros2 launch object_detection ball_detection.launch.py
```

It will start up the camera, read the parameters inside `soft_grasping/config/camera.yaml`, and start the object detection wrapper node.

It is recommended to run the inference node on a machine with a GPU, as the inference process is computationally expensive.

## Scripts

- `ball_detector.py`: ROS2 wrapper node for object detection with YOLOv8. The node subscribes to the RGB image topic, processes the images
  with the YOLOv8 model, and publishes the detected objects on a topic. The custom message definition is specified in the
  `mobile_manipulation_interfaces` package. The message contains information about the bounding boxes of the detected objects,
  the predicted classes and the confidence scores. The node also displays the images with the detected objects and their relative
  classes in a window using OpenCV.
- `dataset_collector.py`: ROS2 node for collecting images from the camera and saving them to disk. The node subscribes to the RGB
  image topic and saves the images to disk. The images are saved in the `dataset` folder in the package directory. The images are
  saved in the `.png` format, and the filenames contain the image acquisition ordered number.
  

## Package structure

```
object_detection
├── README.md
├── CMakeLists.txt
├── package.xml
│
├───models # not included in the repository: the trained models are private, and too large to be included in the repository
│   ├── yolov8_100.keras # high-level code for the YOLOv8 model trained on an augmented dataset of ball images, with 100 original images
│   ├── yolov8_600.keras # high-level code for the YOLOv8 model trained on an augmented dataset of ball images, with 600 original images
│
├───launch
│   ├── ball_detection.launch.py # launch file for the object detection node
│   ├── dataset_collector.launch.py # launch file for the dataset collector node
│
├───notebooks
│   ├── yolov8-training.ipynb # Jupyter notebook for training YOLOv8 on a custom dataset with offline data augmentations
│
├───scripts
│   ├── ball_detector.py # ROS2 node for object detection with YOLOv8
│   ├── dataset_collector.py # ROS2 node for collecting images from the camera and saving them to disk
```


## Dependencies

It is required to install `TensorFlow`, `Keras3`, `KerasCV` and `OpenCV` to run the object detection node.

It is recommended to run the Jupyter Notebook on a machine with a GPU, as the training process is computationally expensive. The
notebook requires the `Albumentations` library for offline data augmentations. The notebook was run in a Kaggle notebook with 
a GPU accelerator, in an instance with all the required libraries pre-installed.

# Robust and Noise-Tolerant Plane Detection using Multiple Coplanar Aruco Markers

Given a multi-aruco setup, consisting of multiple Aruco markers placed on the same plane, this package computes the plane 
of the aruco markers and their correct and precise orientation. The package is designed to be robust to noise 
and outlier data points. The algorithm is meant to compensate lack of precision in the markers' pose estimation,
which is often highly imprecise if the markers are far from the camera or if the camera is not well calibrated.
This system allows estimating the correct orientation of multiple markers at the same time, provided that they are
coplanar and have the same orientation.

#### Contributor
Code developed and tested by: __Simone Giampà__

Project and experimentations conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis Project at Politecnico di Milano, Italy._

## Description

### Multi Aruco Setup

The setup used for these tests is composed of 7 markers in a plane, 2 in the top row, and 5 in the bottom row.
The markers are used to estimate the positions of the buttons on the setup, which is used for robotic manipulation tasks.

The system is:
- **Robust to noise**: SVD and PCA algorithms are used to estimate the plane and the orientation of the multi-aruco setup. 
  These algorithms are robust to noise and can handle a large number of markers.
- **Fast**: operates at the same frequency as the camera driver, but can easily be adapted to work at a higher frequency.
- **Robust to outliers**: the RANSAC algorithm is used to estimate the plane of the multi-aruco setup. This algorithm is 
  robust to outliers and can discard non-relevant data points outside the expected data distribution.

### Algorithm Definition

The algorithm works as follows:
1. Given the Aruco Markers estimated (positions and orientations) poses in the input image, processes the data to estimate the
   most precise and correct orientation of the multi-aruco setup, assuming that all markers are placed on the same plane.
2. The algorithm estimates the plane of the Aruco markers using a RANSAC algorithm. Each iteration of the RANSAC algorithm
   estimates a plane using a random subset of Aruco markers. The plane with the most inliers is selected as the best plane.
3. The algorithm used for the estimation of the plane coordinates is based on SVD (*Singular Value Decomposition*) of the markers
   positions. The SVD algorithm is used to estimate the vector normal to the plane.
4. The normal vector to the plane corresponds to the z-axis of the multi-aruco setup.
5. Then the algorithm estimates the x,y-axes orientation of the plane using the other markers positions. The x-axis orientation
   is estimated using the PCA algorithm (*Principal Component Analysis*) of the bottom markers positions projected onto the plane.
6. The y-axis orientation is estimated using the cross-product between the computed z-axis and the x-axis.
7. Then the final orientation of the multi-aruco setup is obtained by converting the transformation matrix of the vectors into a
    rotation quaternion. The orientation is then applied to all aruco markers.

## Usage

Launch the Aruco pose estimator, the camera ROS2 driver node and the multi-aruco plane detection node with:

```bash
ros2 launch multi_aruco_plane_detection multi_aruco_plane_detection.launch.py
```

The parameters for the aruco detection and pose estimation can be set in the `config/aruco_parameters.yaml` file, 
in the `ros2-aruco-pose-estimation` repository, or they can be optionally set as command line arguments in the launch file mentioned above.

## Installation: build from source

Install the following dependencies:

```bash
sudo apt install ros-$ROS_DISTRO-eigen3-cmake-module
```

Install the Eigen3 library with:

```bash
sudo apt install libeigen3-dev
```


## Project structure

The project is structured as follows:

```
multi_aruco_plane_detection
│
├── include
│   └── multi_aruco_plane_detection.hpp # Header file for the multi-aruco plane detection class
│
├── src
│   └── multi_aruco_plane_detection.cpp # Source file for the multi-aruco plane detection class
│
├── launch
│   └── multi_aruco_plane_detection.launch.py # Launch file for the multi-aruco plane detection node
│
├── rviz
│   └── multi_aruco.rviz # RViz configuration file for the multi-aruco plane detection node
│
├── CMakeLists.txt
├── package.xml
└── README.md
```
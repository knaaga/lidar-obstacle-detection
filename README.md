# Object Detection using LIDAR

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Objective

The goal of this project is to process raw lidar data and detect obstacles in a driving environment. The lidar data is in the form of point clouds. The point clouds are processed using filtering, segmentation and clustering techniques. Specifically, RANSAC with planar model fitting and Euclidean clustering with a KD-Tree are used to segment and cluster the point clouds.

## Versions

* Ubuntu 16.04
* PCL - v1.7.2
* C++ v11
* gcc v5.5

## Local Installation

### Ubuntu 

1. Clone this github repo:

   ```sh
   cd ~
   git clone https://github.com/knaaga/Lidar_Obstacle_Detection.git
   ```
2. Execute the following commands in a terminal

   ```shell
   sudo apt install libpcl-dev
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```



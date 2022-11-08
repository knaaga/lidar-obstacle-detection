# LIDAR Obstacle Detection

## Project Overview
This project primarily deals with [Lidar](https://en.wikipedia.org/wiki/Lidar) data processing and obstacle detection in a city driving environment. The lidar data is in the form of [point clouds](https://en.wikipedia.org/wiki/Point_cloud). The point cloud data (PCD) is processed using filtering, segmentation and clustering techniques. Specifically, RANSAC with planar model fitting and KD-Tree based Euclidean clustering are used to segment and cluster the point clouds.

https://user-images.githubusercontent.com/49369282/200521059-b8ac1380-92bb-407b-bdf8-2e68c11948c4.mp4

#### Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* PCL - v1.7.2
  * [Documentation](https://pointclouds.org/downloads/)

#### Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./environment`.

## Table of Contents
- [The Lidar Model and Simulated Point Clouds](#lidar)
- [Real World Point Cloud Data](#realworldPCD)
- [Planar Segmentation](#segmentation)
- [Euclidean Clustering](#clustering)
- [Bounding Boxes](#boxes)
- [Acknowledgements](#acknowledgements)

## The Lidar Model and Simulated Point Clouds <a name="lidar"></a>
To simulate the point cloud generation process, a simple lidar model is used. This model takes in parameters such as max and min ray distance, angular resolution, a surrounding cars vector, etc. A set of rays are generated using these parameters. The lidar scan function implements raycasting each of these rays are checked for collision with other cars or the ground plane. The scan function returns a point cloud with some Gaussian noise added in. A simple highway scene that explains this is shown below

***INSERT SIMPLE HIGHWAY LIDAR SCAN IMAGE***

## Real World Point Cloud Data <a name="realworldPCD"></a>
The different point cloud processing techniques described in the subsequent section can be applied to the simulated set of point clouds that are obtained using the previously described lidar model. This can serve as a simplified learning experience and can be used to debug and the different algorithims involved. However going forward, these techniques will be applied on real world PCD data obtained from an actual lidar. 

Also, the same techniques used to process a single point cloud dataset is then extended to process a stream of incoming point clouds

***INSERT HIGH RES LIDAR SCAN***

The image above shows a high resolution point cloud that spans a large distance. In order for the processing pipeline to be able to digest the data as quickly as possible, the point cloud will have to be filtered down. There are two key techniques involved here:

### 1. Voxel Grid Filtering
This technique utilizes a voxelized grid approach to reduce the number of points in the dataset. It first creates a 3D voxel grid (a voxel can be thought of a small 3D box in space), with the resolution being controlled by input parameters. The resolution should be low enough to help speed up processing, but not so low that object definition is completely lost. Then, all the points in each of these voxels will be approximated with their centroid. 

### 2. Region of Interest Cropping
The lidar scan extends over a large distance from the ego vehicle. This can be cropped to retain only useful information and hence reduce processing time. An appropriate region of interest includes a good amount of space in front of the ego vehicle so that it is able to react quickly in time to any obstacles moving towards it. For the sides, at least the width of the road should be covered. It would also be beneficial to remove points that are hitting the roof of the ego car. 

The filtered and cropped point cloud data is shown below

***INSERT FILTERED AND CROPPED LIDAR SCAN***

## Planar Segmentation <a name="segmentation"></a>
One of the key objectives of lidar point cloud processing is to separate the road plane from potential obstacles. To achieve this, planar segmentation based on the random sampling consensus (RANSAC) algorithm is used. 

### RANSAC
RANSAC stands for random sampling consensus and is a method for detecting outliers in data. The algorithm runs for a set number of iterations and returns the model that best fits the data. Each of these iterations randomly picks a subset of data and fits a model such as a line or plane through it. The iteration with the highest number of inliers or lower noise is then used as the best model. 

There are different variations to the RANSAC algorithm. One type selects the smallest possible subset of points to fit. For a line, that would be two points, and for a plane three points. Then the number of inliers are counted, by iterating through every remaining point and calculating its distance to the model. The points that are within a certain distance to the model are counted as inliers. The iteration that has the highest number of inliers is then the best model

Other methods of RANSAC could sample some percentage of the model points, for example 20% of the total points, and then fit a line to that. Then the error of that line is calculated, and the iteration with the lowest error is the best model. This method might have some advantages since not every point at each iteration needs to be considered. It’s good to experiment with different approaches and time results to see what works best. The following graphic shows a 2D RANSAC algorithm

***INSERT 2D RANSAC GIF***

The output of the planar segmentation process is a pair of point clouds - one that represents the road and the other than represents obstacles. The segmented PCD data is shown below

***INSERT SEGMENTED PCD***

## Euclidean Clustering <a name="clustering"></a>
Once the obstacles and road points have been segmented, the next step is to cluster the points that represent the different obstacles. One way to do this is the Euclidean clustering algorithm. The idea here is to create association between groups of points depending on how close they are. This involves performing a nearest neighbor search and to do this efficiently, a data structure such as a KD-Tree is required. 

### KD-Tree
A KD-Tree is a K-dimensional binary search tree that organizes data spatially by splitting points between alternating dimensions. By doing this, KD-Tree enables efficient nearest neighbor search with a time complexity of O(log(n)) as opposed to O(n). This is primarily because, by grouping the points into regions in a KD-Tree, the search space is narrowed down drastically and expensive distance computations for potentially thousands of points can be avoided. The algorithm used to construct a KD-Tree is explained [here](https://www.geeksforgeeks.org/k-dimensional-tree/). The 2D points before and after spacial splitting are shown below. Here, the blue lines indicate X dimension splits and red lines indicate Y region splits

***INSERT BEFORE AND AFTER KD TREE SEPARATION***

Once points are able to be inserted into the tree, the next step is being able to search for nearby points inside the tree compared to a given target point. Points within a distance tolerance are considered to be nearby. 

The naive approach of finding nearby neighbors is to go through every single point in the tree and compare their distances with the target, selecting point indices that fall within the distance tolerance of the target. 

Instead with the KD-Tree, a boxed square of size 2 X distance tolerance, centered around the target point is used. If the current node point is within this box, only then the Euclidean distance is calculated and depending on this, it can be determined if the point should be added to the list of nearby points. Further, if this box does not cross over the node division region, the branch on the other side of the region is completely skipped. If it crosses over, then that side is recursively explored. This is shown in the image below. The red crosses indicate regions which were entirely skipped.

***INSERT KD TREE SEARCH***

Once the KD-Tree method for searching nearby points is implemented, the next step is to implement a euclidean clustering method that groups individual cluster indices based on their proximity. To perform the clustering, the following approach can be used:
* Iterate through each point in the cloud and keep track of which points have been processed already. 
* For each unprocessed point add it to a list of points defined as a cluster, then get a list of all the points in close proximity to that point by searching the tree
* For each unprocessed point in close proximity, add it to the cluster and repeat the process of calling proximity points. 
* Once the recursion stops for the first cluster, create a new cluster and move through the point list, repeating the above process for the new cluster.
* Once all the points have been processed, there will be a certain number of clusters found. Return as a list of clusters.

The clustered 2D space is shown below

***INSERT KD TREE CLUSTERED***

The clustered real world PCD data is shown below

***INSERT REAL WORLD PCD CLUSTERED***

## Bounding Boxes <a name="boxes"></a>
As a final touch, bounding boxes can be added around the clusters. The bounding box volume could also be thought of as space the car is not allowed to enter, or it would result in a collision.

***INSERT BOUNDING BOX IMAGE***

In this method of generating bounding boxes, the boxes are always oriented along the X and Y axis. This is acceptable if the cluster has the majority of its points oriented along these axes. However, if the cluster has a very long rectangular object at a 45 deg angle to the X axis, then the resulting bounding box would be unnecessarily large and would constrain the ego vehicle's available space to move around

***INSERT EFFICIENT BOX IMAGE***

In the image above, the bounding box on the right and is more efficient, taking into account the rotation about the Z axis and containing all the points with the minimum area required. This project does not include code for generating such a box, but techniques like [principal components analysis](https://en.wikipedia.org/wiki/Principal_component_analysis) can be used to identify the primary axis of the points and a quaternion member can be used for the rotation




## Ackowledgements <a name="acknowledgements"></a>
* [Udacity Sensor Fusion Program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)





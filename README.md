# Sensor Fusion Nanodegree (Udacity)
## Lidar Obstacle Detection

This project contains code that demonstrates techniques of working with the real point cloud data collected with the Lidar sensor. 

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
```
Create a project folder (ex. project1)
```bash
$> cd project1
$> git clone https://github.com/pranavbajaj/Lidar-Obstacle-Detection.git
```
Delete the build folder
```bash
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)

~~~
#### By default, the project is running on custom implemented code for segmenting and clustering. To run the project on built-in functions, change the following parameter from the cityBlock function in environment.cpp (~/Lidar-Obstacle-Detection/src/environment.cpp).

***bool clustering_builting_function***: If "false", the project runs on custom code for clustering and if "true", the project runs on builtin function. 

***bool segmentation_builtin_function***: If "false", the project runs on custom code for segmenting plan and if "true", the project runs on builtin function. 

**Other Parameters**

***bool render_cluster***: if "true", the object clusters will be displayed. 

***bool render_box***: if "true", regular bounding box will appear around the objects. 

***bool BOXQ***: if "true", BoxQ box will appear around the objects.


### Point Cloud Data processing

**1.Filtering**: Data is filtered so that it can be processed in realtime. Voxel Grid and Region of Interest filtering. 
Voxel Grid: Voxel grid filtering will create a cubic grid and will filter the cloud by only leaving a single point per voxel cube, so the larger the cube length the lower the resolution of the point cloud.
Region of Interest: A boxed region is defined and any points outside that box are removed.

**2. Segmentation**: If the road is flat it’s fairly straightforward to pick out road points from non-road points. To do a method called Planar Segmentation which uses the RANSAC (random sample consensus) algorithm. Road and Obstacle data are seperated into two different point clouds.
RANSAC: RANSAC stands for Random Sample Consensus, and is a method for detecting outliers in data. RANSAC runs for a max number of iterations, and returns the model with the best fit. Each iteration randomly picks a subsample of the data and fits a model through it, such as a line or a plane. Then the iteration with the highest number of inliers or the lowest noise is used as the best model.

**3.Clustering**: All the objects are seperated into individual Point Cloud Data (PCD) from the objects PCD obtained from Segementation. KD-tree and Euclidean Clustering algorithm are used. 
KD-tree: A KD-Tree is a binary tree that splits points between alternating axes. By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering
Euclidean Clustering: The euclidean clustering object takes in a distance tolerance. Any points within that distance will be grouped together. It also has min and max arguments for the number of points to represent as clusters. The idea is: if a cluster is really small, it’s probably just noise and we are not concerned with it. Also a max number of points allows us to better break up very large clusters. If a cluster is very large it might just be that many other clusters are overlapping, and a max tolerance can help us better resolve the object detections.

**4. Bounding Box**: Individual Box are assigned around each clusters(objects). 

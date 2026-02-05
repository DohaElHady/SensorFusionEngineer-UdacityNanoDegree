# Section 1: LiDAR
## Installation Guidelines on Windows
### 1. Global Environoment Setup 
- Open command Prompt with admin access
- Run: _dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart_
- Run: _dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart_
- Run: _wsl --update_
- Run: _wsl --set-version Ubuntu 2_
- Restart your PC
- Download ubunto from app store
<img width="445" height="271" alt="image" src="https://github.com/user-attachments/assets/d4b5fcbf-2c5f-40b0-8726-4967376b087d" />

- open ubunto terminal .. create user and pass
- Open WSL terminal run the following in sequance:
  
  _sudo apt update_
  
  _sudo apt upgrade_
  
  _git clone https://github.com/DohaElHady/SensorFusionEngineer-UdacityNanoDegree/_
  
  _sudo apt install cmake_
  
  _sudo apt install libpcl-dev_
  
  _cd ~/SensorFusionEngineer-UdacityNanoDegree/SFND_Lidar_Obstacle_Detection_
  
  _mkdir build && cd build_
  
  _cmake .._
  
- To fix the cmake error: add an explicit #include <boost/filesystem.hpp> to processPointClouds.h solves the issue.
- Install GUI on existing ubuntu server, in WSL terminal run the following:
  
  - Full version: _sudo apt install xubuntu-desktop_
  - or for mininmal version (recommended for limited resources): _sudo apt install --no-install-recommends ubuntu-desktop --fix-missing_
  
- In WSL terminal run the following:

  _make && ./environment_
- 3D viewer should open as shown below:
  <img width="445" height="250" alt="image" src="https://github.com/user-attachments/assets/03e8e6d2-6f63-411a-b006-6ef72cf26262" />


### 2. LiDAR RANSAC 2D
- Error: PCL requires C++14 or above
  
  update cmakelist in ransac folder as follows:
  
  **From:** add_definitions(-std=c++11)
  
  **To:** add_definitions(-std=c++14)
 
### 3. LiDAR TTC
- Error WSL not connected to network: _sudo nano /etc/resolv.conf_ >> add this line:  _nameserver 8.8.8.8_
- Install openCV: [Guide](https://github.com/Eemilp/install-opencv-on-wsl)
  - Install libdc1394-dev instead of libdc1394-22-dev
  - Install libtbbmalloc2 instead of libtbb2
  - "git checkout 4.x" instead of "git checkout 4.5.1"
  - cmake -D CMAKE_BUILD_TYPE=RELEASE -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -D OPENCV_FFMPEG_SKIP_BUILD_CHECK=ON -D OPENCV_GENERATE_PKGCONFIG=ON -DOPENCV_ENABLE_NONFREE=ON -DENABLE_PRECOMPILED_HEADERS=OFF -DOPENCV_EXTRA_MODULES_PATH=~/opencv_with_contrib/opencv_contrib/modules -D BUILD_opencv_legacy=OFF -D WITH_GTK=ON -D WITH_FFMPEG=OFF -D WITH_OPENGL=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ../opencv   
  - "make -j4" might crash in the middle, just keep repeating it until the build is 100% done, each time should continue from the craching point.
 
    <img width="858" height="286" alt="image" src="https://github.com/user-attachments/assets/297cdb95-2a2e-49b6-8789-3aefee82143f" />

## LiDAR: Run and Visualize the Scene
### 1. Control the Rendered Scene Elements
- In environment.cpp:
```cpp
// RENDER OPTIONS
bool renderScene = true;        // if renderScene is false, we will not render the highway or the cars
bool renderPC = false;          // if renderPC is false, we will not render the point cloud
bool renderPCRay = true;        // if renderPCRay is false, we will not render the rays from the lidar sensor
bool renderSegCloud = false;    // if renderSegCloud is false, we will not render the segmented point cloud
bool renderClusters = false;    // if renderClusters is false, we will not render the clustered point clouds
bool renderBbox = false;        // if renderBbox is false, we will not render the bounding boxes around the clusters
```
- In WSL terminal:

  _make && ./environment_

- The 3D viewer should only shows then the scene and the rays upon the above configurations:

  <img width="400" height="250" alt="image" src="https://github.com/user-attachments/assets/4979f59f-3c45-48ac-9463-4dc8eac4ee83" />

### 2. Control LiDAR Parameters
- In lidar.h:
a. Select the region of detection through minDistance and maxDistance 
```cpp  
// TODO:: Set minDistance to 5 (meters) to remove points from your vehicle's roof.
minDistance = 5; // minimum distance to cast rays, points closer than this will not be added to the point cloud
maxDistance = 50;
```
b. Add noise to the points so the lidar points are not exactly aligned over the circles which is more realistic
```cpp  
// TODO:: set sderr to 0.2 to get a more interesting pcd to simulate realistic lidar noise
// standard deviation error for the point cloud (in meters)
// used to add noise to the points so the lidar points are not exactly aligned over the circles which is more realistic 
sderr = 0.2;
```
c. Increase layers for higher vertical resolution
```cpp  
// TODO:: increase number of layers to 8 to get higher resolution pcd
int numLayers = 8; //number of layers in the vertical direction, more layers means more points in the vertical direction
```

at numLayers = 3

<img width="500" height="300" alt="numLayers = 3" src="https://github.com/user-attachments/assets/a7957070-1042-4c25-ab6f-7b83a581dc36" />

at numLayers = 8

<img width="500" height="300" alt="numLayers = 8" src="https://github.com/user-attachments/assets/efee1f57-2045-43f6-bd60-18bacc683ac9" />

d. Decrease angle difference for higher horizontal resolution
```cpp  
// TODO:: set to pi/64 to get higher resolution pcd
double horizontalAngleInc = pi/64; // the angle increment in the horizontal direction, smaller means more points in the horizontal direction
```

at horizontalAngleInc = pi/6

<img width="500" height="300" alt="horizontalAngleInc = pi/6" src="https://github.com/user-attachments/assets/efee1f57-2045-43f6-bd60-18bacc683ac9" />

at horizontalAngleInc = pi/64

<img width="500" height="300" alt="horizontalAngleInc = pi/64" src="https://github.com/user-attachments/assets/523032eb-c370-4e26-90d8-d33aa8e74cfa" />

### 3. Activate LiDAR Point Cloud
- Use scan function in environment.cpp to get the input cloud:
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
```

- Render the point cloud
```cpp
// renderPointCloud(): takes the viewer, the point cloud, and a name for the point cloud 
// we can have multiple point clouds in the viewer each with a unique name
renderPointCloud(viewer, inputCloud, "inputCloud");
```
<img width="1000" height="450" alt="Point Cloud" src="https://github.com/user-attachments/assets/3488e206-a613-439e-9a52-3335103d1e1c" />

### 4. Planner Segmentation using RANSAC
This method aims to find the points which can fit together creating planes in the environment; named as inliers. 

**Approach 1**
- Create a point cloud processer in environment.cpp:
```cpp
// TODO:: Create point processor
// Create a point cloud processer on the stack using the template class directly, or on the heap using a pointer to the template class
// ProcessPointClouds<pcl::PointXYZ> is a template class that takes a point type as a template parameter
ProcessPointClouds<pcl::PointXYZ> pointProcessor;
```
- Pass the scanned point cloud to it:
```cpp
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentResult = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
```
- Fill up the function _SegmentPlane_ in processPointClouds.cpp:  
  a. Create segmentation object using [SACSegmentation](https://pointclouds.org/documentation/classpcl_1_1_s_a_c_segmentation.html)"
  ```cpp
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  ```
  b. Set up model parameters:
  ```cpp
  seg.setOptimizeCoefficients (true); // get best model
  seg.setModelType (pcl::SACMODEL_PLANE); // we want to segment a plane
  seg.setMethodType (pcl::SAC_RANSAC); // we want to use RANSAC to find the plane
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (distanceThreshold);
  ```
  c. Set the input cloud and do the segmentation:
  ```cpp
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  ```
  
**Approach 2**
- Implement RANSAC manually in _RansacPlane_ function in ransac2d.cpp:
  - Create loop for RANSAC iterations
  - Pick randomely 3 points to define a plane
  - Fit plane to the 3 points
  - Measure distance between every point and fitted plane
  - If distance is smaller than threshold count it as inlier
  - Return indicies of inliers from fitted plane with most inliers

- Test _RansacPlane_ function as follows:
  ```cpp
  // You can change the max iteration and distance tolerance arguments for Ransac function from the line below in ransac2d.cpp
  if(use3D)
    inliers = RansacPlane(cloud, 100, 0.25);
  ```
  ```
  Go to src/quiz/ransac/build
  make && ./quizRansac
  ```
<img width="1000" height="450" alt="RANSAC RESULTS" src="https://github.com/user-attachments/assets/93a7a37c-7e51-4ad1-9560-48610e3b64be" />

## LiDAR TTC
- Run the following: 
  - _cd Lidar_TTC_
  - _mkdir build && cd build_
  -  _cmake .._
  -  _make_
  -  _./compute_ttc_lidar_

## Camera TTC
- Run the following: 
  - _cd Camera_TTC_
  - _mkdir build && cd build_
  -  _cmake .._
  -  _make_
  -  _./compute_ttc_camera_
 
# Section 2: Camera
## Installation Guidelines on Windows
### 1. Global Environoment Setup 
See [Section](#1-global-environoment-setup) for details.
### 2. Camera Based 2D Feature Tracking
See [Section](#3-lidar-ttc) for OpenCV installation details.

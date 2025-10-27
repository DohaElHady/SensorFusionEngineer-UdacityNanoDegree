# Installation Guidelines on Windows
## 1. Global Environoment Setup
- Open command Prompt with admin access
- Run: _dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart_
- Run: _dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart_
- Run: _wsl --set-version Ubuntu 2_
- Restart your PC
- Download ubunto from app store
<img width="445" height="271" alt="image" src="https://github.com/user-attachments/assets/d4b5fcbf-2c5f-40b0-8726-4967376b087d" />

- open ubunto terminal .. create user and pass
- Open WSL terminal run the following in sequance:
  
  _sudo apt update_
  
  _sudo apt upgrade_
  
  _git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git_
  
  _sudo apt install cmake_
  
  _sudo apt install libpcl-dev_
  
  _cd ~/SFND_Lidar_Obstacle_Detection_
  
  _mkdir build && cd build_
  
  _cmake .._
  
- To fix the cmake error: add an explicit #include <boost/filesystem.hpp> to processPointClouds.h solves the issue.
- Install GUI on existing ubuntu server, in WSL terminal run the following:
  
  _sudo apt install xubuntu-desktop_
  
- In WSL terminal run the following:

  _make && ./environment_
- 3D viewer should open as shown below:
  <img width="445" height="250" alt="image" src="https://github.com/user-attachments/assets/03e8e6d2-6f63-411a-b006-6ef72cf26262" />


## 2. LiDAR RANSAC 2D
- Error: PCL requires C++14 or above
  
  update cmakelist in ransac folder as follows:
  
  **From:** add_definitions(-std=c++11)
  
  **To:** add_definitions(-std=c++14)

# Run and Visualize the Scene
## 1. Control the Rendered Scene Elements
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

## 2. Control LiDAR Parameters
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


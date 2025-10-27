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
  <img width="445" height="250" alt="image" src="https://github.com/user-attachments/assets/c72cf29e-48a7-4781-9c0a-86315ef65b58" />

## 2. LiDAR RANSAC 2D
- Error: PCL requires C++14 or above
  
  update cmakelist in ransac folder as follows:
  
  **From:** add_definitions(-std=c++11)
  
  **To:** add_definitions(-std=c++14)


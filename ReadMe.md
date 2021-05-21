[release-image]: https://img.shields.io/badge/release-1.11.1-green.svg?style=flat
[releases]: https://github.com/PointCloudLibrary/pcl/releases
[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt
[ci-latest-build]: https://dev.azure.com/PointCloudLibrary/pcl/_build/latest?definitionId=9&branchName=master
[ci-ubuntu-18.04]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20GCC&jobName=Ubuntu&configuration=Ubuntu%2018.04%20GCC&label=Ubuntu%2018.04%20GCC
[ci-ubuntu-20.04]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Clang&jobName=Ubuntu&configuration=Ubuntu%2020.04%20Clang&label=Ubuntu%2020.04%20Clang
[ci-ubuntu-20.10]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20GCC&jobName=Ubuntu&configuration=Ubuntu%2020.10%20GCC&label=Ubuntu%2020.10%20GCC
[ci-windows-x86]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20MSVC&jobName=Windows%20VS2017%20Build&configuration=Windows%20VS2017%20Build%20x86&label=Windows%20VS2017%20x86
[ci-windows-x64]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20MSVC&jobName=Windows%20VS2017%20Build&configuration=Windows%20VS2017%20Build%20x64&label=Windows%20VS2017%20x64
[ci-macos-10.14]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Clang&jobName=macOS&configuration=macOS%20Mojave%2010.14&label=macOS%20Mojave%2010.14
[ci-macos-10.15]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Clang&jobName=macOS&configuration=macOS%20Catalina%2010.15&label=macOS%20Catalina%2010.15
[ci-docs]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/Documentation?branchName=master
[ci-latest-docs]: https://dev.azure.com/PointCloudLibrary/pcl/_build/latest?definitionId=14&branchName=master

## Self-Driving Car-LiDAR Obstacle Detection


A **self-driving car** needs some techniques for preventing collision with pedestrians, vehicles, and other objects that would exist in the way of a car. One of these tools is **LiDAR** that can be used for obstacle detection on the road. In this repository, I am going to implement a LiDAR Obstacle Detection by using C++. At the beginning of this journey, let's talk about a LiDAR. What is it and how can be used to help to determine objects?

### LiDAR

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit. **Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.
If you want to run my project own, follow the below structure on your system. If you want to run my project own, follow the below structure on your system. In this project, I have used PCL(Point Cloud Library). 

<div align="center">
<img src="https://github.com/PooyaAlamirpour/Lidar-Obstacle-Detection/blob/master/media/ObstacleDetectionFPS.gif" width="400" height="250" />
<p>The demo of the PCL visualization</p>
</div>

### Installing

If you want to run my project own, follow the below structure on your system. In this project, I have used PCL(Point Cloud Library). If you want to run my project own, follow the below structure on your system. In this project, I have used PCL(Point Cloud Library). The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing. PCL is released under the terms of the BSD license, and thus free for commercial and research use. We are financially supported by a consortium of commercial companies, with our own non-profit organization, Open Perception. We would also like to thank individual donors and contributors that have been helping the project.
This library does not have any structure for running code in windows currently. But you can build it from scratch [here](https://pcl-tutorials.readthedocs.io/en/latest/compiling_pcl_windows.html). So you have two approaches. 
> 1- Using docker file that could be downloaded from [here](https://dev.azure.com/PointCloudLibrary/pcl/_build?definitionId=11). This is the official PCL docker file.
> <br>
> 2- Installing Ubuntu on a VM.

#### Ubuntu
```
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/PooyaAlamirpour/Lidar-Obstacle-Detection.git
$> cd Lidar-Obstacle-Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```
Also you can find the continuous integration here.

Build Platform           | Status
------------------------ | ------------------------------------------------------------------------------------------------- |
Ubuntu                   | [![Status][ci-ubuntu-18.04]][ci-latest-build] <br> [![Status][ci-ubuntu-20.04]][ci-latest-build]                              <br> [![Status][ci-ubuntu-20.10]][ci-latest-build]                                                |
Windows                  | [![Status][ci-windows-x86]][ci-latest-build]  <br> [![Status][ci-windows-x64]][ci-latest-build]   |
macOS                    | [![Status][ci-macos-10.14]][ci-latest-build]  <br> [![Status][ci-macos-10.15]][ci-latest-build]   |
Documentation            | [![Status][ci-docs]][ci-latest-docs] |


### References
* Point Cloud Library [Github](https://github.com/PointCloudLibrary/pcl)
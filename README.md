English| [简体中文](./README_cn.md)

## Visual SLAM Algorithm

### Function Introduction

SLAM generally refers to Simultaneous Localization and Mapping. It is well known that the most prominent work in the field of visual SLAM is none other than ORB-SLAM3. The advantages and characteristics of ORB-SLAM3 are not further elaborated here. To facilitate developers in developing applications based on visual SLAM, TogetherROS has integrated, improved, and optimized ORB-SLAM3.
1. Integrated and adapted the SuperPoint feature extraction model to optimize the robustness of image feature extraction in visual SLAM frontend and reduce CPU runtime load.
2. Used ROS2 to encapsulate ORB-SLAM3's point cloud and pose information publishing, as well as image and IMU subscriptions.
3. Added Track asynchronous interface, separating feature extraction and feature point tracking into different threads, improving frame rate processing, which is beneficial for practical engineering applications.
4. Added a new bag-of-words library creation program to help developers build their own bag-of-words library.

In this chapter, ORB-SLAM3 is used as the mapping algorithm, with the EuRoC open dataset and RealSense D435i camera as the source of test data.

Code repository:

<https://c-gitlab.horizon.ai/HHP/box/hobot_slam/orb_slam3>


### Preparation

1. The X3 board has been flashed with the Ubuntu 20.0.4 or Linux system image provided by Horizon.

2. TogetherROS has been successfully installed on the X3 board.

3. RealSense D435i camera has been installed on the X3 board.

4. Open dataset EuRoC.

5. A PC on the same network segment as the X3 board (or connected to the same wireless network), with Ubuntu 20.0.4 system, ROS2 Foxy Desktop version, and the data visualization tool Rviz2 installed.

### Usage Guide

ORB-SLAM3 project itself integrates various types of test programs, such as monocular/stereo and monocular/stereo + IMU, and also classifies them for different evaluation datasets and sensors.
The first subsection explains how to test the SLAM program using the open dataset, and the second subsection explains how to test the SLAM program using the RealSense camera.

#### 1. Using the EuRoC Dataset
Dataset link: <http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.zip>. After downloading the dataset, enter the ORB-SLAM3 project directory. Unzip the dataset and bag-of-words locally, and run the test program. If you want to achieve a higher frame rate, you can overclock the X3 CPU, but it will also increase power consumption and temperature.
Run the following command:

```
# Set up the TogetherROS environment
source /opt/tros/setup.bash
# Overclock X3 CPU to 1.5GHz
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
# Enable X3 CPU performance mode
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
# Enter the ORB_SLAM3 project directory
cd /opt/tros/share/orb_slam3
# Unzip the dataset, V2_01_easy.zip dataset needs to be downloaded separately!
unzip V2_01_easy.zip -d V2_01_easy
# Unzip the bag of words library
tar -xvf ./Vocabulary/ORBvoc.txt.tar.gz
# Grant executable permissions to the program
sudo chmod +x ./Examples/Monocular/mono_euroc 
# Run the program, where the V2_01_easy directory is the EuRoC open source dataset directory downloaded from the internet, developers need to download it themselves!
./Examples/Monocular/mono_euroc ./ORBvoc_refine.txt ./Examples/Monocular/EuRoC.yaml ./V2_01_easy/ ./Examples/Monocular/EuRoC_TimeStamps/V201.txt
```

After running the program, it will take some time to load the bag of words library. Wait a moment, and the program will print the current frame rate.
![](./_static/_images/visual_slam/euroc_result.png)
#### 2. Use RealSense D435i Camera

TogetherROS has developed a set of sample programs based on ORB-SLAM3 and ROS2, integrating the subscription of image and IMU data and the publication of topics such as map point clouds, poses, and traveled trajectories. By using Rviz2 visualization software, developers can easily observe the program's running results, helping in ROS2 development, debugging ORB-SLAM3.
The latest version of the image is patched with the UVC and HID drivers for the RealSense series cameras on the kernel. After directly installing the RealSense SDK and ROS2 package with the apt command, you can use the test program directly.
(For instructions on installing ROS2 packages alongside TogetherROS, refer to the documentation: https://developer.horizon.ai/api/v1/fileData/TogetherROS/quick_start/install_use_ros_pkg.html#id1)
```
# Display the current ROS version. If it is empty, please source /opt/tros/setup.bash
echo $ROS_DISTRO
# Install RealSense SDK
sudo apt-get install ros-$ROS_DISTRO-librealsense2* -y
# Install RealSense ROS wrapper
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt-get install ros-$ROS_DISTRO-realsense2-description -y
```
After installation, we start the Realsense camera as an image publisher node, the visual SLAM node as the image subscriber, subscribing to the image topic, and publishing pose and point cloud information.

Next, we log in to X3 with the root account (password: root) to start the Realsense D435i camera, as insufficient permissions will prevent the camera from starting properly.
```
# Start D435i camera, publish image data
ros2 launch realsense2_camera rs_launch.py enable_depth:=false enable_color:=false enable_infra1:=true depth_module.profile:=640x480x15
```
After the camera starts, you can observe the following logs from the console:
![](./_static/_images/visual_slam/realsense.png)
Next, we start the visual SLAM node:
```
# Configure the TogetherROS environment
source /opt/tros/setup.bash
# Overclock X3 CPU to 1.5GHz
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
# Enable X3 CPU performance mode
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
# Enter the working directory
cd /opt/tros/share/orb_slam3
# Unzip the bag of words library
tar -xvf ./Vocabulary/ORBvoc.txt.tar.gz
# Start the ORB-SLAM3 monocular processing node
ros2 run orb_slam3_example_ros2 mono ./ORBvoc.txt ./Examples/Monocular/RealSense_D435i.yaml
```
Once the visual SLAM node on the X3 end receives the camera image data, it will start printing the current frame rate "fps".At the same time, open the Rviz2 visualization software on the PC side (in the same network segment as the X3 team), add relevant visualization information, and subscribe to the following topics:

![](./_static/_images/visual_slam/rviz2_1.png)

After subscribing to the topics, you can observe the rendering results of feature points in RVIZ2 software, and also observe the white map point cloud and green camera trajectory information generated on the right side of the window as the camera moves.
![](./_static/_images/visual_slam/rviz2_2.png)

### Using ORB-SLAM3 optimized based on SuperPoint

It is well known that deep learning methods have advantages and potentials that traditional algorithms cannot match, especially in terms of stability, efficiency, and accuracy in detection and classification tasks, deep learning methods have shown amazing advantages. In the field of visual SLAM, many applications have emerged using deep learning methods to replace the work of traditional SLAM front-end and back-end, demonstrating significant advantages. SuperPoint and SuperGlue are typical examples.
SuperPoint is a self-supervised deep learning network model that can simultaneously extract the positions and descriptors of image feature points.
We have integrated SuperPoint with ORB-SLAM3, and developers can freely switch the feature point extraction method to be used in the configuration file /opt/tros/share/orb_slam3/Examples/\*/*.yaml. As shown in the figure below, the feature point extraction algorithm used is "SUPERPOINT":
![](./_static/_images/visual_slam/superpoint.png)

The results of using the Superpoint feature extraction algorithm are shown in the figure below. It can be seen that the feature points are extracted very densely, detecting the contours of objects.
![](./_static/_images/visual_slam/superpoint_result.png)

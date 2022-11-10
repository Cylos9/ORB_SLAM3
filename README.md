# Installation guide by Mauhing Yip with some modified part by Cylos9
[Link to original ORB-SLAM3's README.md](https://github.com/UZ-SLAMLab/ORB_SLAM3)

[Link to original guide by Mauhing Yip](https://github.com/Mauhing/ORB_SLAM3.git)

# 1. Installation of ORB-SLAM 3 on a fresh installed Ubuntu 20.04
- Install all liberay dependencies.
```shell

sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update

sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev

sudo apt-get install libglew-dev libboost-all-dev libssl-dev

sudo apt install libeigen3-dev

```
---

### Install OpenCV 3.2.0
> [Cylos9's Note] On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is included so we will use it instead of OpenCV 3.2.0. Please move to the next step. 
---
### Install Pangolin
- Now, we install the Pangolin.
```shell
cd ~/Dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
mkdir build 
cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make -j 3 
sudo make install
```
> If you want to install to conda environment, add `CMAKE_INSTALL_PREFIX=$CONDA_PREFIX` instead.
---

### ORB-SLAM 3 
> [Cylos9's note] The below instruction have modifed from Mauhing's instruction (Using myrepo forked from the original with some changes to make it compatible with ubuntu 20.04)

```shell
cd ~/Dev
git clone https://github.com/Cylos9/ORB_SLAM3.git 
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
---

# 2. Download test datasets

```shell
cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/

```
- Similar for another datasets in EuRoc see here [https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets]


# 3. Run simulation 
```shell
cd ~/Dev/ORB_SLAM3

# Pick of them below that you want to run

# Mono
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

# Mono + Inertial
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

# Stereo
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

# Stereo + Inertial
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
```
> [Cylos9's Note] Run `sudo ldconfig ` if the below error occurs
```shell
./Examples/Monocular/mono_euroc: error while loading shared libraries: libpango_windowing.so: cannot open shared object file: No such file or directory
```

# 4. Validation Estimate vs Ground True
- We need numpy and matplotlib installed in pytho2.7. But Ubuntu20.04 has not pip2.7
```shell
sudo apt install curl
cd ~/Desktop
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
sudo python2 get-pip.py
pip2.7 install numpy matplotlib
```

**Run and plot Ground true**
```
cd ~/Dev/ORB_SLAM3

./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo
```

**Plot estimate vs Ground true**
```
cd ~/Dev/ORB_SLAM3

python evaluation/evaluate_ate_scale.py evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt f_dataset-MH01_stereo.txt --plot MH01_stereo.pdf
```

> [Cylos9's note] open the pdf `MH01_stereo.pdf` in the folder where you have run the code to see it. 

---
# 5. ROS wrapper for ORB-SLAM3 (by Thien94)

> [Cylos9's note] The below instructions is shortned with only necessary parts.

Refer to [the original repo](https://github.com/thien94/orb_slam3_ros_wrapper.git) for more details.

---
## Clone ROS wrapper

- Clone the package. Note that it should be a `catkin workspace`.
```
cd ~/catkin_ws/src/
git clone https://github.com/thien94/orb_slam3_ros_wrapper.git
```

- Open `CMakeLists.txt` and change the directory that leads to ORB-SLAM3 library at the beginning of the file (default is home folder `~/`)
```
cd ~/catkin_ws/src/orb_slam3_ros_wrapper/
gedit CMakeLists.txt
```

- Change `set(ORB_SLAM3_DIR  $ENV{HOME}/ORB_SLAM3)` 
   
   to your installation of ORB-SLAM3. `set(ORB_SLAM3_DIR $ENV{HOME}/Dev/ORB_SLAM3)`

- Build the package normally.
```
cd ~/catkin_ws/
catkin_make
```

- Next, copy the `ORBvoc.txt` file from `ORB-SLAM3/Vocabulary/` folder to the `config` folder in this package. Alternatively, you can change the `voc_file` param in the launch file to point to the right location.

- (Optional) Install `hector-trajectory-server` to visualize the trajectory.
```
sudo apt install ros-noetic-hector-trajectory-server
```

- If everything works fine, you can now try the different launch files in the `launch` folder.
---
## How to run

### EuRoC dataset:
> [Cylos9's note] Download EuRoC's bag files from [this link](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- In one terminal, launch the node:
```
roslaunch orb_slam3_ros_wrapper euroc_monoimu.launch
```
- In another terminal, playback the bag ( in the folder you stored bag files ):
```
rosbag play MH_01_easy.bag
```
Similarly for other sensor types.

# Topics
The following topics are published by each node:
- `/orb_slam3/map_points` ([`PointCloud2`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): all keypoints being tracked.
- `/orb_slam3/camera_pose` ([`PoseStamped`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)): current left camera pose in world frame, as returned by ORB-SLAM3.
- `tf`: transformation from camera frame to world frame.

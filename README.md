# UDACITY - System Integration Project
---

Programming a Real Self-Driving Car for the UDACITY Nanodegree


## The Happy Crashers Team:

|     Image              |     Name      |  LinkedIn    |     email   |
|------------------------|---------------|----------------|---------------|-------------|-------------|
| <img src="./assets/profile.jpg" alt="Camilo Gordillo" width="150" height="150"> | Camilo Gordillo | [Camilo](https://de.linkedin.com/in/camilogordillo/en) | <camigord@gmail.com> |
| <img src="./assets/profile.jpg" alt="Stefano Salati" width="150" height="150"> | Stefano Salati | [Stefano](https://www.linkedin.com/in/stefanosalati/) | <stef.salati@gmail.com> |
| <img src="./assets/profile.jpg" alt="Stefan Rademacher" width="150" height="150"> | Stefan Rademacher | [Stefan](https://www.linkedin.com/in/stefan-rademacher/) |  |
| <img src="./assets/profile.jpg" alt="Thomas GRELIER" width="150" height="150"> | Thomas GRELIER | [Thomas](https://www.linkedin.com/in/thomas-grelier/) |  |
---
## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository

```bash
   git clone https://github.com/camigord/System-Integration-Project.git
```

2. Install python dependencies
```bash
   cd System-Integration-Project
   pip install -r requirements.txt
```
3. Make and run styx
```bash
   cd ros
   catkin_make
   source devel/setup.sh
   roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
   unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
   rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
   cd System-Integration-Project/ros
   roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

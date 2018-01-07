# **Self-Driving Car: System Integration Project** 

### Introduction

For this project, I wrote ROS nodes to implement selected core functionality of the autonomous vehicle system:
* Perception module: traffic light detection node (implemented),  traffic light classification node (work-in-progress, see below)
* Planning module: waypoint updater node (implemented), the purpose of which is to derive and set target velocities per waypoint
* Control module: drive-by-wire node (implemented), which computes and sets the steering angle, throttle and brake values

The nodes communicate with each other using ROS pub/sub architecture. I tested my code using a simulator.

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![System Architecture Diagram](final-project-ros-graph-v2.png)

### Implementation details

#### Planning module
* The implementation of the waypoint updater node can be found [here](../master/ros/src/waypoint_updater/waypoint_updater.py)

#### Control module:
* The implementation of the drive-by-wire node can be found [here](../master/ros/src/twist_controller/dbw_node.py) and [here](../master/ros/src/twist_controller/twist_controller.py)

#### Perception module
* The implementation of the traffic light detection node can be found [here](../master/ros/src/tl_detector/tl_detector.py)
* A deep neural network model architecture for traffic light classification - written in Keras - can be found [here](../master/traffic_light_classifier). The model was trained on dumped simulator images. I will probably need to rewrite and retrain the model using Tensorflow, since Keras is not supported by the Carla simulation environment.
* The implementation of the traffic light classification node is currently in progress, code skeleton can be found [here](../master/ros/src/tl_detector/light_classification/tl_classifier.py). Once the final model is trained, this code will make a call to that final traffic light classification model.

### Results

The car can successfully drive around the simulation track. The car follows the waypoints closely, keeps the target speed of 10 mph and stops at a stop line when the traffic light ahead is red.

The implementation is largely complete in terms of functionality. The key extra functionality I would like to add is the integration of the Keras (or Tensorflow) DNN model to classify traffic light colors. Currently the system uses traffic light states passed from the simulator - this information is obviously not available in a real self-driving car. It should be provided by an in-built traffic light classifier.

Potentially, some fine-tuning might be required for the "edge cases" such as when a car is very close to the traffic light and it turns yellow/red. Currently the car only reacts to the red lights and ignores yellow lights.

This was a large project and I focused mainly on getting the required functionality implemented per node and integrating the nodes with one another. Therefore, the code might need some refactoring to become more concise and efficient. 

***
***
***

### ************************** ORIGINAL UDACITY REPO README BELOW **************************

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

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
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

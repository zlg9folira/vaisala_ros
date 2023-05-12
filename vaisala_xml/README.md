
## VAISALA xml package

This package implements ROS node to obtain information of Vaisala XML API (https://www.vaisala.io) and publish to specific topics. This package publishes three types of messages: full version weather information(full_weather_info.msg), UB version weather information(ub_weather_info.msg) and UB weather images(Image.msg). ROS messages are defined in vaisala_msgs package.

##Installation

`full_weather_info.msg` and `ub_weather_info.msg` are required for this package. Please view `README.md` under `vaisala_msgs` folder for installation of the msg file.

In the path `your_ros_workspace/src/vaisala_xml`, run the following line in terminal to install required python library:
```sh
pip install -r requirements
```

Put `vaisala_xml` package under Ros src. In your Ros workspace, run the following command:
`source devel/setup.bash`
`catkin_make`
Now the preset of this package is all done

## How to run

Under `vasala_xml/launch`, there are three launch file for the demo: `full_weather_info.launch`, `ub_weather_info.launch` and `full_weather_image.launch`.

Before the running, set up the launch file parameter in the launch file.

To run the demon from the lauch file, run the following line in ROS. Since there are no difference for different launch file, the following example is used for `full_weather_info.launch`.
```sh
roslaunch vaisala_node Vaisala_node.launch 
```

User can run `rostopic echo /full_weather_info` to check the published message

## Parameter explanation

`username`		mandatory	string	The username of your VAISALA request
`password`		mandatory	string	The password of your VAISALA request
`station`		mandatory	string	The station of your VAISALA request
`region`		optional	string	The region of your VAISALA request
`cam`			optional	int		The number of camara you want to look up 
`earliesttime`	optional	string	The earliest time of the time interval for your VAISALA request
`latesttime`	optional	string	The latest time of the time interval for your VAISALA request
`request_rate`	optional	int		The rate of sending request to the VAISALA web server, in unit of second
`publish_rate`	optional	int		The rate of publishing message to ROS topic, in unit of HZ


## Ros Scripts

### full_weather_info_publisher.py (vaisala_msgs/msg/full_weather_info.msg)::
This script will publish the full version weather infomation from VAISALA web server to topic `full_weather_info`
The default request rate of this publisher is 300s
The default publish rate of this publisher is 1 HZ

### ub_weather_info_publisher.py (vaisala_msgs/msg/full_weather_info.msg)::
This script will publish the UB version weather infomation from VAISALA web server to topic `UB_weather_info`
The default request rate of this publisher is 300s
The default publish rate of this publisher is 1 HZ

### image_publisher.py (sensor_msgs/msg/Image.msg)::
This script will publish the UB road condition image from VAISALA web server to topic `UB_weather_images`
The default request rate of this publisher is 300s
The default publish rate of this publisher is 10 HZ

### utility.py
This scripts contains all the utility function that used in above publisher scripts. The detailed function explantion can be found in `utility.py`

##Meta
Foad HAjiaghajani  foadhaji@buffalo.edu
Yuyang Bai yuyangba@buffalo.edu 
University at Buffalo - The State University of New York
2022

##Last update
2022





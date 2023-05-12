## VAISALA weather nodes

This package implements ROS node to obtain information from Vaisala API (https://www.vaisala.com/en) and publish information to specific topics. Corresponding ROS messages are defined in vaisala_msgs package.

## How to run
Set up your query parameter in the `/launch/Vaisala_node.launch`.

To run the node:
```sh
roslaunch vaisala_node Vaisala_node.launch 
```
This also prints the query url in the terminal. The infomation will be parsed and published to the topics.

## Parameter explanation

`longitude`		mandatory -	represents the longitude of the location where your sensing unit is installed.
`latitude`		mandatory	- represents the latitude of the location where your sensing unit is installed.
`key`			mandatory	- represents the API key.
`altitude`		optional -	represents the altitude of the location where your sensing unit is installed.
`temp_unit`		optional- choose your temprature unit. Default Celsius.	Options: C, F
`wind_unit`		optional -	choose your wind speed unit. Default m/s.	Options: MS, KTS, KMS, MPH
`time_zone`		optional -	POSIX time zone string for response timestamps.	Tiem Zone Options: UTC, Europe/Helsinki
`ftimes_length`		optional - Component of `ftime` parameter. Represents the maximum time length you want to look up.
`ftimes_timestep`	optional	- Component of `ftime` parameter. Represents the each segement length.
`ftimes_firststep`	optional	- Component of `ftime` parameter. Represents the start time of your forecast.


## Ros API

### Vaisala_msg_publisher
#### Weather information topics (vaisala_msgs/msg/Weather)::
There are different weather topics that are generated and returen in response to the query from Vaisala API. The number of topics is determined by `ftimes` parameter. Each topic name follows `weather_info` plus an interval. For example, if `ftimes` is set as 48/24h, there will be two topics generated. They will be `weather_info0` and `weather_info1`.

#### Location information (vaisala_msgs/msg/Location)::
Location informatiion will be published to topic `location_info`.

### Vaisala_msg_listener
Subscribes to the vaisala topics (used in debug mode)

## Meta
Foad Hajiaghajani 
Yuyang Bai





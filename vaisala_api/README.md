## VAISALA weather nodes

This package implements ROS node to accquires information of Vaisala Website(https://www.vaisala.com/en) and send information to the specific topic. Corresponding ROS messages are defined in Vaisala_msgs.

## How to run
Setting up your query parameter in the `/launch/Vaisala_node.launch`.

To recieve weather information from Vaisala Website and publish information, run the following line in ROS. Vaisala_msgs shoube be installed.
```sh
roslaunch vaisala_node Vaisala_node.launch 
```
This will print the query Url in the terminal. The infomation will be parsed and published to topics.

## Parameter explanation

`longitude`		mandatory	represent the longitude of a place you want to look up.
`latitude`		mandatory	represent the latitude of a place you want to look up.
`key`			mandatory	represent your API key.
`altitude`		optional	represent the altitude of a place you want to look up.
`temp_unit`		optional	choose your temprature unit. Default to Celsius.	Options: C, F
`wind_unit`		optional	choose your wind speed unit. Default to m/s.		Options: MS, KTS, KMS, MPH
`time_zone`		optional	POSIX formatted time zone string for response timestamps. Default by your current location.	Options: UTC, Europe/Helsinki
`ftimes_length`		optional	Component of `ftime` parameter. Represent the maximum time length you want to look up.
`ftimes_timestep`	optional	Component of `ftime` parameter. Represent the each segement length.
`ftimes_firststep`	optional	Component of `ftime` parameter. Represent the start time of your forecast.

For further more information of the `ftimes` explanation, please visit.


## Ros API

### Vaisala_msg_publisher
#### Weather information topics (vaisala_msgs/msg/Weather)::
There are numbers of weather information topics will be generated during pulling query from the Url. The number of topics is determined by the `ftimes` parameter. Each topic are named as `weather_info` and following its number. For example, if the `ftimes` is setted as 48/24h, there will be two topics generated. They are `weather_info0` and `weather_info1`.

The default 'weather_info' topic is used for test

#### Location information topics (vaisala_msgs/msg/Location)::
Location informatiion will be published to topic `location_info`

### Vaisala_msg_listener
Subscribe to the vaisala topics. Mainly used for test

##Meta
Foad Hajiaghajani 
Yuyang Bai





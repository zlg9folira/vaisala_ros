## Copyright
Developers:
******** Foad Hajiaghajani foadhaji@buffalo.edu	************************************
******** Yuyang Bai yuyangba@buffalo.edu *******************************************
******** University at Buffalo - The State University of New York ******************
******** 2022 **********************************************************************

## VAISALA waether msgs

This package contains two classes of ROS message definitions for recieving weather information from VAISALA API(https://www.vaisala.com/en). Messages are defined to recieve both location and weather information from VAISALA API.

## Explanation

Location.msg

tz 	- location name in details
lon 	- longitude of the searching location
lat 	- latitude of the searching location

Weather.msg

dist 	- distance to the observations station from the given coordinates appended with wind direction in English
obs_dt 	- timestamp for the observation
obs_t 	- air temperature for the observation
obs_tf 	- feels-like temperature for the observation
obs_s 	- symbol code for the observation
obs_wn 	- wind direction string for the observation
obs_ws 	- 10-minute average wind speed for the observation
obs_p	- pressure in hPa for the observation
obs_rh	- relative humidity in percentages for the observation
obs_v	- visibility in meters for the observation
fc_dt	- timestamp for the forecast
fc_s	- symbol code for the forecast
fc_tx	- 24h maximum temperature for the forecast
fc_tn	- 24h minimum temperature for the forecast
fc_pr	- accumulated precipitation for the forecast
fc_wsx	- daily maximum of the 10-minute average wind speed for the forecast






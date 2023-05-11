#! /usr/bin/env python2
import os
import rospy
import argparse
import json
from std_msgs.msg import String
from vaisala_msgs.msg import Weather
from vaisala_msgs.msg import Location
#import requests

class VaisalaNode:
	def __init__(self):
#	get default parameter from the launch file
		self.para = 0
		self.lon = rospy.get_param("/longitude")
		self.lat = rospy.get_param('/latitude')
		self.key = rospy.get_param('/key', "1111111")
		self.alt = rospy.get_param('/altitude', None)
		self.tempunit = rospy.get_param('/temp_unit', None)
		self.windunit = rospy.get_param('/wind_unit', None)
		self.tz = rospy.get_param('/time_zone', None)
  		self.ftimes_length = rospy.get_param('/ftimes_length', None)
		self.ftimes_timestep = rospy.get_param('/ftimes_timestep', None)
		self.ftimes_firststep = rospy.get_param('/ftimes_firststep', None)

#	generate the Vaisala Url
		self.url = self.geturl()

#	Set up nodes
		self.weather_pub = rospy.Publisher('weather_info', Weather, queue_size=10)
		self.location_pub = rospy.Publisher('location_info', Location, queue_size=10)
		rospy.init_node('weather_info')
		self.rate = rospy.Rate(1)

	def main(self):
		while not rospy.is_shutdown():

#			uncomment to request from the url
			#r = requests.get(url)
    			#json_object = r.url

#			reading from the sample json file, comment this while url is setted up
			with open('/home/bai/catkin_ws/src/vaisala_node/scripts/example.json', 'r') as openfile:
				json_object = json.load(openfile)

			#parse the json object
			dataset = json_object.get('loc')[0]
			observation = dataset.get('obs')
			forecast_list = dataset.get('fc')
    		
			#generating nodes based on the number of forecast
			for i in range(len(forecast_list)):
				weather_pub = rospy.Publisher('weather_info{}'.format(i), Weather, queue_size=10)
				msg = self.generate_weather_msg(observation, forecast_list[i])
				weather_pub.publish(msg)
			hello_str = "hello world %s" % rospy.get_time()
			rospy.loginfo(self.url)
			
			self.rate.sleep()

#	generate Vaisala url based on tuples
	def geturl(self):
		ftimes = self.get_ftime_tuple()
		url = "https://www.vaisala.com/en/products/data?"
		parameter = {'lon': self.lon, 'lat': self.lat, 'alt': self.alt, 'tempunit': self.tempunit, 'windunit': self.windunit, 'tz': self.tz, 'ftimes': ftimes}
		for key in parameter.keys():
			ele = parameter[key]
			if ele is not None:		
				url += key + '=' + str(ele) + '&'
		url = url[:-1]
		url += "/{}".format(self.key)
		return url

#	generate ftime tuple based on ftime length, ftime timestep and ftime first_step
	def get_ftime_tuple(self):
		ftimes = None
		if self.ftimes_length is not None and self.ftimes_timestep is not None:
		    ftimes = "{}/{}h".format(self.ftimes_length, self.ftimes_timestep)
		    if self.ftimes_firststep is not None:
		        ftimes += "/{}".format(self.ftimes_firststep)

	def generate_weather_msg(self, ob, fc):
		msg = Weather()
		msg.station = str(ob['station'])
		msg.dist = str(ob['dist'])

		msg.obs_dt = str(ob['dt'])
		msg.obs_t = int(ob['t'])
		msg.obs_tf = int(ob['tf'])
		msg.obs_s = str(ob['s'])
		msg.obs_wn = str(ob['wn'])
		msg.obs_ws = int(ob['ws'])
		msg.obs_p = float(ob['p'])
		msg.obs_rh = int(ob['rh'])
		msg.obs_v = int(ob['v'])

		msg.fc_dt = str(fc['dt'])
		msg.fc_s = str(fc['s'])
		msg.fc_tx = int(fc['tx'])
		msg.fc_tn = int(fc['tn'])
		msg.fc_pr = float(fc['pr'])
		msg.fc_wsx = str(fc['wsx'])
		msg.fc_cloudiness = int(fc['cloudiness'])
		msg.fc_sunrise = str(fc['sunrise'])
		msg.fc_sunset = str(fc['sunset'])
		msg.fc_sr = str(fc['sr'])
		return msg

		
		
		

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
	vaisala_Node = VaisalaNode()
	try:
		vaisala_Node.main()
	
	except rospy.ROSInterruptException:
		pass


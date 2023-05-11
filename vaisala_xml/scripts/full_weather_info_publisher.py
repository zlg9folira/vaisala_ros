#! /usr/bin/env python2

# ---------  full version of VAISALA weather information publisher scripts ------------
# Authors:			Foad Hajiaghajani, Yuyang Bai
# Email:			foadhaji[at]buffalo.edu
# Institution:			University at Buffalo SUNY
# Last Modified:		Mar 2022
# -------------------------------------------------------------------------------------

import rospy
import time
from vaisala_msgs.msg import Full_weather_info
import utility

class full_Vaisala_node:
	def __init__(self):
#	generate the Vaisala Url
		self.url = utility.get_url()
		_,_,_,_,_,_,_,self.request_time,self.publish_time = utility.parse_launch_file()
		if self.request_time is None:
			self.request_time = 300
		if self.publish_time is None:
			self.publish_time = 1

#	Set up weather node and information publisher
		self.info_pub = rospy.Publisher('full_weather_info', Full_weather_info, queue_size=10)
		rospy.init_node('full_weather_info')
		self.rate = rospy.Rate(self.publish_time)
		self.last_request = time.time()
		self.msg = utility.generate_full_weather_msg(self.url)

	def main(self):
		while not rospy.is_shutdown():
			current_time = time.time()
			if int(current_time - self.last_request) > self.request_time:
				self.last_request = current_time
				self.msg = utility.generate_full_weather_msg(self.url)
				
			self.info_pub.publish(self.msg)
			self.rate.sleep()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
	node = full_Vaisala_node()
	try:
		node.main()
	
	except rospy.ROSInterruptException:
		pass


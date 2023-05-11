#! /usr/bin/env python2

# ---------  UB version of VAISALA weather image publisher scripts ------------
# Author:			Foad HAjiaghajani, Yuyang Bai
# Email:			foadhaji[at]buffalo.edu
# Institution:			University at Buffalo SUNY
# Last Modified:		Mar 2022
# -----------------------------------------------------------------------------

import os
import rospy
import time
from sensor_msgs.msg import Image
import utility

class VaisalaNode:
	def __init__(self):
#	generate the Vaisala Image Url
		self.image_url = utility.get_image_url()
		_,_,_,_,_,_,_,self.request_time,self.publish_time = utility.parse_launch_file()
		if self.request_time is None:
			self.request_time = 300
		if self.publish_time is None:
			self.publish_time = 10

#	Set up nodes
		self.image_pub = rospy.Publisher('UB_weather_images', Image, queue_size=1)
		rospy.init_node('UB_weather_images')
		self.rate = rospy.Rate(self.publish_time)
		self.last_request = time.time()
		self.image = utility.get_image(self.image_url)
		
	def main(self):
		while not rospy.is_shutdown():
			current_time = time.time()
			if int(current_time - self.last_request) > self.request_time:
				self.image = Image()
				self.image = utility.get_image(self.image_url)
				self.last_request = current_time
				
			self.image.header.frame_id = "vaisala"
			self.image.header.stamp = rospy.Time.now()
			self.image_pub.publish(self.image)
			
			self.rate.sleep()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
	vaisala_Node = VaisalaNode()
	try:
		vaisala_Node.main()
	
	except rospy.ROSInterruptException:
		pass

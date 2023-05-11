#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from vaisala_msgs.msg import Weather

def callback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data))
    
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("weather_info0", Weather, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()

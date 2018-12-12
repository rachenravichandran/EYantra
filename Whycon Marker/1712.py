#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from time import time

def getPose(coordinates):
	print(time())
	for i in range(0,5):
		print('Marker '+str(i+1)+' -  '+str(coordinates.poses[i].position.x)+' '+str(coordinates.poses[i].position.y)+' '+str(coordinates.poses[i].position.z))

def whyconMarker():
	rospy.init_node('marker_coordinate', anonymous=True)
	rospy.Subscriber("/whycon/poses", PoseArray, getPose)
	rospy.spin()

if __name__ == '__main__':
	try:
		whyconMarker()
	except rospy.ROSInterruptException:
		pass


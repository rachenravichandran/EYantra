#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty
from time import time

sample = 10.00
w = 0
g = 0
KD=sample*1000
kp=11.0
KI=0.0005
KD=2.0
ki=KI*sample/1000.0
kd=KD/sample*1000
setpt=[[4.25, 4.05, 11.00],[-4.0,3.25,11.00],[-3.8,-4.2,11.00],[4.25,-4.2,11.00],[0.00,0.00,11.00]]
ierr=[0.0,0.0,0.0]
prevpt=[4.13,1.79,12.47]
position=[0.00,0.00,0.00]
ctrloutput=[0.00,0.00,0.00]
xyz=PoseArray()

def getPose(coordinates):
	global xyz
	xyz = coordinates

def pidCompute(currentpt):
	twist=Twist()
	global w,g,ierr,prevpt
	twist.angular.x=0
	twist.angular.y=0
	twist.angular.z=0
	flag=0
	for n in range(0,3):
		error = setpt[g][n] - currentpt[n]
		ierr[n] += ki*error
		derr = currentpt[n] - prevpt[n]
		ctrloutput[n] = (kp * error + ierr[n] - derr * kd)/sample
		prevpt[n] = currentpt[n]
		if ((error < 0.1 and error > -0.1) or (error < 0.8 and error > -0.3 and n == 2)):
			flag += 1
	twist.linear.x=-ctrloutput[1]  #control output for up movement
	twist.linear.y=-ctrloutput[0]  #control output for right movement
	twist.linear.z=-ctrloutput[2]
	if (flag == 3 and w < 5):		
		w += 1
		if (g <4):
			g += 1
		ierr=[0.00,0.00,0.00]
	return twist

def cruiser():
	rospy.init_node('issue_motion')
	prevtime=time()*1000
	rospy.Subscriber("/whycon/poses", PoseArray, getPose)
	control = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
	land = rospy.Publisher('/ardrone/land', Empty,queue_size=1)
	twist=Twist()
	starttime=time()
	flag=0
	bye=0
	while(1):
		takeoff.publish(Empty())
		nowtime=time()
		if (nowtime-starttime>1):
			break
	while(1):
		nowtime=time()*1000
		timechange=nowtime-prevtime
		if (timechange >= sample):
			prespose=xyz
			position[0]=prespose.poses[0].position.x
			position[1]=prespose.poses[0].position.y
			position[2]=prespose.poses[0].position.z
			twist=pidCompute(position)
			control.publish(twist)
			prevtime=nowtime
		if (w == 5 and flag < 1):
			landstart=time()
			flag += 1
		landlag=time()
		if(w == 5 and (landlag-landstart>4)):
			startime=time()
			while(1):			
				land.publish(Empty())
				nowtime=time()
				if(nowtime-startime>0.5):			
					bye=1
					break
		if (bye == 1):
			break

if __name__ == '__main__':
	try:
		cruiser()
	except rospy.ROSInterruptException:
		pass


#!/usr/bin/env python
'''
* Team Id : 1712
* Author List : Rachen R, Raghul V, Ruban Raj D, Manirathinam S
* Filename: 1712_pidcontrol.py
* Theme: Chaser Drone
* Functions: * droner() - class functions:
                        arm(), disarm(), pidCompute(int[]), access_data(Int32), getPose(PoseArray),cruiser(), main()
* Global Variables: NONE
'''
import roslib
import rospy

from plutodrone.srv import *
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from time import time
from std_msgs.msg import *


class droner():
	def __init__(self):
		rospy.init_node('key_command')
		self.control = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		rospy.Subscriber("/whycon/poses", PoseArray, self.getPose)
		rospy.Subscriber('/get_yaw', Int32, self.access_data)
		self.cmd = PlutoMsg()
		self.cmd.rcRoll =1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX1 =1500
		self.cmd.rcAUX2 =1500
		self.cmd.rcAUX3 =1500
		self.cmd.rcAUX4 =1000
		self.sample= 100  
		KI=self.sample/1000
		KD=1/self.sample*1000
		self.kp=[28.0,29.2,100.0,6.0]
		self.ki=[KI*0.120,KI*12.0,KI*0.0,KI*0.20]
		self.kd=[KD*5.20,KD*8.2,KD*0.0,KD*1.0]
		self.HOLD=[-3.0,2.5,16.0,100]
		self.ierr=[0.00,0.00,0.00,0]  #integral error
		self.prevpt=[0.00,0.00,0.00,0] #previous inputs
		self.position=[0.00,0.00,0.00,0]
		self.ctrloutput=[0.00,0.00,0.00,0] #control outputs to drone
		self.cmd = PlutoMsg() #command to drone
		self.yaw=0
		self.xyz = PoseArray()#(x,y,z) coordinates from whycon
		
'''
* Function Name: arm()
* Input: self
* Output: NONE
* Logic: arms the drone
* Example Call: self.arm()
'''
	def arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1000
		self.cmd.rcAUX1 =1500
		self.cmd.rcAUX2 =1500
		self.cmd.rcAUX3 =1500
		self.cmd.rcAUX4 =1500
		self.control.publish(self.cmd)
		rospy.sleep(.1)
		
'''
* Function Name: disarm()
* Input: self
* Output: NONE
* Logic: disarms the drone
* Example Call: self.disarm()
'''
	def disarm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 =1200
		self.control.publish(self.cmd)
		
'''
* Function Name: access_data(Int32)
* Input: self, req #req contains yaw coordinates
* Output: yaw
* Logic: gets yaw value
* Example Call: function called automatically #when drone publishes the sensor values
'''
	def access_data(self,req):
		self.yaw = req.data

'''
* Function Name: getPose(self,PoseArray)
* Input: self, coordinates 
* Output: xyz
* Logic: gets coordinates value
* Example Call: function called automatically
'''
	def getPose(self,coordinates):
		self.xyz = coordinates

'''
* Function Name: pidCOmpute(self,int[])
* Input: self, currentpt
* Output: ctrloutput[]
* Logic: computes PID output
* Example Call: pidCompute(currentpt) #function called by cruiser()
'''
	def pidCompute(self,currentpt):
		for n in range(0,4):
			error = self.HOLD[n] - currentpt[n]
			if(n==2 or n==1):
				mapper = -1 #mapping drone in frame of reference of camera
                        else:
                                mapper = 1
			self.ierr[n] += self.ki[n]*error
			derr = currentpt[n] - self.prevpt[n]	
			self.ctrloutput[n] = 1500+ (self.kp[n] * error + self.ierr[n] - derr * self.kd[n])*mapper
			self.prevpt[n] = currentpt[n]		
			if(self.ctrloutput[n]>2000):
				self.ctrloutput[n]=2000
			if(self.ctrloutput[n]<1000):
				self.ctrloutput[n]=1000
		self.cmd.rcRoll = self.ctrloutput[0]
		self.cmd.rcPitch =self.ctrloutput[1]
		self.cmd.rcThrottle =self.ctrloutput[2]
		self.cmd.rcYaw = self.ctrloutput[3]
		self.control.publish(self.cmd)

'''
* Function Name: cruiser(self)
* Input: self
* Output: NONE
* Logic: controls drone and implementation of task 3
* Example Call: cruiser() #by main
'''
	def cruiser(self):

		rospy.sleep(2)
		self.arm()
		prevtime=time()*1000
		startime=time()
		while(1):
                        if(time()-startime > 30):
                                break		
                        nowtime=time()*1000
                        timechange=nowtime-prevtime
                        if (timechange >= self.sample):
                                prespose=self.xyz
                                self.position[3]=self.yaw
                                if(self.position[3]>180):
                                        self.position[3]=self.position[3]-360
                                self.position[0]=prespose.poses[0].position.x
                                self.position[1]=prespose.poses[0].position.y
                                self.position[2]=prespose.poses[0].position.z
                                self.pidCompute(self.position)
                                prevtime=nowtime
		self.cmd.rcRoll =1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw =1500
		self.cmd.rcThrottle =1200
		self.control.publish(self.cmd)
		rospy.sleep(2)	
		self.disarm()
		rospy.sleep(1)

if __name__ == '__main__':
	try:
		test = droner()
		test.cruiser()
	except rospy.ROSInterruptException:
		pass

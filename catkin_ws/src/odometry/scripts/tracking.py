#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT
import rospy
from std_msgs.msg import Float64MultiArray
from math import pi, radians
class Tracking:
	def __init__(self):
		self.node_name = rospy.get_name()	
		self.state = 1
		self.trig = None
		self.motorhat = Adafruit_MotorHAT(addr= 0x60)
		self.leftMotor 	= self.motorhat.getMotor(1)
		self.rightMotor = self.motorhat.getMotor(2)
		self.right_pwm = 60
		self.left_pwm = 60
		self.leftMotor.setSpeed(self.left_pwm)
		self.rightMotor.setSpeed(self.right_pwm)
		self.subPosition=rospy.Subscriber("/serial_node/odometry",Float64MultiArray,self.cbPosition)

		rospy.on_shutdown(self.custom_shutdown)
		rospy.loginfo("[%s] Initialized!" %self.node_name)
	def cbPosition(self,msg):
		x     = msg.data[0]
		y     = msg.data[1]
		theta = msg.data[2]
		theta = theta % (2* pi)
		print x,y,theta

		# stages: 1) straight line,
		#         2) semi-circle
		#         3) straight line again.
		if self.state == 1:
			self.leftMotor.run(1)
			self.rightMotor.run(1)
			self.leftMotor.setSpeed(60)
			self.rightMotor.setSpeed(60)
			if x >= 1:
				self.state = 2
		elif self.state == 2:
			self.leftMotor.run(1)
			self.rightMotor.run(1)
			self.leftMotor.setSpeed(60)
			self.rightMotor.setSpeed(60)
			if theta*180/pi >= 180:
				self.state = 3
		elif self.state == 3:
			self.leftMotor.run(1)
			self.rightMotor.run(1)
			self.leftMotor.setSpeed(60)
			self.rightMotor.setSpeed(60)
			if x <= 0:
				self.state = 4
		elif self.state == 4:
			self.leftMotor.run(3)
			self.rightMotor.run(3)
			self.leftMotor.setSpeed(0)
			self.rightMotor.setSpeed(0)

	def custom_shutdown(self):
		self.leftMotor.run(4)
		self.rightMotor.run(4)
		del self.motorhat

if __name__ == '__main__':
	rospy.init_node('tracking', anonymous = False)
	Track = Tracking()
	rospy.spin()

#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
from std_msgs.msg import Bool

import time

import random
from array import *

DEBUG = False

T = 0.1

ALPHA_ID = 3
BETA_ID  = 7

RESP_NOT_REACHEBLE = 7000

class Omega:

	def __init__(self):
		# Creates a node and make sure it is a
		# unique node (using anonymous=True)
		rospy.init_node('beta_node', anonymous=True)

		# Publisher which will publish to the topic.

		self.alpha_position_subscriber = rospy.Subscriber('/alphaBot/position', UInt16, self.alphaUpdatePosition)
		self.beta_position_subscriber = rospy.Subscriber('/betaBot/position', UInt16, self.betaUpdatePosition)

		self.rate = rospy.Rate(100) # 10 ms

		self.alphaX = 0
		self.alphaY = 0
		
		self.betaX = 0
		self.betaY = 0

		rospy.Timer(rospy.Duration(T), self.timer_callback) # logs on T

	def alphaUpdatePosition(self, data):
		if DEBUG:
			print "resp: " + str(data)

		if data.data != RESP_NOT_REACHEBLE:
			self.alphaX = data.data // 18
			self.alphaY = data.data % 18

	def betaUpdatePosition(self, data):
		if DEBUG:
			print "resp: " + str(data)

		if data.data != RESP_NOT_REACHEBLE:
			self.betaX = data.data // 18
			self.betaY = data.data % 18

	def timer_callback(self, event):
		if DEBUG:
			print "Timer called at " + str(event.current_real)
		else:
			print chr(27) + "[2J"

		print "alpha (x: " + str(self.alphaX) + ", y: " + str(self.alphaY) + ")"
		print "beta (x: " + str(self.betaX) + ", y: " + str(self.betaY) + ")"


if __name__ == '__main__':
	me = Omega()
	rospy.Rate(5).sleep() # Setiing up a subscriber may take a while ...

	# me.test()
	rospy.spin()
	print "killing controller ..."

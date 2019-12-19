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

REQ_POSITION = 0
REQ_TOP      = 1
REQ_RIGHT    = 2
REQ_BOTTOM   = 3
REQ_LEFT     = 4

RESP_NOT_REACHEBLE = 7000

class Beta:

	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True)
		rospy.init_node('beta_node', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.request_publisher = rospy.Publisher('/betaBot/requestPosition', UInt8, queue_size=10)

		self.position_subscriber = rospy.Subscriber('/betaBot/position', UInt16, self.updatePosition)

		self.rate = rospy.Rate(100) # 10 ms

		# spawn robot
		self.betaX = 0
		self.betaY = 0

		rospy.Timer(rospy.Duration(T), self.timer_callback) # logs on T

	def updatePosition(self, data):
		if DEBUG:
			print "resp: " + str(data)

		if data.data != RESP_NOT_REACHEBLE:
			self.betaX = data.data // 18
			self.betaY = data.data % 18


		if not DEBUG :
			print chr(27) + "[2J"
		print "beta (x: " + str(self.betaX) + ", y: " + str(self.betaY) + ")"

	def requestMovement(self):
		msg = UInt8()

		x = int(random.getrandbits(5) % 4)
		if x == 0:
			msg.data = REQ_TOP
		if x == 1:
			msg.data = REQ_RIGHT
		if x == 2:
			msg.data = REQ_BOTTOM
		if x == 3:
			msg.data = REQ_LEFT
		if (self.betaX == 0) & (self.betaY == 0) :
			msg.data = REQ_POSITION
		if DEBUG:
			print "resq: " + str(msg.data)
		self.request_publisher.publish(msg)

	def timer_callback(self, event):
		self.requestMovement()

		if DEBUG:
			print "Timer called at " + str(event.current_real)


if __name__ == '__main__':
	me = Beta()
	rospy.Rate(5).sleep() # Setiing up a subscriber may take a while ...

	# me.test()
	rospy.spin()
	print "killing controller ..."

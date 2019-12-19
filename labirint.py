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

RESP_REACHEBLE     = 5     # not in use
RESP_NOT_REACHEBLE = 7000

class Labirint:

	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True)
		rospy.init_node('labirint_node', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.alphaPosition_publisher = rospy.Publisher('/alphaBot/position', UInt16 ) #, queue_size=16)
		self.betaPosition_publisher = rospy.Publisher('/betaBot/position', UInt16 ) #, queue_size=16)

		self.requestAlpha_subscriber = rospy.Subscriber('/alphaBot/requestPosition', UInt8, self.updateAlphaPosition)
		self.requestBeta_subscriber = rospy.Subscriber('/betaBot/requestPosition', UInt8, self.updateBetaPosition)

		self.rate = rospy.Rate(400) # 2.5 ms
		rospy.Timer(rospy.Duration(T), self.timer_callback) # logs on T

		# create map

		L0 = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
		L1 = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
		self.map = map(list, [L0, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L1, L0])
		self.generateLabirint()

		# spawn robots

		self.alphaX = 0
		self.alphaY = 0
		self.spawnAlpha()

		self.betaX = 0
		self.betaY = 0
		self.spawnBeta()

		if DEBUG :
			self.printLabirint()

	def generateLabirint(self):
		for x in range(1, 18):
			for y in range(1, 18):
				self.map[x][y] = int(random.getrandbits(1))

	def printLabirint(self):
		print chr(27) + "[2J" 
		for x in range(0, 19):
			print(self.map[x])

	def spawnAlpha(self):
		flag = True
		while flag :
			x = random.randint(1, 17)
			y = random.randint(1, 17)
			if self.map[x][y] == 0 :
				self.alphaX = x
				self.alphaY = y

				self.map[x][y] = ALPHA_ID

				flag = False
		if DEBUG :
			print ""
			print "alpha: (x: " + str(x) + ", y: " + str(y) + ")"
			print "this point is : " + str(self.map[x][y])

	def spawnBeta(self):
		flag = True
		while flag:
			x = random.randint(1, 17)
			y = random.randint(1, 17)
			if self.map[x][y] == 0 :
				self.betaX = x
				self.betaY = y

				self.map[x][y] = BETA_ID

				flag = False
		if DEBUG:
			print ""
			print "beta: (x: " + str(x) + ", y: " + str(y) + ")"
			print "this point is : " + str(self.map[x][y])

	def updateAlphaPosition(self, data):
		msg = UInt16()
		
		if DEBUG:
			print ""
			print "----"
			print "req from ALPHA with " + str(data)
			print "alpha: (x: " + str(self.alphaX) + ", y: " + str(self.alphaY) + ")"
			

		request = data.data
		if request == REQ_POSITION :
			if DEBUG:
				print "this point is  (x: " + str(self.alphaX) + " , y: " + str(self.alphaY) + ")"

			msg.data = self.alphaX * 18 + self.alphaY

		if request == REQ_TOP :
			if DEBUG:
				print "top point is  (x: " + str(self.alphaX - 1) + " , y: " + str(self.alphaY) + ")"
			
			if self.map[self.alphaX - 1][self.alphaY] == 0 :
				if DEBUG:
					print "top point is reacheble"

				self.map[self.alphaX][self.alphaY] = 0
				self.map[self.alphaX - 1][self.alphaY] = ALPHA_ID

				self.alphaX -= 1
				msg.data = self.alphaX * 18 + self.alphaY
			else:
				if DEBUG:
					print "top point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		if request == REQ_RIGHT :
			if DEBUG:
				print "right point is  (x: "+ str(self.alphaX) + " , y: " + str(self.alphaY + 1) + ")"
			
			if self.map[self.alphaX][self.alphaY + 1] == 0 :
				if DEBUG:
					print "right point is reacheble"

				self.map[self.alphaX][self.alphaY] = 0
				self.map[self.alphaX][self.alphaY + 1] = ALPHA_ID

				self.alphaY += 1
				msg.data = self.alphaX * 18 + self.alphaY
			else:
				if DEBUG:
					print "right point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		if request == REQ_BOTTOM :	
			if DEBUG:
				print "bottom point is  (x: " + str(self.alphaX + 1) + " , y: " + str(self.alphaY) + ")"
			
			if self.map[self.alphaX + 1][self.alphaY] == 0 :
				if DEBUG:
					print "bottom point is reacheble"

				self.map[self.alphaX][self.alphaY] = 0
				self.map[self.alphaX + 1][self.alphaY] = ALPHA_ID

				self.alphaX += 1
				self.alphaY 
				msg.data = self.alphaX * 18 + self.alphaY
			else:
				if DEBUG:
					print "bottom point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		if request == REQ_LEFT :
			if DEBUG:
				print "left point is  (x: " + str(self.alphaX) + " , y: " + str(self.alphaY - 1) + ")"
			
			if self.map[self.alphaX][self.alphaY - 1] == 0 :
				if DEBUG:
					print "left point is reacheble"

				self.map[self.alphaX][self.alphaY] = 0
				self.map[self.alphaX][self.alphaY - 1] = ALPHA_ID

				self.alphaY -= 1
				msg.data = self.alphaX * 18 + self.alphaY
			else:
				if DEBUG:
					print "left point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		self.alphaPosition_publisher.publish(msg)
		
		if DEBUG:
			print "RESP   " + str(msg)

	def updateBetaPosition(self, data):
		msg = UInt16()
		
		if DEBUG:
			print "req from BETA with " + str(data)
			print 

		request = data.data
		if request == REQ_POSITION :
			if DEBUG:
				print "beta, this point is (x: " + str(self.alphaX) + ", y: " + str(self.alphaY) + ")"

			msg.data = self.betaX * 18 + self.betaY


		if request == REQ_TOP :
			if DEBUG:
				print "top point is  (x: " + str(self.betaX - 1) + " , y: " + str(self.betaY) + ")"
			
			if self.map[self.betaX - 1][self.betaY] == 0 :
				if DEBUG:
					print "top point is reacheble"

				self.map[self.betaX][self.betaY] = 0
				self.map[self.betaX - 1][self.betaY] = BETA_ID

				self.betaX -= 1
				msg.data = self.betaX * 18 + self.betaY
			else:
				if DEBUG:
					print "top point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		if request == REQ_RIGHT :
			if DEBUG:
				print "right point is  (x: " + (self.betaX) + " , y: " + (self.betaY + 1) + ")"
			
			if self.map[self.betaX][self.betaY + 1] == 0 :
				if DEBUG:
					print "right point is reacheble"

				self.map[self.betaX][self.betaY] = 0
				self.map[self.betaX][self.betaY + 1] = BETA_ID

				self.betaY += 1
				msg.data = self.betaX * 18 + self.betaY
			else:
				if DEBUG:
					print "right point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		if request == REQ_BOTTOM :
			if DEBUG:
				print "bottom point is  (x: " + str(self.betaX + 1) + " , y: " + str(self.betaY) + ")"
			
			if self.map[self.betaX + 1][self.betaY] == 0 :
				if DEBUG:
					print "bottom point is reacheble"

				self.map[self.betaX][self.betaY] = 0
				self.map[self.betaX + 1][self.betaY] = BETA_ID

				self.betaX += 1
				msg.data = self.betaX * 18 + self.betaY
			else:
				if DEBUG:
					print "bottom point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		if request == REQ_LEFT :
			if DEBUG:
				print "left point is  (x: " + str(self.betaX) + " , y: " + str(self.betaY - 1) + ")"
			
			if self.map[self.betaX][self.betaY - 1] == 0 :
				if DEBUG:
					print "left point is reacheble"

				self.map[self.betaX][self.betaY] = 0
				self.map[self.betaX][self.betaY - 1] = BETA_ID

				self.betaY -= 1
				msg.data = self.betaX * 18 + self.betaY
			else:
				if DEBUG:
					print "left point is NOT reacheble"
				msg.data = RESP_NOT_REACHEBLE

		self.betaPosition_publisher.publish(msg)

		if DEBUG:
			print "RESP   " + str(msg)

	def timer_callback(self, event):
		if DEBUG:
		    print "Timer called at " + str(event.current_real)
		else:
			self.printLabirint()

if __name__ == '__main__':
	me = Labirint()
	rospy.Rate(5).sleep() # Setiing up a subscriber may take a while ...

	# me.test()
	rospy.spin()
	print "   killing controller ..."

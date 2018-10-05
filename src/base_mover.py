#!/usr/bin/env python

import rospy
from hexapods.msg import HexPos, LegPos
from hexapods.srv import MoveHex, MoveLeg
from std_msgs.msg import String



class HexMover(object):
	""" Runs the rosservices to move both an individual leg and all legs to specified positions. 
	"""
	def __init__(self):
		rospy.init_node('hexmover')
		move_leg = rospy.Service("move_leg", String, self.move_leg)
		
	def move_leg(self, legPos):
		""" Moves leg from binary input
		"""
		leg = int(legPos[0:3],2)
		motor = int(legPos[3:5],2)
		angle = int(legPos[5::],2)
		self.move_servo(self.map_leg_to_servoID(leg, motor))

	def map_leg_to_servoID(self, leg, servonum):
		# Dummy code, to match leg and servo number to whatever is used to identify which servo we write to
		return (leg * 3 + servonum)

	def move_servo(self, servo_ID, pos):
		# Move servo 
		pass
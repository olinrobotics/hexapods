#!/usr/bin/env python

import rospy
from hexapods.msg import HexPos, LegPos
from hexapods.srv import MoveHex, MoveLeg



class Step(object):
	""" Contains everything needed to generate a single stepping motion
	input is string genetics for this step: 6 bits for motor, 6 bits for angle
	motor is the int ID for the given motor
	angle is an int angle for the angle 
	"""
	def __init__(self, input_genetics):
		self.motor = int(input_genetics[0:5],2)
		self.angle = int(input_genetics[5::],2)


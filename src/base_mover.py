#!/usr/bin/env python

import rospy
from hexapods.msg import HexPos, LegPos
from hexapods.srv import MoveHex, MoveLeg



class HexMover(object):
	""" Runs the rosservices to move both an individual leg and all legs to specified positions. 
	"""
	def __init__(self):
		rospy.init_node('hexmover')
		self.publisher = rospy.Publisher('/cmd_pos', HexPos, queue_size=10)
		move_hex = rospy.Service("move_hex", MoveHex, self.move_hex)
		move_leg = rospy.Service("move_leg", MoveLeg, self.move_leg)
		
	def publish_cmd(self, leg_cmds):
		""" Just in case we want to publish instead of using a service:
			leg_cmds is nested list of [[legnumber, servo a val, servo b val, servo c val], ...]
			This places that data in the required format at publishes it to /cmd_pos
		"""
		hexPos = HexPos()

		for leg in leg_cmds:
			thisleg = LegPos()
			thisleg.leg = leg[0]
			thisleg.a = leg[1]
			thisleg.b = leg[2]
			thisleg.c = leg[3]
			hexPos.append(thisleg)

		self.publisher.publish(hexPos)

	def move_hex(self, msg):
		""" rosservice for moving all of the legs together. Takes HexPos msg, returns bool and string.
		"""
		hex_cmd = msg.msg
		for leg in hex_cmd:
			self.move_leg(leg)
		return True, ""

	def move_leg(self, legPos):
		""" rosservice for moving one specific leg. Takes LegPos msg, returns bool and string.
		"""
		self.move_servo(self.map_leg_to_servoID(legPos.leg, 0), legPos.a)
		self.move_servo(self.map_leg_to_servoID(legPos.leg, 1), legPos.b)
		self.move_servo(self.map_leg_to_servoID(legPos.leg, 2), legPos.c)
		return True, ""

	def map_leg_to_servoID(self, leg, servonum):
		# Dummy code, to match leg and servo number to whatever is used to identify which servo we write to
		return (leg.data * 3 + servonum)

	def move_servo(self, servo_ID, pos):
		# Move servo 
		pass
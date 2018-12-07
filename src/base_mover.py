#!/usr/bin/env python

import rospy
from hexapods.msg import HexPos, LegPos
from hexapods.srv import MoveHex, MoveLeg
from std_msgs.msg import String
import time



class HexMover(object):
	""" Runs the rosservices to move both an individual leg and all legs to specified positions. 
	"""
	def __init__(self):
		rospy.init_node('hexmover')
		move_leg = rospy.Service("move_leg", MoveLeg, self.move_leg)
		
	def move_leg(self, legPos):
		""" Moves leg from binary input
		"""
		motor = self.map_servo(int(legPos.msg[0:5],2))
		angle = self.map_angle(int(legPos.msg[5:13],2))
		wait = int(legPos.msg[13::],2)
		print("Moving servo %d, to position %d, then waiting %d ms" % (motor, angle, wait))
		self.move_servo(self.map_leg_to_servoID(motor), angle)
		time.sleep(wait / 1000.0)
		return True, ""

	def map_angle(self, angle):
		""" Map the angle to the specified range
		"""
		return self.map(angle, 263, 180)

	def map_servo(self, servo):
		""" Map the servonum to the specified range
		"""
		return self.map(servo, 31, 17)

	def map(self, value, inmax, outmax):
	    return int(float(value) * outmax / float(inmax))

	def map_leg_to_servoID(self, servonum):
		# Dummy code, to match servo number to whatever is used to identify which servo we write to
		return servonum

	def move_servo(self, servo_ID, pos):
		# Move servo 
		pass

	def run(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			r.sleep()

if __name__ == '__main__':
	
	hexmover = HexMover()
	hexmover.run()
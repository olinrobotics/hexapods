#!/usr/bin/env python

import rospy
from hexapods.msg import HexPos, LegPos
from hexapods.srv import MoveHex, MoveLeg
import random
import numpy as np
import time
from std_msgs.msg import String



class GenerateGenetics(object):
	def __init__(self, saveLoc, fromFile = False, fileLoc = None):
		self.step_set = []
		self.move_leg = rospy.ServiceProxy("move_leg", MoveLeg)
		if not fromFile:
			self.generate_new_set()
		else:
			self.read_from_file(fileLoc)
		self.write_to_file(saveLoc)


	def generate_new_step(self):
		""" Generate a new random string of an 11-length binary
		"""
		out = [str(i) for i in (np.random.randint(2, size=11))]
		return "".join(out)

	def generate_new_set(self):
		""" Generate a brand-new set of steps
		"""
		number_steps = 30
		for i in range(number_steps):
			self.step_set.append(self.generate_new_step())

	def mate_set(self, parent1, parent2):
		""" Mate two behavioral sets
		"""
		out1 = []
		out2 = []
		for i in range(len(parent1)):
			child1, child2 = self.mate(parent1[i], parent2[i])
			out1.append(child1)
			out2.append(child2)


	def mate(self, parent1, parent2):
		""" Mate two steps, return the two children. For example:
		self.mate("00000000000", "11111111111") = "00000111111", "11111000000"
		"""
		child1 = parent1[0:5] + parent2[6::]
		child2 = parent2[0:5] + parent1[6::]
		return child1, child2

	def mutate(self, gene):
		""" Randomly change one value in the gene. For example:
		self.mutate("00000000000") = "00000100000"
		"""
		index = random.randint(0,11)
		if(gene[index] == "0"):
			return gene[0:index] + "1" + gene[index+1:11]
		else:
			return gene[0:index] + "0" + gene[index+1:11]

	def read_from_file(self, fileLoc):
		f = open(fileLoc, 'r')
		for line in f:
			self.step_set.append(line.strip())
		f.close()

	def write_to_file(self, fileLoc):
		""" Write the current step set to the given file location
		"""
		f = open(fileLoc, 'w')
		for gene in self.step_set:
			print gene
			f.write(gene + '\n')
		f.close()

	def move(self):
		for step in self.step_set:
			self.move_leg(step)
			time.sleep(0.25)
		print("inches moved: ")
		success = int(input())

if __name__ == '__main__':
	g = GenerateGenetics("out.txt", True, "out.txt")












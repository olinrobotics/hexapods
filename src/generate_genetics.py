#!/usr/bin/env python

import rospy
from hexapods.msg import HexPos, LegPos
from hexapods.srv import MoveHex, MoveLeg
import random
import numpy as np
import time
from std_msgs.msg import String
import pickle
from step_set import StepSet
import operator


class GenerateGenetics(object):
	def __init__(self, saveLoc, fromFile = False, fileLoc = None):
		self.population_set = []
		self.population_size = 10
		self.move_leg = rospy.ServiceProxy("move_leg", MoveLeg)
		if not fromFile:
			self.generate_new_population()
		else:
			self.population_set = self.load_obj(fileLoc)

	def generate_new_population(self):
		for i in range(self.population_size):
			self.population_set.append(self.generate_new_set())

	def generate_new_step(self):
		""" Generate a new random string of an 11-length binary
		"""
		out = [str(i) for i in (np.random.randint(2, size=11))]
		return "".join(out)

	def generate_new_set(self):
		""" Generate a brand-new set of steps
		"""
		number_steps = 30
		step_set = []
		for i in range(number_steps):
			step_set.append(self.generate_new_step())
		return step_set

	def evaluate_generation(self, individual):
		""" Evaluate a generation
		"""
		for step in individual:
			self.move_leg(step)
			time.sleep(0.25)
		print("inches moved: ")
		success = int(input())
		return success

	def computePerfPopulation(self, population):
		populationPerf = []
		for individual in population:
			populationPerf.append([self.evaluate_generation(individual), individual])
		return sorted(populationPerf, reverse=True)

	def selectFromPopulation(self, populationSorted, best_sample, lucky_few):
		nextGeneration = []
		for i in range(best_sample):
			nextGeneration.append(populationSorted[i][0])
		for i in range(lucky_few):
			nextGeneration.append(random.choice(populationSorted)[0])
		random.shuffle(nextGeneration)
		return nextGeneration

	def mate_generations(self, parent1, parent2):
		""" Mate two generations
		"""
		out1 = []
		out2 = []
		for i in range(len(parent1.step_set)):
			child1, child2 = self.mate(parent1.step_set[i], parent2.step_set[i])
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

	def save_obj(self, obj, name):
		with open('obj/'+ name + '.pkl', 'wb') as f:
			pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

	def load_obj(self, name):
		with open('obj/' + name + '.pkl', 'rb') as f:
			return pickle.load(f)

	def run(self):
		populationSorted = self.computePerfPopulation(self.population_set)
		print(populationSorted)
		self.selectFromPopulation(populationSorted, 4, 1)
		




if __name__ == '__main__':
	g = GenerateGenetics("out.txt")
	g.run()












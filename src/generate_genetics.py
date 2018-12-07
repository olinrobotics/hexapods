#!/usr/bin/env python

import rospy
from hexapods.msg import HexPos, LegPos
from hexapods.srv import MoveHex, MoveLeg
import random
import numpy as np
import time
from std_msgs.msg import String
import pickle
import operator


class GenerateGenetics(object):
	def __init__(self, saveLoc, fileLoc = None):
		self.population_set = []
		self.population_size = 10
		self.move_leg = rospy.ServiceProxy("move_leg", MoveLeg)
		self.saveLoc = saveLoc
		self.mutation_liklihood = 15
		self.step_len = 21
		self.
		if fileLoc is not None:
			self.population_set = self.clean_up_input(self.load_obj(fileLoc))
			while len(self.population_set) < self.population_size:
				self.population_set.append(self.generate_new_individual())
		else:
			self.population_set = self.generate_new_population()

	def clean_up_input(self, input_list):
		""" Remove any items from the input that are smaller than they should be. 
		"""
		for item in input_list:
			if len(item) < self.step_len:
				input_list.remove(item)
		return input_list

	def generate_new_population(self):
		""" Generate a brand-new population of individuals
		"""
		new_population = []
		for i in range(self.population_size):
			new_population.append(self.generate_new_individual())
		return new_population

	def generate_new_individual(self):
		""" Generate a brand-new set of steps
		"""
		number_steps = 10
		step_set = []
		for i in range(number_steps):
			step_set.append(self.generate_new_step())
		return step_set

	def generate_new_step(self):
		""" Generate a new random string of an 19-length binary
		This describes the position of a specific servo, and the amount of time (in ms) to wait until moving the next leg.
		Done in binary to increase the variability of the effect of a mutation-
		a switched bit will sometimes have a very small effect (such as changing the position
		of a leg by a single degree), but sometimes a very large effect (changing the position greatly)
		"""
		out = [str(i) for i in (np.random.randint(2, size=self.step_len))]
		return "".join(out)

	def evaluate_individual(self, individual):
		""" Evaluate an individual 
		"""
		for step in individual:
			self.move_leg(step)
		success = random.random()  #TODO: success should be evaluated based on movement from position
		return success

	def compute_performance_population(self, population):
		""" Evauluate each individual in the population and sort them according to performance
		"""
		population_performance = []
		for individual in population:
			population_performance.append([self.evaluate_individual(individual), individual])
		return sorted(population_performance, reverse=True)

	def select_from_population(self, population_sorted, best_sample, lucky_few):
		""" Select the highest performers from a sorted population. 
		populationSorted: sorted population in the form [[score, individual]] for each individual
		best_sample: the number of highest performers to pass on to the next generation
		lucky_few: the number of lucky individuals that will pass on to the next generation
		"""
		next_generation = []
		for i in range(best_sample):
			next_generation.append(population_sorted[i][1])
		for i in range(lucky_few):
			next_generation.append(random.choice(population_sorted)[1])
		random.shuffle(next_generation)
		return next_generation

	def mate_generation(self, generation):
		""" Mate a generation with itself. For example, if we have five individuals
		in a generation, 1 and 2 will mate to form two new individuals, 3 and 4 will mate
		to form two new individuals, and if there is an odd number (such as here), the odd
		one will just pass on to the next generation. 
		"""
		out_generation = []
		for i in range(0, len(generation), 2):
			if i + 1 < len(generation):
				out_generation += self.mate_individual(generation[i], generation[i + 1])
			else:
				out_generation += [generation[i]]
		return out_generation

	def mate_individual(self, parent1, parent2):
		""" Mate two individuals, generate two children
		"""
		child1 = []
		child2 = []
		for i in range(len(parent1)):
			[step1, step2] = self.mate(parent1[i], parent2[i], self.mutation_liklihood)
			child1.append(step1)
			child2.append(step2)
		return child1, child2

	def mate(self, gene1, gene2, mutation_liklihood = 0):
		""" Mate two steps, return mated steps. For example:
		self.mate("00000000000", "11111111111") = "00000111111", "11111000000", 
		return as randomized list of two children.

		Mutation liklihood is out of 100
		"""
		if (mutation_liklihood > random.randint(0,100)):
			gene1 = self.mutate(gene1)
		halfway = int(self.step_len / 2)
		outgene1 = gene1[0:halfway] + gene2[halfway::]
		outgene2 = gene2[0:halfway] + gene1[halfway::]
		outlist = [outgene1, outgene2]
		random.shuffle(outlist)
		return outlist

	def mutate(self, gene):
		""" Randomly change one value in the gene. For example:
		self.mutate("00000000000") = "00000100000"
		"""
		index = random.randrange(0,10)
		print("mutating: " + gene)
		if(gene[index] == "0"):
			return gene[0:index] + "1" + gene[index+1::]
		else:
			return gene[0:index] + "0" + gene[index+1::]

	def save_obj(self, obj, name):
		""" Save object to given filename
		"""
		print("saving to file: " + name + ".pkl")
		with open('../obj/'+ name + '.pkl', 'wb') as f:
			pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

	def load_obj(self, name):
		""" Load object from given filename
		"""
		print("loading " + name + ".pkl")
		with open('../obj/' + name + '.pkl', 'rb') as f:
			return pickle.load(f)

	def run(self):
		population_sorted = self.compute_performance_population(self.population_set)
		best_of_generation = self.select_from_population(population_sorted, 4, 1)
		new_generation = self.mate_generation(best_of_generation)
		self.save_obj(new_generation, self.saveLoc)
		


if __name__ == '__main__':
	g = GenerateGenetics("out", "out")
	g.run()












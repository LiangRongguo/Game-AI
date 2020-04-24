'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *


# Creates the path network as a list of lines between all path nodes that are traversable by the agent.
def myBuildPathNetwork(pathnodes, world, agent=None):
	lines = []
	### YOUR CODE GOES BELOW HERE ###

	# list of obstacles
	list_of_obstacles = world.getObstacles()

	# list of all points in all obstacles
	list_of_obstacles_points = []

	# list of all lines from all obstacles
	list_of_lines = []

	# agent's radius
	radius = agent.getMaxRadius()

	for obstacle in list_of_obstacles:
		for line in obstacle.getLines():
			list_of_lines.append(line)
		for point in obstacle.getPoints():
			list_of_obstacles_points.append(point)

	# a dictionary to store the directed lines
	# key: source pathnode
	# value: list of unobstructed pathnodes connected to the key pathnode
	node_dict = {}

	# find all reachable lines from every pathnode
	for pathnode in pathnodes:
		# shallow copy for later operation
		tmp_pathnodes = copy.copy(pathnodes)
		# exclude itself from consideration for finding closest pathnode
		tmp_pathnodes.remove(pathnode)

		# use a while loop to find closest pathnodes until no nodes left or no unobstructed nodes found
		while tmp_pathnodes and findClosestUnobstructed(pathnode, tmp_pathnodes, list_of_lines):
			# get closest pathnode
			closest_unobstructed_pathnode = findClosestUnobstructed(pathnode, tmp_pathnodes, list_of_lines)

			if not node_dict.get(pathnode):
				# if not see this pathnode as source before, construct a new unobstructed nodes list for this pathnode
				node_dict[pathnode] = []

			# append current closest pathnode to the list
			node_dict[pathnode].append(closest_unobstructed_pathnode)

			# exclude this closest pathnode for later consideration and calculation
			tmp_pathnodes.remove(closest_unobstructed_pathnode)

	# filter all lines in the dictionary under the condition of radius
	for pathnode in pathnodes:
		if node_dict.get(pathnode):
			# get the target nodes for current path node
			unobstructed_pathnodes = node_dict.get(pathnode)
			for target in unobstructed_pathnodes:
				# for each target node
				flag = False
				# flag = True  -> the obstacle is inside the radius,  not include this line into final result
				# flag = False -> the obstacle is outside the radius, include this line into final result
				for obstacle_point in list_of_obstacles_points:
					# calculate minimum distance
					if minimumDistance((pathnode, target), obstacle_point) < radius:
						flag = True
						break

				if not flag:
					lines.append((pathnode, target))

			# exclude this target path node from later operation
			node_dict.pop(pathnode)

	### YOUR CODE GOES ABOVE HERE ###
	return lines


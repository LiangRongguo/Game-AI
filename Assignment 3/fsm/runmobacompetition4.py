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

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import *

from constants import *
from utils import *
from core import *
from astarnavigator2 import *
from agents import *
from moba import *
from mybuildpathnetwork import *


if len(sys.argv) < 3:
	print("Usage: python " + sys.argv[0] + " classname1 classname2")
	print("classname1 and classname2 must be in files with the same located in this directory.")
	exit(1)

module1 = __import__(sys.argv[1])
module2 = __import__(sys.argv[2])
class1 = getattr(module1, sys.argv[1])
class2 = getattr(module2, sys.argv[2])

############################
### How to use this file
###
### Use this file to conduct a competition with other agents.
### Step 1: Give your MyMinion class an unique name, e.g., MarkMinion. Change the file name to match the class name exactly.
### Step 2: python runmobacompetition.py classname1 classname2



############################
### SET UP WORLD

dims = (1200, 1200)

obstacles = [[(400, 100), (800, 100), (1100, 400), (1100, 800), (1010, 875), (990, 875), (900, 750), (900, 500), (700, 300), (450, 300), (325, 210), (325, 190)]
			 ]

rotate = list(map(lambda poly: list(map(lambda point: (dims[0]-point[1], point[0]), poly)), obstacles))

obstacles = rotate

mirror = list(map(lambda poly: list(map(lambda point: (dims[0]-point[0], dims[1]-point[1]), poly)), obstacles))

obstacles = obstacles + mirror

obstacles = obstacles + [[(550, 570), (600, 550), (660, 570), (650, 630), (600, 650), (540, 630)]]

pathnodes = list(map(lambda point: (dims[0]-point[1], point[0]), [(720, 480), (450, 670), (380, 370), (160, 160), (800, 800), (1015, 1015), 
			 (1150, 850), (1150, 40), (320, 40), (850, 1150), (50, 1150), (50, 300)])) + [(270, 50), (50, 270), (1150, 930), (930, 1150)]


###########################
### Minion Subclasses

class MyHumanMinion(class1):
	
	def __init__(self, position, orientation, world, image = NPC, speed = SPEED, viewangle = 360, hitpoints = HITPOINTS, firerate = FIRERATE, bulletclass = SmallBullet):
		class1.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)

class MyAlienMinion(class2):
	
	def __init__(self, position, orientation, world, image = JACKAL, speed = SPEED, viewangle = 360, hitpoints = HITPOINTS, firerate = FIRERATE, bulletclass = SmallBullet):
		class2.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)

########################

world = MOBAWorld(SEED, dims, dims, 2, 60)
agent = Hero((SCREEN[0]/2, SCREEN[1]/2), 0, world)
agent.team = 0
world.setPlayerAgent(agent)
world.initializeTerrain(obstacles, (0, 0, 0), 4)
agent.setNavigator(Navigator())
world.debugging = True


b1 = Base(BASE, (1125, 75), world, 1, MyHumanMinion)
world.addBase(b1)

t11 = Tower(TOWER, (950, 100), world, 1)
world.addTower(t11)
t12 = Tower(TOWER, (1100, 250), world, 1)
world.addTower(t12)

b2 = Base(BASE, (75, 1125), world, 2, MyAlienMinion)
world.addBase(b2)

t21 = Tower(TOWER, (100, 950), world, 2)
world.addTower(t21)

t22 = Tower(TOWER, (250, 1100), world, 2)
world.addTower(t22)

nav = AStarNavigator2()
nav.agent = agent
nav.setWorld(world)

nav.pathnodes = pathnodes
nav.pathnetwork = myBuildPathNetwork(pathnodes, world, agent)


b1.setNavigator(nav)
b2.setNavigator(nav)

for n in pathnodes:
	drawCross(world.debug, n)
nav.drawPathNetwork(world.debug)

world.makePotentialGates()

world.run()

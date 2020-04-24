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
from moba2 import *
from MyHero import *
from WanderingMinion import *
from clonenav import *
from RiedlHero import *
from mybuildpathnetwork import *


############################
### How to use this file
###
### Use this file to conduct a competition with other agents.
### Step 1: Give your MyHero class an unique name, e.g., MarkHero. Change the file name to match the class name exactly.
### Step 2: python runherocompetition.py classname1 classname2

############################
### SET UP WORLD

dims = (1200, 1200)

obstacles = [[(250, 150), (600, 200), (550, 350), (260, 390)],
			 [(800, 200), (1040, 140), (1050, 160), (1025, 500), (1000, 500), (810, 310)]]


mirror = list(map(lambda poly: list(map(lambda point: (dims[0]-point[0], dims[1]-point[1]), poly)), obstacles))

obstacles = obstacles + mirror

obstacles = obstacles + [[(550, 570), (600, 550), (660, 570), (650, 630), (600, 650), (540, 630)]]

pathnodes = [(160, 160), (300, 40), (1150, 40), (50, 1150), (50, 300), (720, 480), (700,100), 
			 (1090, 660), (1150, 1000), (900, 1150), (1060, 1060), (250, 520), (50, 580), (510, 780), 
			 (510, 1100), (880, 660)]


###########################
### Minion Subclasses


class WanderingHumanMinion(WanderingMinion):
	
	def __init__(self, position, orientation, world, image = NPC, speed = SPEED, viewangle = 360, hitpoints = HITPOINTS, firerate = FIRERATE, bulletclass = SmallBullet):
		WanderingMinion.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)

class WanderingAlienMinion(WanderingMinion):
	
	def __init__(self, position, orientation, world, image = JACKAL, speed = SPEED, viewangle = 360, hitpoints = HITPOINTS, firerate = FIRERATE, bulletclass = SmallBullet):
		WanderingMinion.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)


########################
### Hero Subclasses

class MyHumanHero(MyHero):
	
	def __init__(self, position, orientation, world, image = AGENT, speed = SPEED, viewangle = 360, hitpoints = HEROHITPOINTS, firerate = FIRERATE, bulletclass = BigBullet, dodgerate = DODGERATE, areaeffectrate = AREAEFFECTRATE, areaeffectdamage = AREAEFFECTDAMAGE):
		MyHero.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass, dodgerate, areaeffectrate, areaeffectdamage)



class MyAlienHero(RiedlHero):
	
	def __init__(self, position, orientation, world, image = ELITE, speed = SPEED, viewangle = 360, hitpoints = HEROHITPOINTS, firerate = FIRERATE, bulletclass = BigBullet, dodgerate = DODGERATE, areaeffectrate = AREAEFFECTRATE, areaeffectdamage = AREAEFFECTDAMAGE):
		RiedlHero.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass, dodgerate, areaeffectrate, areaeffectdamage)



########################

world = MOBAWorld(SEED, dims, dims, 0, 60)
agent = GhostAgent(ELITE, (600, 575), 0, SPEED, world)
#agent = Hero((600, 500), 0, world, ELITE)
world.setPlayerAgent(agent)
world.initializeTerrain(obstacles, (0, 0, 0), 4)
agent.setNavigator(Navigator())
agent.team = 0
world.debugging = True




b1 = Base(BASE, (75, 75), world, 1, WanderingHumanMinion, MyHumanHero, BUILDRATE, 1000000)
world.addBase(b1)

b2 = Base(BASE, (1125, 1125), world, 2, WanderingAlienMinion, MyAlienHero, BUILDRATE, 1000000)
world.addBase(b2)

nav = AStarNavigator2()
nav.agent = agent
nav.setWorld(world)

nav.pathnodes = pathnodes
nav.pathnetwork = myBuildPathNetwork(pathnodes, world, agent)


b1.setNavigator(nav)
b2.setNavigator(nav)


hero1 = MyHumanHero((125, 125), 0, world)
hero1.setNavigator(cloneAStarNavigator(nav))
hero1.team = 1
world.addNPC(hero1)

nav2 = RiedlAStarNavigator2()
nav2.agent = agent
nav2.setWorld(world)

nav2.pathnodes = nav.pathnodes
nav2.pathnetwork = nav.pathnetwork



hero2 = MyAlienHero((1050, 1025), 0, world)
hero2.setNavigator(nav2)
hero2.team = 2
world.addNPC(hero2)

for n in pathnodes:
	drawCross(world.debug, n)
nav.drawPathNetwork(world.debug)

world.makePotentialGates()

hero1.start()
hero2.start()



world.run()

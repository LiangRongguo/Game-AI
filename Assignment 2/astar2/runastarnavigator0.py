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
from nearestgatherer import *
from mybuildpathnetwork import *

def getLocation(mover):
	return mover.getLocation()
	
	
def cloneAStarNavigator(nav):
	newnav = nav.__class__()
	newnav.world = nav.world
	newnav.pathnodes = nav.pathnodes
	newnav.pathnetwork = nav.pathnetwork
	#newnav.navmesh = nav.navmesh
	return newnav


pathnodes = [(400, 600), (650, 400), (650, 200), (1075, 150), (100, 200), (100, 500), (1000, 700), (450, 800)]

nav = AStarNavigator2()

world = GatedWorld(SEED, (1224, 900), (1224, 900), 2, 60)
agent = Agent(AGENT, (612, 450), 0, SPEED, world)
world.initializeTerrain([[(628, 698), (582, 717), (549, 688), (554, 566), (676, 548)], [(942, 484), (811, 396), (843, 299), (921, 300)], [(457, 422), (381, 490), (300, 515), (310, 400), (454, 350)]])
world.setPlayerAgent(agent)

nav.pathnodes = pathnodes
nav.pathnetwork = myBuildPathNetwork(pathnodes, world, agent)

nav.setWorld(world)
agent.setNavigator(nav)
world.initializeResources([(350, 550), (750, 600), (700, 750), (1050, 300), (900, 150), (250, 300)], RESOURCE)
world.debugging = True

g = NearestGatherer(NPC, (50, 50), 0.0, SPEED, world)
nav2 = cloneAStarNavigator(nav)
g.setNavigator(nav2)
g.setTargets(list(map(getLocation, list(world.resources))))
world.addNPC(g)

for n in pathnodes:
	drawCross(world.debug, n)
nav.drawPathNetwork(world.debug)

g.start()

world.makePotentialGates()
world.drawPotentialGates()
world.run()

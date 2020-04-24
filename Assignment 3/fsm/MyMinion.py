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
from moba import *


class MyMinion(Minion):
	def __init__(self, position, orientation, world, image = NPC, speed = SPEED, viewangle = 360, hitpoints = HITPOINTS, firerate = FIRERATE, bulletclass = SmallBullet):
		Minion.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)

		### Add your states to self.states (but don't remove Idle)
		### YOUR CODE GOES BELOW HERE ###
		self.states = [Idle, MoveToAndAttackTower, Attack, MoveToAndAttackTowerBase]
		### YOUR CODE GOES ABOVE HERE ###

	def start(self):
		Minion.start(self)
		self.changeState(Idle)


############################
### Idle
###
### This is the default state of MyMinion. The main purpose of the Idle state is to figure out what state to change to and do that immediately.
class Idle(State):
	def enter(self, old_state):
		State.enter(self, old_state)
		self.agent.stopMoving()

	def execute(self, delta=0):
		State.execute(self, delta)
		### YOUR CODE GOES BELOW HERE ###
		my_team = self.agent.getTeam()
		enemy_towers = self.agent.world.getEnemyTowers(myteam=my_team)
		enemy_bases = self.agent.world.getEnemyBases(myteam=my_team)

		if enemy_towers:
			self.agent.changeState(MoveToAndAttackTower, enemy_towers)
		elif enemy_bases:
			enemy_base = enemy_bases[0]
			self.agent.changeState(MoveToAndAttackTowerBase, enemy_base)

		### YOUR CODE GOES ABOVE HERE ###
		return None
##############################
### Taunt
###
### This is a state given as an example of how to pass arbitrary parameters into a State.
### To taunt someome, Agent.changeState(Taunt, enemyagent)


class Taunt(State):
	def parseArgs(self, args):
		self.victim = args[0]

	def execute(self, delta=0):
		if self.victim is not None:
			print("Hey " + str(self.victim) + ", I don't like you!")
		self.agent.changeState(Idle)

##############################
### YOUR STATES GO HERE:


class MoveToAndAttackTower(State):
	def parseArgs(self, args):
		self.targets = args[0]

	def enter(self, old_state):
		State.enter(self, old_state)
		closest_index = 0
		if len(self.targets) > 1:
			dist1 = distance(self.agent.getLocation(), self.targets[0].getLocation())
			dist2 = distance(self.agent.getLocation(), self.targets[1].getLocation())
			if dist2 > dist1:
				closest_index = 1
		self.agent.navigateTo(self.targets[closest_index].getLocation())
		self.target_index = closest_index

	def execute(self, delta=0):
		if not self.targets[self.target_index].alive:
			self.agent.changeState(Idle)

		closest_index = 0
		if len(self.targets) > 1:
			dist1 = distance(self.agent.getLocation(), self.targets[0].getLocation())
			dist2 = distance(self.agent.getLocation(), self.targets[1].getLocation())
			if dist2 < dist1:
				closest_index = 1
		self.target_index = closest_index

		if distance(self.agent.getLocation(), self.targets[self.target_index].getLocation()) < SMALLBULLETRANGE:
			self.agent.stopMoving()
			self.agent.turnToFace(self.targets[self.target_index].getLocation())
			self.agent.shoot()
		else:
			for npc in self.agent.getVisible():
				if npc in self.agent.world.getEnemyNPCs(self.agent.getTeam()) and npc.isAlive() and distance(self.agent.getLocation(), npc.getLocation()) < SMALLBULLETRANGE:
					self.agent.changeState(Attack, npc)

	def exit(self):
		self.agent.stopMoving()


class Attack(State):
	def parseArgs(self, args):
		self.enemy = args[0]

	def execute(self, delta=0):
		if self.enemy.alive and distance(self.enemy.getLocation(), self.agent.getLocation()) < SMALLBULLETRANGE:
			self.agent.turnToFace(self.enemy.getLocation())
			self.agent.shoot()
		else:
			self.agent.changeState(Idle)


class MoveToAndAttackTowerBase(State):
	def parseArgs(self, args):
		self.target = args[0]

	def enter(self, oldstate):
		State.enter(self, oldstate)
		self.agent.navigateTo(self.target.getLocation())

	def execute(self, delta=0):
		if not self.target.alive:
			self.agent.changeState(Idle)

		if distance(self.agent.getLocation(), self.target.getLocation()) < SMALLBULLETRANGE:
			self.agent.stopMoving()
			self.agent.turnToFace(self.target.getLocation())
			self.agent.shoot()
		else:
			for npc in self.agent.getVisible():
				if npc in self.agent.world.getEnemyNPCs(self.agent.getTeam()) and npc.isAlive() and distance(self.agent.getLocation(), npc.getLocation()) < SMALLBULLETRANGE:
					self.agent.changeState(Attack, npc)

	def exit(self):
		self.agent.stopMoving()

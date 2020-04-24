import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *
from astarnavigator2 import *
from planner import *
from npcworld import *
from mybuildpathnetwork import *

############################

goal_state = State(propositions=['has_food', 'has_money'])
init_state = State(propositions=["gun_for_sale", "ammo_for_sale", 'bunny_alive', 'money_in_bank', 'dirty', 'at_home'])

buy_gun = TriggerAction('buy_gun', preconditions=["gun_for_sale", 'at_store'],
    add_list=["has_gun"], delete_list=["gun_for_sale"])

buy_ammo = TriggerAction('buy_ammo', preconditions=['ammo_for_sale', 'at_store'],
    add_list=['has_ammo'], delete_list=['ammo_for_sale'])

load_gun = TriggerAction('load_gun', preconditions=['has_gun', 'has_ammo'],
    add_list=['gun_loaded'], delete_list=['has_ammo'])

shoot_bunny = TriggerAction('shoot_bunny', preconditions=['has_gun', 'gun_loaded', 'at_forest'],
    add_list=['has_food'], delete_list=['bunny_alive', 'gun_loaded'])

rob_bank = TriggerAction('rob_bank', preconditions=['has_gun', 'gun_loaded', 'money_in_bank', 'at_bank'],
    add_list=['has_money'], delete_list=['money_in_bank'])

take_bath = TriggerAction('take_bath', preconditions=['at_home', 'dirty'],
    add_list=['wet'], delete_list=['dirty'])

dry_off = TriggerAction('dry_off', preconditions=['wet', 'at_home'],
    add_list=[], delete_list=['wet'])

go_home_bank = MoveAction('go_home_bank', preconditions=['at_home'],
    add_list=['at_bank'], delete_list=['at_home'], cost=10)

go_home_forest = MoveAction('go_home_forest', preconditions=['at_home'],
    add_list=['at_forest'], delete_list=['at_home'], cost=10)

go_home_store = MoveAction('go_home_store', preconditions=['at_home'],
    add_list=['at_store'], delete_list=['at_home'], cost=10)

go_store_bank = MoveAction('go_store_bank', preconditions=['at_store'],
    add_list=['at_bank'], delete_list=['at_store'], cost=10)

go_store_forest = MoveAction('go_store_forest', preconditions=['at_store'],
    add_list=['at_forest'], delete_list=['at_store'], cost=10)

go_store_home = MoveAction('go_store_home', preconditions=['at_store'],
    add_list=['at_home'], delete_list=['at_store'], cost=10)

go_bank_forest = MoveAction('go_bank_forest', preconditions=['at_bank'],
    add_list=['at_forest'], delete_list=['at_bank'], cost=10)

go_bank_store = MoveAction('go_bank_store', preconditions=['at_bank'],
    add_list=['at_store'], delete_list=['at_bank'], cost=10)

go_bank_home = MoveAction('go_bank_home', preconditions=['at_bank'],
    add_list=['at_home'], delete_list=['at_bank'], cost=10)

go_forest_bank = MoveAction('go_forest_bank', preconditions=['at_forest'],
    add_list=['at_bank'], delete_list=['at_forest'], cost=10)

go_forest_store = MoveAction('go_forest_store', preconditions=['at_forest'],
    add_list=['at_store'], delete_list=['at_forest'], cost=10)

go_forest_home = MoveAction('go_forest_home', preconditions=['at_forest'],
    add_list=['at_home'], delete_list=['at_forest'], cost=10)

actions = [buy_gun, buy_ammo, load_gun, shoot_bunny, rob_bank, take_bath, dry_off, 
           go_home_store, #go_home_bank, go_home_forest,
           go_store_bank, go_store_forest, #go_store_home,
           go_bank_forest, # go_bank_store, go_bank_home, 
           go_forest_bank] #, go_forest_home, go_forest_store]


#############################
### Get the Game World up and running

pathnodes = [(400, 600), (650, 400), (650, 200), (1075, 150), (100, 200), (100, 500), (1000, 700), (450, 800)]


nav = AStarNavigator2()

world = NPCWorld(SEED, WORLD, SCREEN, init_state.propositions)

home = Place("at_home", (475, 350), 100, 100, world, color=(0, 0, 255), linewidth=4)
home.possible_triggers = ['take_bath', 'dry_off']
store = Place("at_store", (650, 400), 100, 100, world, color=(255, 0, 255), linewidth=4)
store.possible_triggers = ['buy_gun', 'buy_ammo', 'load_gun']
bank = Place("at_bank", (800, 600), 100, 100, world, color=(255, 0, 0), linewidth=4)
bank.possible_triggers = ['rob_bank', 'load_gun']
forest = Place("at_forest", (800, 100), 100, 100, world, color=(0, 255, 0), linewidth=4)
forest.possible_triggers = ['shoot_bunny', 'load_gun']
world.add_place(home)
world.add_place(store)
world.add_place(bank)
world.add_place(forest)

agent = NPCAgent(AGENT, (SCREEN[0]/2, SCREEN[1]/2), 0, SPEED, world)
agent.initial_state = init_state
agent.goal_state = goal_state
agent.actions = actions

world.initializeTerrain([[(628, 698), (582, 717), (549, 688), (554, 546), (676, 548)], [(942, 484), (811, 396), (843, 299), (921, 300)], [(457, 422), (371, 506), (300, 515), (300, 400), (454, 350)]], (0, 0, 0), 4, TREE) 
world.setPlayerAgent(agent)

nav.pathnodes = pathnodes
nav.pathnetwork = myBuildPathNetwork(pathnodes, world, agent)

nav.setWorld(world)
agent.setNavigator(nav)

for n in pathnodes:
  drawCross(world.debug, n)
nav.drawPathNetwork(world.debug)

agent.start()
print("THE PLAN")
if agent.the_plan is not None:
	for act in agent.the_plan:
		print(act.name)

world.debugging = True
world.run()
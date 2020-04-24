from planner import *

#############################

init_state = State(['at_bed', 'blue_key_at_bed', 'red_key_at_table', 'gold_at_chest', 
                    'red_door_locked', 'blue_door_locked',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
goal_state = State(['has_gold'])

pickup_blue_key = Action('pickup_blue_key', 
                         preconditions = ['at_bed', 'blue_key_at_bed'], 
                         add_list = ['has_blue_key'], 
                         delete_list = ['blue_key_at_bed'])
open_blue_door = Action('open_blue_door', 
                         preconditions = ['at_blue_door', 'has_blue_key', 'blue_door_locked'], 
                         add_list = ['path_blue_door_to_table'], 
                         delete_list = ['blue_door_locked'])
pickup_red_key = Action('pickup_red_key', 
                         preconditions = ['at_table', 'red_key_at_table'], 
                         add_list = ['has_red_key'], 
                         delete_list = ['red_key_at_table'])
open_red_door = Action('open_red_door', 
                         preconditions = ['at_red_door', 'has_red_key', 'red_door_locked'], 
                         add_list = ['path_red_door_to_chest'], 
                         delete_list = ['red_door_locked'])
pickup_gold = Action('pickup_gold', 
                         preconditions = ['at_chest', 'gold_at_chest'], 
                         add_list = ['has_gold'], 
                         delete_list = ['gold_at_chest'])
move_bed_to_blue_door = Action('move_bed_to_blue_door', 
                         preconditions = ['at_bed', 'path_bed_to_blue_door'], 
                         add_list = ['at_blue_door'], 
                         delete_list = ['at_bed'])
move_blue_door_to_table = Action('move_blue_door_to_table', 
                         preconditions = ['at_blue_door', 'path_blue_door_to_table'], 
                         add_list = ['at_table'], 
                         delete_list = ['at_blue_door'])
move_table_to_red_door = Action('move_table_to_red_door', 
                         preconditions = ['at_table', 'path_table_to_red_door'], 
                         add_list = ['at_red_door'], 
                         delete_list = ['at_table'])
move_red_door_to_chest = Action('move_red_door_to_chest', 
                         preconditions = ['at_red_door', 'path_red_door_to_chest'], 
                         add_list = ['at_chest'], 
                         delete_list = ['at_red_door'])

actions = [pickup_blue_key, open_blue_door, pickup_red_key, open_red_door, pickup_gold, 
           move_bed_to_blue_door, move_blue_door_to_table, move_table_to_red_door, move_red_door_to_chest]

############################
### Test the heuristic

state10 = init_state
state9 = State(['at_bed', 'has_blue_key', 'red_key_at_table', 'gold_at_chest', 
                    'red_door_locked', 'blue_door_locked',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state8 = State(['at_blue_door', 'has_blue_key', 'red_key_at_table', 'gold_at_chest', 
                    'red_door_locked', 'blue_door_locked',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state7 = State(['at_blue_door', 'has_blue_key', 'red_key_at_table', 'gold_at_chest', 
                    'red_door_locked', 'path_blue_door_to_table', 
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state6 = State(['at_table', 'has_blue_key', 'red_key_at_table', 'gold_at_chest', 
                    'red_door_locked', 'path_blue_door_to_table', 
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state5 = State(['at_table', 'has_blue_key', 'has_red_key', 'gold_at_chest', 
                    'red_door_locked', 'path_blue_door_to_table',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state4 = State(['at_red_door', 'has_blue_key', 'has_red_key', 'gold_at_chest', 
                    'red_door_locked', 'path_blue_door_to_table',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state3 = State(['at_red_door', 'has_blue_key', 'has_red_key', 'gold_at_chest', 
                    'path_red_door_to_chest', 'path_blue_door_to_table',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state2 = State(['at_chest', 'has_blue_key', 'has_red_key', 'gold_at_chest', 
                    'path_red_door_to_chest', 'path_blue_door_to_table',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])
state1 = State(['at_chest', 'has_blue_key', 'has_red_key', 'has_gold', 
                    'path_red_door_to_chest', 'path_blue_door_to_table',
                    'path_bed_to_blue_door', 'path_table_to_red_door'])

tests = [state1, state2, state3, state4, state5, state6, state7, state8, state9, state10]

print("TESTING compute_heuristic")


the_planner = Planner()
for n, state in enumerate(tests):
	h = the_planner.compute_heuristic(state, goal_state, actions)
	print("TEST", n, "h=", h)

#############################
### Run planner

print("TESTING astar")

for n, state in enumerate(tests):
     print("TEST", n)
     the_planner = Planner()
     the_planner.initial_state = state
     the_planner.goal_state = goal_state
     the_planner.actions = actions

     plan, closed = the_planner.astar(state, goal_state, actions)
     for act in plan:
          print(act.name)
     print("states visited", len(closed))


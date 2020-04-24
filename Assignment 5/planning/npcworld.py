from constants import *
from utils import *
from core import *

import pdb
import copy
from functools import reduce

from planner import *

############################################################
### NPCAgent
### 
### Agent that knows how to create and execute plans


class NPCAgent(Agent, Planner):

  def __init__(self, image, position, orientation, speed, world, hitpoints = HITPOINTS, firerate = FIRERATE, bulletclass = Bullet):
    Agent.__init__(self, image, position, orientation, speed, world, hitpoints, firerate, bulletclass)
    Planner.__init__(self)
    self.world = world       # pointer to world object

  ### Called every tick
  def update(self, delta):
    Agent.update(self, delta)
    Planner.update(self, delta)

  ### When the agent starts, it should have an initial_state and goal_state (State objects),
  ### And runs the planner to generate a plan
  def start(self):
    Agent.start(self)
    Planner.start(self)
    if self.initial_state is not None and self.goal_state is not None and len(self.actions) > 0:
      ### Run the planner
      self.the_plan, _ = self.astar(self.initial_state, self.goal_state, self.actions)

  ### Stop the agents
  def stop(self):
    Agent.stop(self)
    Planner.stop(self)
    



##############################################################
### Place
###
### A place is a rectangle in the game world that can be moved to and in which certain actions
### can be executed. The label is "at_x" and can be looked up to get the x,y coordinates for navigation.
###
### Possible_triggers are the names of actions (strings) that can be executed here if the agent wants.

class Place(Thing):

  def __init__(self, label, position, width, height, world, color = (0, 0, 0), linewidth = 1):
    self.world = world                 # pointer to world object
    self.position = position           # upper left corner
    self.width = width                 # width
    self.height = height               # height
    self.label = label                 # the label with which the agent can find the object
    self.actions_performed_here = []   # record of actions that have been performed in this place (for debugging)
    self.color = color                 # color
    self.linewidth = linewidth         # line thickness
    self.possible_triggers = []        # The names of actions that can be performed legally here.
    self.center = (position[0]+width/2, position[1]+height/2)    # center point
    # Draw the rectangle
    s = pygame.Surface((width*2, height*2), pygame.SRCALPHA, 32)
    s = s.convert_alpha()
    pygame.draw.rect(s, color, (0,0,width,height), linewidth)
    self.surface = s
    self.font = pygame.font.SysFont("monospace", 15)

  # Draw the place
  def draw(self, parent):
    if self.surface != None:
      # The rectangle
      parent.blit(self.surface, self.position)
      # The name (label)
      label = self.font.render(self.label, True, self.color)
      parent.blit(label, self.position)
      # Draw the actions that have been performed here (for debugging)
      for n, a in enumerate(self.actions_performed_here):
        l = self.font.render(a, True, self.color)
        loc = (self.position[0], self.position[1]+(self.height/2)+(n*20))
        parent.blit(l, loc)
    return None

  ### Called when an action is triggered here
  def trigger(self, action):
    pass

  ### Is the agent currently in the place?
  def agent_here(self):
    return (self.world.agent.getLocation()[0] > self.position[0]) and (self.world.agent.getLocation()[0] < self.position[0] + self.width) and (self.world.agent.getLocation()[1] > self.position[1]) and (self.world.agent.getLocation()[1] < self.position[1] + self.height)

  ### Called every tick. Update the world state based on whether the agent is here or not.
  ### When the agent is here, add the label to the world state
  ### When the agent is not here, remove the label from the world state
  def update(self, delta = 0):
    if self.agent_here():
      self.world.add_proposition_to_world_state(self.label)
    else:
      self.world.remove_proposition_from_world_state(self.label)



############################################################
### DoorPlace

class DoorPlace(Place):

  def __init__(self, label, position, width, height, world, p1, p2, sprite = GATE, color = (0, 0, 0), linewidth = 1):
    Place.__init__(self, label, position, width, height, world, color, linewidth)
    self.door = Gate(p1, p2, sprite, world)
    world.gates.append(self.door)

  ### Called when an action is triggered here
  def trigger(self, action):
    if self.door in self.world.gates and isinstance(action, DoorAction):
      self.world.gates.remove(self.door)

#############################################################

### Threading callback
def threaded_draw_with_places():
  global game_world
  self = game_world
  while True:
    offsetX = self.camera[0] - self.agent.rect.center[0]
    offsetY = self.camera[1] - self.agent.rect.center[1]
    self.screen.fill((255, 255, 255))
    self.screen.blit(self.background, [offsetX, offsetY])
    yield
    if self.debugging:
      self.background.blit(self.debug, (0, 0))
    yield
    self.sprites.draw(self.background)
    yield
    for o in self.obstacles:
      o.draw(self.background)
    yield
    for g in self.gates:
      g.draw(self.background)
    yield
    self.drawMousePosition()
    self.drawPlaces()
    yield
    pygame.display.flip()
    yield


#############################################################
### NPCWorld
###
### A special type of world that knows about places and has a current world state
### (a set of string propositions)

class NPCWorld(GatedWorld):

  def __init__(self, seed, worlddimensions, screendimensions, initial_state):
    GatedWorld.__init__(self, seed, worlddimensions, screendimensions, 0, 0)
    self.world_state = initial_state # The world state (set of strings)
    self.places = []                 # list of places

  ### Add a proposition (string) to world state
  def add_proposition_to_world_state(self, prop):
    self.world_state.add(prop)

  ### Remove a proposition (string) from world_state
  def remove_proposition_from_world_state(self, prop):
    if prop in self.world_state:
      self.world_state.remove(prop)

  ### Add a place
  def add_place(self, place):
    self.places.append(place)

  ### Overrides the run loop from GameWorld. Uses threading.
  def run(self):
    global game_world
    self.sprites = pygame.sprite.RenderPlain((self.agent))
    for m in self.movers:
      self.sprites.add(m)
    clock = pygame.time.Clock()
    
    # Draw obstacles. 
    for o in self.obstacles:
      o.draw(self.background)

    game_world = self
    draw_iterator = threaded_draw_with_places()
    update_iterator = threaded_update()

    while True:
      clock.tick(TICK)
      delta = clock.get_rawtime()
      self.handleEvents()
      self.update(delta)
      self.sprites.update(delta) 

      try:
        next(draw_iterator)
      except StopIteration:
        pass

  ### Called every tick. Updates all other objects in the world, including places.
  def update(self, delta = 0):
    GameWorld.update(self, delta)
    for p in self.places:
      p.update(delta)

  ### Draw places
  def drawPlaces(self):
    for p in self.places:
      p.draw(self.background)

  ### Draw world
  def drawWorld(self):
    GameWorld.drawWorld(self)
    drawPlaces()

  ### Are a set of preconditions from an action satisfied by game world state?
  def check_preconditions(self, preconds):
    for precond in preconds:
      if precond not in self.world_state:
        return False
    return True

  ### Look up the place for a label
  def get_x_y_for_label(self, label):
    for place in self.places:
      if label == place.label:
        return place.center
    return None

  ### Trigger an action
  def trigger(self, action):
    for place in self.places:
      # Is the agent in a place where it is legal to trigger this action?
      if place.agent_here() and action.name in place.possible_triggers:
        place.trigger(action)
        # Record the fact that it happened
        place.actions_performed_here.append(action.name)
        # Update the world state
        self.world_state = self.world_state.difference(action.delete_list).union(action.add_list)
        print('world state:', self.world_state)
        return True
    return False





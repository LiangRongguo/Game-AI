from constants import *
from utils import *
from core import *
from functools import reduce


########################### 
### gensym

GENSYMBOL = "g"
GENCOUNT = 0

def gensym():
    global GENCOUNT
    GENCOUNT = GENCOUNT + 1
    return str(GENSYMBOL) + str(GENCOUNT)

##########################
### Action Class
### 
### This class is for actions that the planner will create plans out of.
### They are based on STRIPS, where actions have preconditions (list of strings)
### and effects are broken in to add_list and delete_list.
### Actions can also have cost.
###
### Note: it is unclear whether a plan can have more than one instance of an action.
### I think so, but it is untested.
###
### Actions should be instantiated in advance of running the planner.


class Action(object):

  def __init__(self, name, preconditions, add_list, delete_list, cost = 1):
    self.name = name   # String name of action
    self.agent = None  # Pointer back to agent to control
    self.cost = cost   # Integer cost
    self.preconditions = set(preconditions) # Set of strings where each string is an unique symbol
    self.add_list = set(add_list)           # Set of strings where each string is an unique symbol
    self.delete_list = set(delete_list)     # Set of strings where each string is an unique symbol
    self.first = True  # First time execute is being called?

  ### Reset the action. Do this if the action is going to be used again.
  def reset(self):
    self.first = True

  ### Execute the action
  def execute(self, delta = 0):
    ret = True             # default return value
    # If this is the first time, call enter()
    if self.first:
      ret = self.enter()
      self.first = False
    return ret

  ### Do this the first time execute is being called.
  def enter(self):
    if self.agent is not None:
      # Check preconditions against the game engine (via agent)
      return self.agent.check_preconditions(self.preconditions)
    else:
      return False

######################
### MoveAction Class
###
### Subclass of Action that knows how to invoke the agent's navigator

class MoveAction(Action):

  def __init__(self, name, preconditions, add_list, delete_list, cost = 1):
    Action.__init__(self, name, preconditions, add_list, delete_list, cost)
    self.target = None   # Where is the agent going?

  ### Reset the action
  def reset(self):
    Action.reset(self)
    self.target = None

  ### Body of execution loop. 
  ### Just wait until the agent get's to the target destination
  def execute(self, delta = 0):
    if Action.execute(self, delta):
      if self.target is None:
        # Fail
        return False
      elif distance(self.agent.getLocation(), self.target) < self.agent.getRadius():
        # Execution succeeds
        return True
      else:
        # still executing
        return None
    return False

  ### First time executing, invoke the agent's navigator
  def enter(self):
    if Action.enter(self):
      at_prop = None              # one of the effects must be "at" something (horrible hack)
      # Find the "at" effect
      for prop in self.add_list:
        if prop[0:2] == 'at':
          at_prop = prop
      if at_prop is not None:
        # Get the x, y for the proposition from the game world
        self.target = self.agent.get_x_y_for_label(at_prop)
        if self.target is not None:
          # Start navigating
          self.agent.navigateTo(self.target)
          return True
    return False

#########################
### TriggerAction
###
### Performs discrete, instantaneous actions

class TriggerAction(Action):

  ### Just call trigger on the world (via agent)
  def execute(self, delta = 0):
    if Action.execute(self, delta):
      if self.agent is not None:
        return self.agent.trigger(self)
    return False


##########################
### DoorAction
###
### Opens Doors

class DoorAction(TriggerAction):
  pass


##########################
### State
###
### A state of the world. Holds a set of strings as propositions that will match against preconditions
### and effects of Actions.
### Also holds g and h values, and computes f values.
### Keeps track of it's parent state and the action that can be used to transition from the
### parent to this state. 

class State(object):

  def __init__(self, propositions):
    self.id = gensym() # Randomly generated identifier
    self.propositions = set(propositions) # The propositions
    self.g = 0         # cost from the initial state
    self.h = 0         # approximate cost to the goal
    self.parent = None # parent state
    self.causing_action = None # the name of the action that caused this state.

  ### Return g-value
  def get_g(self):
    return self.g

  ### Return h-value
  def get_h(self):
    return self.h

  ### Calculate f-value
  def get_f(self):
    return self.g + self.h

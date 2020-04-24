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
import operator
from pygame.locals import *

from constants import *
from core import *
from moba2 import *
from behaviortree import *
import pdb

import inspect

HEROHITPOINTS = 50
BUILDRATE = 180
TOWERFIRERATE = 15
BASEFIRERATE = 20
BULLETRANGE = 150
SMALLBULLETRANGE = 150
BIGBULLETRANGE = 250
TOWERBULLETRANGE = 250
TOWERBULLETDAMAGE = 10
TOWERBULLETSPEED = (20, 20)
TOWERBULLET = "sprites/bullet2.gif"
BASEBULLETRANGE = 200
BASEBULLETDAMAGE = 10
BASEBULLETSPEED = (20, 20)
BASEBULLET = "sprites/bullet2.gif"
SPAWNNUM = 3
MAXSPAWN = 4
AREAEFFECTDAMAGE = 25
AREAEFFECTRATE = 60
AREAEFFECTRANGE = 2
SAFETHRESHOLD = 30
BASEHITPOINTS2 = 1000000

########################
### PYGAME STUFF

def load_image(name, colorkey=None):
  image = pygame.image.load(name)
  image = image.convert()
  if colorkey is not None:
    if colorkey is -1:
      colorkey = image.get_at((0,0))
    image.set_colorkey(colorkey, RLEACCEL)
  return image, image.get_rect()


############################
### OTHER STUFF

### Distance between two points
def ff89(p1, p2):
	return (((p2[0]-p1[0])**2) + ((p2[1]-p1[1])**2))**0.5
  
# Calc the gradient 'm' of a line between p1 and p2
def ff88(p1, p2):
  
	# Ensure that the line is not vertical
	if (p1[0] != p2[0]):
		m = (p1[1] - p2[1]) / float(p1[0] - p2[0])
		return m
	else:
		return None
 
# Calc the point 'b' where line crosses the Y axis
def ff87(p, m):
	return  p[1] - (m * p[0])
 
# Calc the point where two infinitely long lines (p1 to p2 and p3 to p4) intersect.
# Handle parallel lines and vertical lines (the later has infinite 'm').
# Returns a point tuple of points like this ((x,y),...) or None
# In non parallel cases the tuple will contain just one point.
# For parallel lines that lay on top of one another the tuple will contain
# all four points of the two lines
def ff86(p1, p2, p3, p4):
	m1 = ff88(p1, p2)
	m2 = ff88(p3, p4)
      
	# See if the the lines are parallel
	if (m1 != m2):
		# Not parallel
      
		# See if either line is vertical
		if (m1 is not None and m2 is not None):
			# Neither line vertical           
			b1 = ff87(p1, m1)
			b2 = ff87(p3, m2)  
			x = (b2 - b1) / float(m1 - m2)       
			y = (m1 * x) + b1           
		else:
			# Line 1 is vertical so use line 2's values
			if (m1 is None):
				b2 = ff87(p3, m2)   
				x = p1[0]
				y = (m2 * x) + b2
			# Line 2 is vertical so use line 1's values               
			elif (m2 is None):
				b1 = ff87(p1, m1)
				x = p3[0]
				y = (m1 * x) + b1           
			else:
				assert false
              
		return ((x,y),)
	else:
		# Parallel lines with same 'b' value must be the same line so they intersect
		# everywhere. In this case we return the start and end points of both lines
		# the ff85 method will sort out which of these points
		# lays on both line segments
		b1, b2 = None, None # vertical lines have no b value
		if m1 is not None:
			b1 = ff87(p1, m1)
          
		if m2 is not None:   
			b2 = ff87(p3, m2)
      
		# If these parallel lines lay on one another   
		if b1 == b2:
			return p1,p2,p3,p4
		else:
			return None  
  
  
 
# For line segments (i.e., not infinitely long lines) the intersect point
# may not lay on both lines.
#   
# If the point where two lines intersect is inside both lines' bounding
# rectangles then the lines intersect. Returns intersect point if the line
# intersects or None if not
def ff85(p1, p2, p3, p4):
	p = ff86(p1, p2, p3, p4)
	if p is not None:
		p = p[0]
		if ff84(p[0], p1[0], p2[0]) and ff84(p[1], p1[1], p2[1]) and ff84(p[0], p3[0], p4[0]) and ff84(p[1], p3[1], p4[1]):
			return p
	return None

# Checks if the first number is between the other two numbers.
# Also returns true if all numbers are very close together to the point where they are essentially equal
# (i.e., floating point approximation).
def ff84(p, p1, p2):
	return p + EPSILON >= min(p1, p2) and p - EPSILON <= max(p1, p2)

# Checks if two numbers are very close to the same value (i.e., floating point approximation).
def ff83(n1, n2):
	return abs(n1 - n2) < EPSILON

# Checks if two points are very close to the same value (i.e., floating point approximation).
def ff82(p1, p2):
	return ff83(p1[0], p2[0]) and ff83(p1[1], p2[1])


def ff81(p1, p2, line):
	return ff85(line[0], line[1], p1, p2)
	#pygame.draw.line(background, (0, 0, 0), p1, p2)
	
def ff80(p1, p2, worldLines):
	for l in worldLines:
		hit = ff81(p1, p2, l)
		if hit != None:
			return hit
	return None

# Check whether the line between p1 and p2 intersects with line anywhere except an endpoint.
def ff79(p1, p2, line):
	# They are the same line: bad
	if (p1 == line[0] and p2 == line[1]) or (p2 == line[0] and p1 == line[1]):
		return p1
	# They are not the same line but share an endpoint: good
	if (p1 == line[0] or p2 == line[1]) or (p2 == line[0] or p1 == line[1]):
		return None
	# They do not share any points
	hitpoint = ff85(line[0], line[1], p1, p2)
	if hitpoint != None:
		return hitpoint
	return None

# Check whether the line between p1 and p2 intersects any line anywhere except an endpoint of any of the lines.
def ff78(p1, p2, worldLines):
	for l in worldLines:
		hit = ff79(p1, p2, l)
		if hit != None:
			return hit
	return None


# Return minimum distance between line segment and point
def ff77(line, point):
	d2 = ff89(line[1], line[0])**2.0
	if d2 == 0.0: 
		return ff89(point, line[0])
	# Consider the line extending the segment, parameterized as line[0] + t (line[1] - line[0]).
	# We find projection of point p onto the line. 
	# It falls where t = [(point-line[0]) . (line[1]-line[0])] / |line[1]-line[0]|^2
	p1 = (point[0] - line[0][0], point[1] - line[0][1])
	p2 = (line[1][0] - line[0][0], line[1][1] - line[0][1])
	t = ff68(p1, p2) / d2  # numpy.dot(p1, p2) / d2
	if t < 0.0: 
		return ff89(point, line[0])	# Beyond the line[0] end of the segment
	elif t > 1.0: 
		return ff89(point, line[1])	# Beyond the line[1] end of the segment
	p3 = (line[0][0] + (t * (line[1][0] - line[0][0])), line[0][1] + (t * (line[1][1] - line[0][1]))) # projection falls on the segment
	return ff89(point, p3)


#Polygon is a set of points
def ff76(point, polygon):
	last = None
	threshold = EPSILON
	for p in polygon:
		if last != None and ff77((last, p), point) < threshold:
			return True
		last = p
	return ff77((polygon[0], polygon[len(polygon) - 1]), point) < threshold

def ff75(p1, p2, range):
	return ff89(p1, p2) <= range

def ff74(point, range, list):
	for pt in list:
		if ff75(point, pt, range):
			return True
	return False

def ff73(poly, screen, color = (0, 0, 0), width = 1, center = False):
	last = None
	for p in poly:
		if last != None:
			pygame.draw.line(screen, color, last, p, width)
		last = p
	pygame.draw.line(screen, color, poly[0], poly[len(poly)-1], width)
	if center:
		c = ( sum(map(lambda p: p[0], poly))/float(len(poly)), sum(map(lambda p: p[1], poly))/float(len(poly)) )
		pygame.draw.line(screen, color, (c[0]-2, c[1]-2), (c[0]+2, c[1]+2), 1)
		pygame.draw.line(screen, color, (c[0]+2, c[1]-2), (c[0]-2, c[1]+2), 1)

def ff72(poly1, poly2):
	#if two triangles share 2 points, they are adjacent
	points = []
	for p1 in poly1:
		for p2 in poly2:
			if p1 == p2:
				points.append(p1)
	return points

def ff71(poly1, poly2):
	points = ff72(poly1, poly2)
	if len(points) >= 2:
		isAdjacent = False
		for i, point in enumerate(points[:-1]):
			nextPoint = points[i + 1]
			point1Index = poly1.index(point)
			if poly1[(point1Index + 1) % len(poly1)] == nextPoint or poly1[point1Index - 1] == nextPoint:
				point2Index = poly2.index(point)
				if poly2[(point2Index + 1) % len(poly2)] == nextPoint or poly2[point2Index - 1] == nextPoint:
					isAdjacent = True
					break
		if isAdjacent:
			return points
	return False
		
def ff70(points):
	p1 = None
	p2 = None
	negpos = 0
	for p3 in points:
		if p1 != None and p2 != None:
			#cross product must always be the same sign
			zcross = ff69(p1, p2, p3)
			if negpos == 0:
				if zcross >= 0:
					negpos = 1
				else:
					negpos = -1
			elif negpos >= 0 and zcross < 0:
				return False
			elif negpos < 0 and zcross > 0:
				return False
		p1 = p2
		p2 = p3
	#Do the last check
	zcross = ff69(points[len(points)-2], points[len(points)-1], points[0])
	if negpos >= 0 and zcross < 0:
		return False
	elif negpos < 0 and zcross > 0:
		return False
	zcross = ff69(points[len(points)-1], points[0], points[1])
	if negpos >= 0 and zcross < 0:
		return False
	elif negpos < 0 and zcross > 0:
		return False
	else:
		return True
	
def ff69(p1, p2, p3):
	dx1 = p2[0] - p1[0]
	dy1 = p2[1] - p1[1]
	dx2 = p3[0] - p2[0]
	dy2 = p3[1] - p2[1]
	return (dx1*dy2) - (dy1*dx2)
	
def ff68(p1, p2):
	return (p1[0]*p2[0]) + (p1[1]*p2[1])


#Special routine for appending a line to a list of lines, making sure there are no duplicates added. Changes made by side-effect.
def ff67(line, lines):
	if (line in lines) == False and (ff66(line) in lines) == False:
		lines.append(line)
		return False
	else:
		return True
	
#Reverse the order of points in a line.	
def ff66(line):
	return (line[1], line[0])
	
#Determine whether a point is inside an simple polygon. Polygon is a set of lines.
def ff65(point, polygon):
	count = 0
	intersectEndPoints = {}
	for l in polygon:
		outsidePoint = (-10, SCREEN[1]/2.0)
		result = ff81(point, outsidePoint, l)
		if result != None:
			if ff82(result, point):
				return True

			# Handles an edge case where the testing line touches the same endpoint of two lines.
			matchingPoint = None
			if ff82(result, l[0]):
				matchingPoint = (l[0], l[1])
			elif ff82(result, l[1]):
				matchingPoint = (l[1], l[0])
			if matchingPoint is not None:
				if matchingPoint[0] in intersectEndPoints:
					# Check whether the point is tangent or intersecting the polygon at this matching point
					# by checking the intersection of the line segment formed by the other endpoints of the two polygon line segments.
					if ff85(point, outsidePoint, intersectEndPoints[matchingPoint[0]], matchingPoint[1]) is not None:
						continue
				else:
					intersectEndPoints[matchingPoint[0]] = matchingPoint[1]
			count = count + 1
	return count%2 == 1

#Determine whether a point is inside an simple polygon. Polygon is a set of points.
def ff64(point, polygon):
	lines = []
	last = None
	for p in polygon:
		if last != None:
			lines.append((last, p))
		last = p
	lines.append((polygon[len(polygon)-1], polygon[0]))
	return ff65(point, lines)

# Angle between two lines originating at (0, 0). Length of lines must be greater than 0.
def ff63(pt1, pt2):
	x1, y1 = pt1
	x2, y2 = pt2
	inner_product = x1*x2 + y1*y2
	len1 = math.hypot(x1, y1)
	len2 = math.hypot(x2, y2)
	return math.acos(inner_product/(len1*len2))

def ff62(v):
	return reduce(lambda x, y: (x**2)+(y**2), v)**0.5
	
# Find the point in nodes closest to p that is unobstructed
# NOTE: there is a problem in that this function doesn't compute whether there is enough clearance for an agent to get to the nearest unobstructed point
def ff61(p, nodes, worldLines):
	best = None
	dist = INFINITY
	for n in nodes:
		if ff80(p, n, worldLines) == None:
			d = ff89(p, n)
			if best == None or d < dist:
				best = n
				dist = d
	return best
	
def ff60(surface, point, color = (0, 0, 0), size = 2, width = 1):
	pygame.draw.line(surface, color, (point[0]-size, point[1]-size), (point[0]+size, point[1]+size), width)
	pygame.draw.line(surface, color, (point[0]+size, point[1]-size), (point[0]-size, point[1]+size), width)




def ff59(poly1, poly2):
	overlaps = []
	#Lines of poly1
	lines1 = []
	last = None
	for p in poly1:
		if last != None:
			lines1.append((last, p))
		last = p
	lines1.append((poly1[0], poly1[len(poly1)-1]))
	#lines of poly2
	lines2 = []
	last = None
	for p in poly2:
		if last != None:
			lines2.append((last, p))
		last = p
	lines2.append((poly2[0], poly2[len(poly2)-1]))
	#Check to see if any lines from poly1 are in poly2
	for l in lines1:
		center = ( sum(map(lambda p: p[0], l))/2.0, sum(map(lambda p: p[1], l))/2.0 )
		if ff65(center, lines2):
			overlaps.append(l[0])
			overlaps.append(l[1])
	if len(overlaps) > 0:
		return overlaps
	else:
		return False



#Return the the points that are link to the given point by some line
def ff58(point, lines):
	result = set()
	for l in lines:
		if (l[0] == point):
			result.add(l[1])
		elif(l[1] == point):
			result.add(l[0])
	return list(result)







class RiedlAStarNavigator2(PathNetworkNavigator):

				
	### Finds the shortest path from the source to the destination using A*.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., its current location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		self.setPath(None)
		### Make sure the next and dist matrices exist
		if self.agent != None and self.world != None: 
			self.source = source
			self.destination = dest
			### Step 1: If the agent has a clear path from the source to dest, then go straight there.
			### Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
			### Tell the agent to move to dest
			if ff56(source, dest, self.world.getLinesWithoutBorders(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				### Step 2: If there is an obstacle, create the path that will move around the obstacles.
				### Find the path nodes closest to source and destination.
				start = ff55(source, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				end = ff55(dest, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				if start != None and end != None:
					### Remove edges from the path network that intersect gates
					newnetwork = ff57(self.pathnetwork, self.world.getGates(), self.world)
					closedlist = []
					### Create the path by traversing the pathnode network until the path node closest to the destination is reached
					path, closedlist = ff53(start, end, newnetwork)
					if path is not None and len(path) > 0:
						### Determine whether shortcuts are available
						path = ff48(source, dest, path, self.world, self.agent)
						### Store the path by calling self.setPath()
						self.setPath(path)
						if self.path is not None and len(self.path) > 0:
							### Tell the agent to move to the first node in the path (and pop the first node off the path)
							first = self.path.pop(0)
							self.agent.moveToTarget(first)
		return None
		
	### Called when the agent gets to a node in the path.
	### self: the navigator object
	def checkpoint(self):
		ff50(self)
		return None

	### This function gets called by the agent to figure out if some shortcuts can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return ff47(self)

	def update(self, delta):
		ff51(self, delta)


### Removes any edge in the path network that intersects a worldLine (which should include gates).
def ff57(network, worldLines, world):
	newnetwork = []
	for l in network:
		hit = ff80(l[0], l[1], worldLines)
		if hit == None:
			newnetwork.append(l)
	return newnetwork



### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def ff56(p1, p2, worldLines, worldPoints, agent):
    ### YOUR CODE GOES BELOW HERE ###
    threshold = agent.getMaxRadius()
    collide = ff80(p1, p2, worldLines)
    if collide is None:
        tooclose = False
        for p in worldPoints:
            if ff77((p1, p2), p) < threshold:
                tooclose = True
        if not tooclose:
            return True
    ### YOUR CODE GOES ABOVE HERE ###
    return False

### Given a location, find the closest pathnode that the agent can get to without collision
### agent: the agent
### location: the location to check from (typically where the agent is starting from or where the agent wants to go to) as an (x, y) point
### pathnodes: a list of pathnodes, where each pathnode is an (x, y) point
### world: pointer to the world
def ff55(location, pathnodes, worldLines, agent):
	node = None
	### YOUR CODE GOES BELOW HERE ###
	node = ff61(location, pathnodes, worldLines)
	### YOUR CODE GOES ABOVE HERE ###
	return node

#PART OF SOLUTION
def ff54(x, list, func = lambda x: x):
	for i in range(len(list)):
		if func(x) < func(list[i]):
			list.insert(i, x)
			return list
	list.append(x)
	return list

### Implement the a-star algorithm
### Given:
### Init: a pathnode (x, y) that is part of the pathnode network
### goal: a pathnode (x, y) that is part of the pathnode network
### network: the pathnode network
### Return two values: 
### 1. the path, which is a list of states that are connected in the path network
### 2. the closed list, the list of pathnodes visited during the search process
def ff53(init, goal, network):
	path = []
	open = []
	closed = []
	### YOUR CODE GOES BELOW HERE ###
	### Node: (state, g, h, parent)
	init = (init, 0, ff89(init, goal), None)
	closed = set()
	nodes = set()
	open = [init]
	current = init
	
	# search
	while current is not None and current[0] != goal and len(open) > 0:
		closed.add(current[0])
		nodes.add(current)
		open.pop(0)
		suc = ff52(current, network, goal)
		for s in suc:
			if s[0] not in closed:
				ff54(s, open, lambda x:x[1]+x[2])
		if len(open) > 0:
			current = open[0]
		else:
			current = None

	# Reconstruct path
	if current is not None:
		while current[3] is not None:
			path.append(current[0])
			parent = current[3]
			for n in list(nodes):
				if parent == n[0]:
					current = n
					break
		path.append(current[0])
		path.reverse()
	closed = list(closed)
	### YOUR CODE GOES ABOVE HERE ###
	return path, closed


def ff52(node, network, goal):
	states = []
	for l in network:
		if l[0] == node[0]:
			states.append((l[1], node[1]+ff89(l[0], l[1]), ff89(l[1], goal), node[0]))
		elif l[1] == node[0]:
			states.append((l[0], node[1]+ff89(l[0], l[1]), ff89(l[0], goal), node[0]))
	return states



def ff51(nav, delta):
	### YOUR CODE GOES BELOW HERE ###
	if nav.getPath() is not None:
		gates = nav.world.getGates()
		last = nav.agent.getLocation()
		for p in nav.getPath() + [nav.getDestination()]:
			if last is not None:
				hit = ff80(last, p, gates)
				if hit is not None:
					#nav.computePath(nav.agent.getLocation(), nav.getDestination())
					nav.setPath(None)
					nav.agent.stopMoving()
					return None
			last = p
		### YOUR CODE GOES ABOVE HERE ###
		return None




def ff50(nav):
	### YOUR CODE GOES BELOW HERE ###
	'''
	last = None
	worldLines = nav.world.getLines()
	for p in nav.path:
		if last is not None:
			hit = ff80(last, p, worldLines)
			if hit is not None:
			# Plan is no longer valid
			#nav.computePath(nav.agent.getLocation(), nav.destination)
			nav.setPath(None)
			nav.agent.stopMoving()
			return None
		last = p
	'''
	### YOUR CODE GOES ABOVE HERE ###
	return None





### PART OF SOLUTION
def ff49(nav):
	if nav.path != None and nav.agent.moveTarget != nav.destination:
		hit = ff80(nav.agent.rect.center, nav.destination, nav.world.getLines())
		if hit == None:
			tooclose = False
			for p in nav.world.getPoints():
				if ff77((nav.agent.rect.center, nav.destination), p) < nav.agent.getRadius()*2.0:
					tooclose = True
			if not tooclose:
				return True
	return False

### This function optimizes the given path and returns a new path
### source: the current position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the A* algorithm
### world: pointer to the world
def ff48(source, dest, path, world, agent):
	path = copy.deepcopy(path)
	### YOUR CODE GOES BELOW HERE ###
	alllines = world.getLines()
	newstart = None
	newend = None
	for p in path:
		fronthit = ff80(source, p, alllines)
		if fronthit == None:
			tooclose = False
			for p1 in world.getPoints():
				if ff77((source, p), p1) < world.agent.getRadius()*2.0:
					tooclose = True
			if not tooclose:
				newstart = p
		if newend == None:
			backhit = ff80(p, dest, alllines)
			if backhit == None:
				tooclose = False
				for p1 in world.getPoints():
					if ff77((dest, p), p1) < world.agent.getRadius()*2.0:
						tooclose = True
				if not tooclose:
					newend = p
	newpath = []
	start = False
	end = False
	for p in path:
		if end == False:
			if start == False:
				if p == newstart:
					newpath.append(p)
					start = True
			else:
				newpath.append(p)
			if p == newend:
				newpath.append(p)
				end = True
	path = newpath
	### YOUR CODE GOES BELOW HERE ###
	return path


### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def ff47(nav):
	### YOUR CODE GOES BELOW HERE ###
	if nav.path != None and nav.agent.moveTarget != nav.destination:
		if ff49(nav):
			nav.path = []
			nav.agent.moveToTarget(nav.destination)
			return True
		elif ff46(nav):
			next = nav.path.pop(0)
			nav.agent.moveToTarget(next)
			return True
	### YOUR CODE GOES ABOVE HERE ###
	return False

# PART OF SOLUTION
def ff46(nav):
	if nav.path != None and len(nav.path) > 0:
		next = nav.path[0]
		hit = ff80(nav.agent.rect.center, next, nav.world.getLines())
		if hit == None:
			tooclose = False
			for p in nav.world.getPoints():
				if ff77((nav.agent.rect.center, next), p) < nav.agent.getRadius()*2.0:
					tooclose = True
			if tooclose:
				return False
			else:
				return True
	return False







###########################	
### ff43

ff44 = "g"
ff45 = 0

def ff43():
	global ff45
	ff45 = ff45 + 1
	return str(ff44) + str(ff45)

###########################
### BTNode
###
### Each update, the execute at the root of the tree will be called, and it recursively figures out which leaf should execute

class ff28(object):
	
	### children: children BTNodes
	### id: an id number. Randomly assigned unless set.
	### current: the current child (index integer)
	### agent: the executing agent
	### first: is this the first time it is executed?
	
	def __init__(self, agent, args = []):
		self.id = ff43()
		self.agent = agent
		self.children = []
		self.current = None
		self.first = True
		self.ff29(args)
	
	def ff29(self, args):
		if len(args) > 0:
			self.id = args

	### Add a child to the BTNode, ff33 the current counter
	def addChild(self, child):
		self.children.append(child)
		if self.current == None:
			self.current = 0
		
	### Perform a behavior, should be called every tick
	### Returns True if the behavior succeeds, False if the behavior fails, or None if the behavior should continue to execute during the next tick.
	def execute(self, delta = 0):
		if self.first:
			self.ff42()
			self.first = False
		return True
			
	def ff42(self):
		return None
	
	### Print each node id in tree in a depth-first fashion
	def ff32(self):
		print(self.id)
		for child in self.children:
			child.ff32()

	### ff33 the tree for another run. For BTNode, this means moving the current child counter back to 0.
	def reset(self):
		self.current = 0
		self.first = True
		for child in self.children:
			child.reset()

	def ff34(self, id):
		self.id = id

	def ff35(self):
		return self.id

	def ff36(self):
		return self.agent

	def getChild(self, index):
		return self.children[index]

	def ff38(self):
		return self.children

	def ff39(self):
		return len(self.children)

	def ff40(self):
		return self.current

	def ff41(self, index):
		self.current = index
		
##########################
### Sequence
###
### A sequence node tries to execute every child in order, one after another. It fails if any child fails. It succeeds when all children have succeeded.


class ff27(ff28):

	### execute() is called every tick. It recursively tries to execute the currently indexed child.
	### If a child fails, the sequence fails and returns False.
	### If a child succeeds, the sequence goes on to the next child on the next tick. If the sequence gets to the end of the list, with all children succeeding, the sequence succeeds.
	### If a child requires several ticks to complete execution, then the child will return None. If a child returns None, the sequence also returns None.
	### If a sequence node has no children, it succeeds automatically.
	def execute(self, delta = 0):
		ff28.execute(self, delta)
		### YOUR CODE GOES BELOW HERE ###
		if len(self.children) > 0:
			# I have children
			if self.current >= len(self.children):
				# I have reached the end of my children, I am done
				return True
			else:
				cur = self.children[self.current]
				res = cur.execute(delta)
				if res == False:
					# One of my sequence failed, I also fail
					return False
				elif res == True:
					# one of my sequence succeeded, go on to the next
					self.current = self.current + 1
					if self.current >= len(self.children):
						# I reached the end, I succeed
						return True
					else:
						# keep going
						return None
				else:
					# My child returned none, so keep going
					return None
		# assert: I have no children
		### YOUR CODE GOES ABOVE HERE ###
		return True

###########################
### Selector
###
### A selector node tries each child in order until one succeeds or they all fail. If any child succeeds, the selector node also succeeds and stops trying children. If all children fail, then the selector node also fails.
		
class ff26(ff28):

	### execute() is called every tick. It recursively tries to execute the currently indexed child.
	### If the child succeeds, the selector node succeeds and returns True.
	### If the child fails, the selector goes on to the next child in the next tick. If the selector gets to the end of the list and all children have failed, the selector fails and returns False.
	### If a child requires several ticks to complete execution, then the child will return None. If a child returns None, the selector also returns None.
	### If a selector node has no children, it fails.
	def execute(self, delta = 0):
		ff28.execute(self, delta)
		### YOUR CODE GOES BELOW HERE ###
		if len(self.children) > 0:
			# I have children
			if self.current >= len(self.children):
				# I have reached the end of my children
				return False
			else:
				cur = self.children[self.current]
				res = cur.execute(delta)
				if res == True:
					# My child succeeded, so I am done
					return True
				elif res == False:
					# My child failed, go on to the next child
					self.current = self.current + 1
					if self.current >= len(self.children):
						# I have run out of children
						return False
					else:
						# I have more children, keep going
						return None
				else:
					# My child returned none, keep going
					return None
		# assert: I have no children
		### YOUR CODE GOES ABOVE HERE ###
		return False





###########################
### SET UP BEHAVIOR TREE


def ff25(agent):
	myid = str(agent.getTeam())
	spec = None
	### YOUR CODE GOES BELOW HERE ###
	spec = [(ff26, myid+".ff261"), (ff13, 0.5, myid+".ff13"), [(ff15, 0.5, myid+".hp.daemon"), [(ff26, myid+".ff262"), [(ff14, 2, myid+".str.daemon"), [(ff27, myid+".hero.ff27"), (ff10, myid+".chase.hero"), (ff9, myid+".kill.hero")]], [(ff27, myid+".minion.ff27"), (ff12, myid+".chase.minion"), (ff11, myid+".kill.minion")]]]]
	### YOUR CODE GOES ABOVE HERE ###
	return spec

def ff24(agent):
	myid = str(agent.getTeam())
	root = None
	### YOUR CODE GOES BELOW HERE ###
	
	### YOUR CODE GOES ABOVE HERE ###
	return root

### Helper function for making BTNodes (and sub-classes of BTNodes).
### type: class type (BTNode or a sub-class)
### agent: reference to the agent to be controlled
### This function takes any number of additional arguments that will be passed to the BTNode and parsed using BTNode.ff29()
def ff23(type, agent, *args):
	node = type(agent, args)
	return node

###############################
### BEHAVIOR CLASSES:


##################
### Taunt
###
### Print disparaging comment, addressed to a given NPC
### Parameters:
###   0: reference to an NPC
###   1: node ID string (optional)

class ff22(ff28):

	### target: the enemy agent to taunt

	def ff29(self, args):
		ff28.ff29(self, args)
		self.target = None
		# First argument is the target
		if len(args) > 0:
			self.target = args[0]
		# Second argument is the node ID
		if len(args) > 1:
			self.id = args[1]

	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.target is not None:
			print("Hey " + str(self.target) + " I don't like you!")
		return ret

##################
### ff21
###
### Move the agent to a given (x, y)
### Parameters:
###   0: a point (x, y)
###   1: node ID string (optional)

class ff21(ff28):
	
	### target: a point (x, y)
	
	def ff29(self, args):
		ff28.ff29(self, args)
		self.target = None
		# First argument is the target
		if len(args) > 0:
			self.target = args[0]
		# Second argument is the node ID
		if len(args) > 1:
			self.id = args[1]

	def ff42(self):
		ff28.ff42(self)
		self.agent.navigateTo(self.target)

	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.target == None:
			# failed executability conditions
			return False
		elif ff89(self.agent.getLocation(), self.target) < self.agent.getRadius():
			# Execution succeeds
			return True
		else:
			# executing
			return None
		return ret

##################
### Retreat
###
### Move the agent back to the base to be healed
### Parameters:
###   0: percentage of hitpoints that must have been lost to retreat
###   1: node ID string (optional)


class ff20(ff28):
	
	### percentage: Percentage of hitpoints that must have been lost
	
	def ff29(self, args):
		ff28.ff29(self, args)
		self.percentage = 0.5
		# First argument is the factor
		if len(args) > 0:
			self.percentage = args[0]
		# Second argument is the node ID
		if len(args) > 1:
			self.id = args[1]

	def ff42(self):
		ff28.ff42(self)
		base = self.agent.world.getBaseForTeam(self.agent.getTeam())
		if base:
			self.agent.navigateTo(base.getLocation())
	
	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.agent.getHitpoints() > self.agent.getMaxHitpoints() * self.percentage:
			# fail executability conditions
			return False
		elif self.agent.getHitpoints() == self.agent.getMaxHitpoints():
			# Exection succeeds
			return True
		else:
			# executing
			return None
		return ret

##################
### ff19
###
### Find the closest minion and move to intercept it.
### Parameters:
###   0: node ID string (optional)


class ff19(ff28):

	### target: the minion to chase
	### timer: how often to replan

	def ff29(self, args):
		ff28.ff29(self, args)
		self.target = None
		self.timer = 50
		# First argument is the node ID
		if len(args) > 0:
			self.id = args[0]

	def ff42(self):
		ff28.ff42(self)
		self.timer = 50
		enemies = self.agent.world.getEnemyNPCs(self.agent.getTeam())
		if len(enemies) > 0:
			best = None
			dist = 0
			for e in enemies:
				if isinstance(e, Minion):
					d = ff89(self.agent.getLocation(), e.getLocation())
					if best == None or d < dist:
						best = e
						dist = d
			self.target = best
		if self.target is not None:
			navTarget = self.ff8()
			if navTarget is not None:
				self.agent.navigateTo(navTarget)


	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.target == None or self.target.isAlive() == False:
			# failed execution conditions
			return False
		elif self.target is not None and ff89(self.agent.getLocation(), self.target.getLocation()) < BIGBULLETRANGE:
			# succeeded
			return True
		else:
			# executing
			self.timer = self.timer - 1
			if self.timer <= 0:
				self.timer = 50
				navTarget = self.ff8()
				if navTarget is not None:
					self.agent.navigateTo(navTarget)
			return None
		return ret

	def ff8(self):
		if self.target is not None:
			return self.target.getLocation()
		else:
			return None

##################
### ff18
###
### Kill the closest minion. Assumes it is already in range.
### Parameters:
###   0: node ID string (optional)


class ff18(ff28):

	### target: the minion to shoot

	def ff29(self, args):
		ff28.ff29(self, args)
		self.target = None
		# First argument is the node ID
		if len(args) > 0:
			self.id = args[0]

	def ff42(self):
		ff28.ff42(self)
		self.agent.stopMoving()
		enemies = self.agent.world.getEnemyNPCs(self.agent.getTeam())
		if len(enemies) > 0:
			best = None
			dist = 0
			for e in enemies:
				if isinstance(e, Minion):
					d = ff89(self.agent.getLocation(), e.getLocation())
					if best == None or d < dist:
						best = e
						dist = d
			self.target = best


	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.target == None or ff89(self.agent.getLocation(), self.target.getLocation()) > BIGBULLETRANGE:
			# failed executability conditions
			return False
		elif self.target.isAlive() == False:
			# succeeded
			return True
		else:
			# executing
			self.ff7()
			return None
		return ret

	def ff7(self):
		if self.agent is not None and self.target is not None:
			self.agent.turnToFace(self.target.getLocation())
			self.agent.shoot()


##################
### ff17
###
### Move to intercept the enemy Hero.
### Parameters:
###   0: node ID string (optional)

class ff17(ff28):

	### target: the hero to chase
	### timer: how often to replan

	def ff29(self, args):
		ff28.ff29(self, args)
		self.target = None
		self.timer = 50
		# First argument is the node ID
		if len(args) > 0:
			self.id = args[0]

	def ff42(self):
		ff28.ff42(self)
		self.timer = 50
		enemies = self.agent.world.getEnemyNPCs(self.agent.getTeam())
		for e in enemies:
			if isinstance(e, Hero):
				self.target = e
				navTarget = self.ff8()
				if navTarget is not None:
					self.agent.navigateTo(navTarget)
				return None


	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.target == None or self.target.isAlive() == False:
			# fails executability conditions
			return False
		elif ff89(self.agent.getLocation(), self.target.getLocation()) < BIGBULLETRANGE:
			# succeeded
			return True
		else:
			# executing
			self.timer = self.timer - 1
			if self.timer <= 0:
				navTarget = self.ff8()
				if navTarget is not None:
					self.agent.navigateTo(navTarget)
			return None
		return ret

	def ff8(self):
		if self.target is not None:
			return self.target.getLocation()
		else:
			return None

##################
### ff16
###
### Kill the enemy hero. Assumes it is already in range.
### Parameters:
###   0: node ID string (optional)


class ff16(ff28):

	### target: the minion to shoot

	def ff29(self, args):
		ff28.ff29(self, args)
		self.target = None
		# First argument is the node ID
		if len(args) > 0:
			self.id = args[0]

	def ff42(self):
		ff28.ff42(self)
		self.agent.stopMoving()
		enemies = self.agent.world.getEnemyNPCs(self.agent.getTeam())
		for e in enemies:
			if isinstance(e, Hero):
				self.target = e
				return None

	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.target == None or ff89(self.agent.getLocation(), self.target.getLocation()) > BIGBULLETRANGE:
			# failed executability conditions
			return False
		elif self.target.isAlive() == False:
			# succeeded
			return True
		else:
			#executing
			self.ff7()
			return None
		return ret

	def ff7(self):
		if self.agent is not None and self.target is not None:
			self.agent.turnToFace(self.target.getLocation())
			self.agent.shoot()


##################
### ff15
###
### Only execute children if hitpoints are above a certain threshold.
### Parameters:
###   0: percentage of hitpoints that must be remaining to pass the daemon check
###   1: node ID string (optional)


class ff15(ff28):
	
	### percentage: percentage of hitpoints that must be remaining to pass the daemon check
	
	def ff29(self, args):
		ff28.ff29(self, args)
		self.percentage = 0.5
		# First argument is the factor
		if len(args) > 0:
			self.percentage = args[0]
		# Second argument is the node ID
		if len(args) > 1:
			self.id = args[1]

	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		if self.agent.getHitpoints() < self.agent.getMaxHitpoints() * self.percentage:
			# Check failed
			return False
		else:
			# Check didn't fail, return child's status
			return self.getChild(0).execute(delta)
		return ret

##################
### ff14
###
### Only execute children if agent's level is significantly above enemy hero's level.
### Parameters:
###   0: Number of levels above enemy level necessary to not fail the check
###   1: node ID string (optional)

class ff14(ff28):

	### advantage: Number of levels above enemy level necessary to not fail the check

	def ff29(self, args):
		ff28.ff29(self, args)
		self.advantage = 0
		# First argument is the advantage
		if len(args) > 0:
			self.advantage = args[0]
		# Second argument is the node ID
		if len(args) > 1:
			self.id = args[1]

	def execute(self, delta = 0):
		ret = ff28.execute(self, delta)
		hero = None
		# Get a reference to the enemy hero
		enemies = self.agent.world.getEnemyNPCs(self.agent.getTeam())
		for e in enemies:
			if isinstance(e, Hero):
				hero = e
				break
		if hero == None or self.agent.level <= hero.level + self.advantage:
			# fail check
			return False
		else:
			# Check didn't fail, return child's status
			return self.getChild(0).execute(delta)
		return ret





#################################
### MY CUSTOM BEHAVIOR CLASSES

class ff13(ff20):
	
	
	def execute(self, delta = 0):
		ff6(self.agent)
		ff5(self.agent)
		ff4(self.agent)
		ret = ff20.execute(self, delta)
		return ret

class ff12(ff19):
	
	def execute(self, delta = 0):
		ff6(self.agent)
		ff5(self.agent)
		ret = ff19.execute(self, delta)
		ff4(self.agent)
		return ret

	def ff8(self):
		if self.target is not None and self.target.getMoveTarget() is not None:
			return self.target.getMoveTarget()
		else:
			return None


class ff11(ff18):

	def execute(self, delta = 0):
		ff6(self.agent)
		ff4(self.agent)
		ret = ff18.execute(self, delta)
		return ret

	def ff7(self):
		if self.agent is not None and self.target is not None:
			ff3(self.agent, self.target)


class ff10(ff17):

	def execute(self, delta = 0):
		ff5(self.agent)
		ff4(self.agent)
		ret = ff17.execute(self, delta)
		return ret

	def ff8(self):
		if self.target is not None and self.target.getMoveTarget() is not None:
			return self.target.getMoveTarget()
		else:
			return None

class ff9(ff16):

	def execute(self, delta = 0):
		ff4(self.agent)
		ret = ff16.execute(self, delta)
		return ret

	def ff7(self):
		if self.agent is not None and self.target is not None:
			ff3(self.agent, self.target)


#################################
### MY HELPER FUNCTIONS

def ff6(agent):
	if agent.canFire():
		heroes = agent.getVisibleType(Hero)
		for h in heroes:
			if ff89(agent.getLocation(), h.getLocation()) < BIGBULLETRANGE:
				ff3(agent, h)
				return None

def ff5(agent):
	if agent.canFire():
		minions = agent.getVisibleType(Minion)
		for m in minions:
			if m.getTeam() != agent.getTeam() and ff89(agent.getLocation(), m.getLocation()) < BIGBULLETRANGE:
				ff3(agent, m)
				return None

def ff4(agent):
	if agent.canDodge():
		bullets = agent.getVisibleType(Bullet)
		for b in bullets:
			if b.getOwner().getTeam() != agent.getTeam() and ff89(agent.getLocation(), b.getLocation()) < agent.getRadius()*3:
				ff2(agent, b, agent.world)
				return None

def ff3(agent, target):
	targetLoc = target.getLocation()
	if target.isMoving():
		targetDir = target.getOrientation()
		rad = math.radians(targetDir)
		targetVec = (math.cos(rad)*SPEED[0]*10, -math.sin(rad)*SPEED[1]*10)
		targetPoint = (targetLoc[0] + targetVec[0], targetLoc[1] + targetVec[1])
		if agent.canFire():
			agent.turnToFace(targetPoint)
			agent.shoot()
	# ff60(agent.world.debug, targetPoint)
	else:
		agent.turnToFace(targetLoc)
		agent.shoot()

def ff2(agent, bullet, world):
	angle1 = bullet.getOrientation() - 90
	angle2 = bullet.getOrientation() + 90
	if angle1 < 0:
		angle1 = angle1 + 360
	if angle2 >= 360:
		angle2 = angle2 - 360
	rad1 = math.radians(angle1)
	v1 = (math.cos(rad1), -math.sin(rad1))
	point1 = ((v1[0]*agent.getRadius()*1.5)+agent.getLocation()[0], (v1[1]*agent.getRadius()*1.5)+agent.getLocation()[1])
	rad2 = math.radians(angle2)
	v2 = (math.cos(rad2), -math.sin(rad2))
	point2 = ((v2[0]*agent.getRadius()*1.5)+agent.getLocation()[0], (v2[1]*agent.getRadius()*1.5)+agent.getLocation()[1])
	d1 = ff89(point1, bullet.getLocation())
	d2 = ff89(point2, bullet.getLocation())
	if d1 > d2:
		b = ff1(angle1, point1, agent, world)
		if b:
			return True
		else:
			return ff1(angle2, point2, agent, world)
	else:
		b = ff1(angle2, point2, agent, world)
		if b:
			return True
		else:
			return ff1(angle1, point1, agent, world)
	return False

def ff1(angle, point, agent, world):
	# ff60(world.debug, point)
	if point[0] < 0 or point[0] > world.dimensions[0] or point[1] < 0 or point[1] > world.dimensions[1]:
		return False
	for l in world.getLines():
		if ff77(l, point) <= agent.getRadius()*2:
			return False
	agent.dodge(angle)
	return True



##############################################################
### MyHero

class RiedlHero(Hero, BehaviorTree):
	
	def __init__(self, position, orientation, world, image = AGENT, speed = SPEED, viewangle = 360, hitpoints = HEROHITPOINTS, firerate = FIRERATE, bulletclass = BigBullet, dodgerate = DODGERATE, areaeffectrate = AREAEFFECTRATE, areaeffectdamage = AREAEFFECTDAMAGE):
		Hero.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass, dodgerate, areaeffectrate, areaeffectdamage)
		BehaviorTree.__init__(self)
	
	
	def update(self, delta):
		Hero.update(self, delta)
		BehaviorTree.update(self, delta)
	
	
	def start(self):
		# Build the tree
		spec = ff25(self)
		if spec is not None and (isinstance(spec, list) or isinstance(spec, tuple)):
			self.buildTree(spec)
		else:
			self.setTree(ff24(self))
		# Start the agent
		Hero.start(self)
		BehaviorTree.start(self)
	
	
	def stop(self):
		Hero.stop(self)
		BehaviorTree.stop(self)




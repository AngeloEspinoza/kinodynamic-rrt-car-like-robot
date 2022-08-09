import pygame
import random
import math
import numpy as np

class Graph():
	"""
	A class for the Rapidly-Exploring Random Tree (RRT).
	
	Attributes
	----------
	start : tuple
		Initial position of the tree in X and Y respectively.
	goal : tuple
		End position of the tree in X and Y respectively.
	map_dimensions : tuple
		Map width and height in pixels.
	"""

	def __init__(self, start, goal, map_dimensions):
		self.x_init = start
		self.x_goal = goal
		self.desired_orientation = 0
		self.WIDTH, self.HEIGHT = map_dimensions
		self.EPSILON = 20.0

		self.max_simulation_time = 5.0 # Max forward simulation time
		self.max_simulations = 2 # Max forward simulations loops
		self.increment = self.max_simulation_time
		self.robot_last_position = []
		self.robot_last_orientation = []
		self.is_forward_simulation_finished = False
		self.is_goal_found = False
		self.is_forward_simulation_time_finished = False
		self.collision_free = False # Flag for the collision
		self.iteration = 0
		self.deleter = 3

		self.robot_last_position.append(start)
		self.robot_last_orientation.append(0)

		self.k = 0
		self.last_position = self.x_init
		self.last_orientation = 0
		self.u1 = random.uniform(0, 10)
		self.u2 = random.uniform(-30, 30)
		self.u = [self.u1, self.u2]

		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)
		self.YELLOW = (255, 255, 0)
		self.FUCSIA = (255, 0, 255)


	def is_free(self, point, obstacles, tree=None):
		"""Checks whether a node is colliding with an obstacle or not.

		When dealing with obstacles it is necessary to check 
		for the collision with them from the generated node.

		Parameters
		----------
		point : tuple
			Point to be checked.
		obstacles : pygame.Rect
			Rectangle obstacle.
		tree : list
			Tree containing all the coordinate nodes.

		Returns
		-------
		bool
		"""
		for obstacle in obstacles:
			if obstacle.collidepoint(point):
				# tree.remove(point)
				return False

		return True

	def generate_random_node(self):
		"""Generates a random node on the screen.

		The x and y coordinate is generated given an uniform
		distribution of the size of the screen width and height.

		Parameters
		----------
		None

		Returns
		-------
		tuple
			Coordinates of the random node. 
		"""
		# Sampling a robot configuration (x, y, theta) 
		self.x_rand = random.uniform(0, self.WIDTH), random.uniform(0, self.HEIGHT), \
			math.radians(random.uniform(-30, 30))

		if self.iteration&7 == 0:
			self.x_rand = self.x_goal

		return self.x_rand

	def euclidean_distance(self, p1, p2):
		"""Euclidean distance between two points.

		Parameters
		----------
		p1 : int
			Start point.
		p2 : int 
			End point.

		Returns
		-------
		float
			Euclidean distance metric.
		"""
		theta = self.x_rand[2]
		alpha = min(abs(self.last_orientation - theta), 2*math.pi - \
			abs(self.last_orientation - theta))

		return  math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + alpha**2)

	def nearest_neighbor(self, tree, x_rand):
		"""Returns the index of the nearest neighbor.
		
		The nearest neighbor from all the nodes in the tree
		to the randomly generated node.

		Parameters
		----------
		tree : list
			Tree containing all the coordinate nodes.
		x_rand : tuple 
			Coordinate of the random node generated.

		Returns
		-------
		tuple
			Nearest node to the random node generated.	
		"""
		distances = []

		try:
			# Start with the depuration of the repeated positions
			if len(tree) > self.max_simulations+1:
				tree.pop(self.deleter)
				self.robot_last_orientation.pop(self.deleter)
				self.deleter += self.max_simulations
		except Exception as e:
			pass

		for state in tree:
			distance = self.euclidean_distance(state, x_rand)
			distances.append(distance)

		# Index of the minimum distance to the generated random node
		self.min_distance = np.argmin(distances)
		x_near = tree[self.min_distance]

		return x_near

	def simulate(self, robot, event, environment, obstacles):
		"""Simulates forward in time the sampled control inputs.
		
		Given a control input u, it simulates the robot configuration.

		Parameters
		----------
		robot : robot object
			The robot where the control inputs will be simualated.
		event : Event
			All events happenning in the screen.
		environment : environment object
			Environment to draw the trails and the simulations forward
			in time.

		Returns
		-------
		list
			Collection of positions that the robot has passed through.
		"""
		time = robot.last_time // 1000	
		self.is_forward_simulation_finished = True
		collision_free = self.is_free(point=(robot.x, robot.y), obstacles=obstacles)
		if time <= self.max_simulation_time and self.k < self.max_simulations:
			robot.draw(map=environment.map) # Draw robot at each forward in time simulation
			robot.move(event=event, u=self.u) # Move the robot 
			environment.trail(position=(robot.x, robot.y)) # Draw the trail
		else:
			# Sample control input
			self.u1 = random.uniform(0, 10)
			self.u2 = random.uniform(-30, 30)
			self.u = [self.u1, self.u2]

			# One forward in time simulation done
			self.k += 1 
			# New simulation time for the next loop			
			self.max_simulation_time = time + self.increment 

			# Simulation time finished
			self.is_forward_simulation_time_finished = True

		if self.k > self.max_simulations:
			self.k = 0 # Restart forward simulations 
			self.is_forward_simulation_finished = False

			return self.robot_last_position 
		elif self.is_forward_simulation_time_finished:
			# Store the last trails of the robot
			environment.store_trails()

			# Simulation time restarted 
			self.is_forward_simulation_time_finished = False
			
			
			# if collision_free:
			# Store and last configuration of the robot
			self.store_configuration(position=(robot.x, robot.y),
				orientation=robot.theta)

			# Reconfigure the robot position and orientation
			robot.x, robot.y = self.last_position
			robot.theta = self.last_orientation

	def new_state(self, x_rand, robot, event, environment,
			obstacles=None):
		"""Advances a small step towards the random node.

		Given the forward simulations in time, it selects
		the more suitable to advance towards the x_rand
		node.

		Parameters
		----------
		x_rand : tuple
			Coordinate of the random node generated.
		x_near : tuple 
			Coordinate of the nearest neighbor node. 
		robot : robot object
			The robot where the control inputs will be simualated.
		event : Event
			All events happenning in the screen.
		environment : environment object
			Environment to draw the trails and the simulations forward
			in time.
		obstacles : list
			List of obstacles of type Rect.

		Returns
		-------
		tuple
			Best position towards x_rand.
		float
			Best orientation towards x_rand.
		"""
		if obstacles is None:
			obstacles = []

		# Simulation forward in time
		simulation = self.simulate(robot=robot, event=event,
			environment=environment, obstacles=obstacles)
		
		# Bias the tree every certain iterations
		if self.iteration%7 == 0:
			x_rand = self.bias()

		if simulation is not None:
			# Make a tree expansion
			self.iteration += 1 

			# New control input
			self.u_new = self.nearest_neighbor(simulation, x_rand)

			# Robot heading angle with the minimum distance to x_rand  
			self.theta_new = self.robot_last_orientation[self.min_distance]
			
			if self.is_goal_reached():
				pygame.draw.line(surface=environment.map,
					color=self.RED, start_pos=self.u_new,
					end_pos=self.x_goal)
				self.is_goal_found = True
			
			# Set last robot configuration and compare which is the nearest
			collision_free = self.is_free(point=self.u_new, obstacles=obstacles)			
			
			# self.draw_new_node(map=environment.map)

			if collision_free:
				self.collision_free = True
				self.last_position = self.u_new
				self.last_orientation = self.theta_new
				environment.compare(position=self.last_position)

			return self.u_new, self.theta_new

	def generate_parents(self, values, parent):
		"""Generates a list of parents and their children.
		
		Sets up a list of the parents and its corresponding
		children of the tree given a value and the value 
		of the nearest neighbor.

		Parameters
		----------
		values : list
			Collection of values of the assigned x_new node.
		parent : list
			Collection of parents to be fulfilled given its
			correspondant x_near value.

		Returns
		-------
		list
			Ordered collection of the parents.
		"""
		parent_value = values[self.min_distance] # Value nearest node
		parent_index = len(parent) # Used to be the index of the parent list
		parent.insert(parent_index, parent_value)

		if self.is_goal_reached():
			# Insert in the very last index the last value recorded plus one
			parent.insert(parent_index+1, values[-1]+1)

		return parent

	def store_configuration(self, position, orientation):
		"""Stores the current robot configuration.

		Given a position and an orientation, they are stored to later
		use.

		Parameters
		----------
		position : tuple
			Robot position (X, Y) in the C-space.
		orientation : tuple 
			Robot orientation (theta) in the C-space.

		Returns
		-------
		None
		"""
		self.robot_last_position.append(position)
		self.robot_last_orientation.append(orientation)

	def is_goal_reached(self):
		"""Checks whether the tree has reached the goal."""
		if abs(self.u_new[0] - self.x_goal[0]) < self.EPSILON and \
			abs(self.u_new[1] - self.x_goal[1]) < self.EPSILON: 

			return True

		return False

	def bias(self):
		"""Biasing by changing the random node."""
		return self.x_goal

	def draw_initial_node(self, map):
		"""Draws the x_init node."""
		pygame.draw.circle(surface=map, color=self.BLUE, 
			center=self.x_init, radius=3)

	def draw_goal_node(self, map):
		"""Draws the x_goal node."""
		pygame.draw.circle(surface=map, color=self.RED, 
			center=self.x_goal[:2], radius=3)

	def draw_random_node(self, map):
		"""Draws the x_rand node."""
		pygame.draw.circle(surface=map, color=self.GREEN, 
			center=self.x_rand[:2], radius=3)

	def draw_new_node(self, map):
		"""Draws the x_new node."""
		pygame.draw.circle(surface=map, color=self.BROWN, 
			center=self.u_new, radius=2.5)

	def draw_random_robot_configuration(self, robot_img, environment):
		self.img = pygame.image.load(robot_img)

		# Create the translation and rotation animation 
		rotated = pygame.transform.rotozoom(surface=self.img,
			angle=math.degrees(self.x_rand[2]), scale=1)
		rect = rotated.get_rect(center=self.x_rand[:2])			
		environment.map.blit(source=rotated, dest=rect)
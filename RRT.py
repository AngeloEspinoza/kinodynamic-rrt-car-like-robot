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
		Initial configuration of the tree in x, y and theta respectively.
	goal : tuple
		End configuration of the tree in x, y and theta respectively.
	map_dimensions : tuple
		Map width and height in pixels.
	"""

	def __init__(self, start, goal, map_dimensions):
		self.x_init = start
		self.x_goal = goal
		self.WIDTH, self.HEIGHT = map_dimensions

		self.max_simulation_time = 4 # Max forward simulation time
		self.max_simulations = 1 # Max forward simulations loops
		self.robot_last_position = []
		self.robot_last_orientation = []
		self.controls = []
		self.is_forward_simulation_finished = True
		self.is_goal_found = False
		self.is_forward_simulation_time_finished = False
		self.iteration = 0

		self.robot_last_position.append(self.x_init[:2])
		self.robot_last_orientation.append(math.degrees(self.x_init[2]))

		self.last_position = self.x_init[:2]
		self.last_orientation = self.x_init[2]
		self.u1 = random.uniform(-0.4, 0.4)
		self.u2 = random.uniform(-0.5, 0.5)
		self.u = [self.u1, self.u2]

		self.x_positions = []
		self.y_positions = []
		self.theta_orientations = []

		self.x_position = []
		self.y_position = []
		self.theta_orientation = []

		self.x_interpolation = []
		self.y_interpolation = []
		self.theta_interpolation = []

		self.interpolation_factor = 10

		self.x_interpolation.append(self.x_init[0])
		self.y_interpolation.append(self.x_init[1])
		self.theta_interpolation.append(self.x_init[2])

		self.u_news = []
		self.theta_news = []

		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)
		self.YELLOW = (255, 255, 0)
		self.FUCSIA = (255, 0, 255)

	def is_free(self, configuration, obstacles):
		"""Checks if a configuration is colliding with an obstacle.

		When dealing with obstacles it is necessary to check 
		for the collision with them from the generated node.

		Parameters
		----------
		configuration : pygame.Rect
			Configuration to be checked.
		obstacles : pygame.Rect
			Rectangle obstacle.

		Returns
		-------
		bool
		"""
		for obstacle in obstacles:
			if obstacle.colliderect(configuration):
				return False

		return True

	def generate_random_node(self, bias):
		"""Generates a random configuration.

		The position (x, y) of the robot is generated given	an uniform
		distribution of the size of the screen width and height, while
		the orientation (theta)	is generated from 0 to 360 degrees. 

		Parameters
		----------
		bias : int
			Goal bias percentage.

		Returns
		-------
		tuple
			Coordinates of the random node. 
		"""
		# Sampling a robot configuration (x, y, theta) 
		self.x_rand = int(random.uniform(0, self.WIDTH)), int(random.uniform(0, self.HEIGHT)), \
			math.radians(random.uniform(0, 360))

		bias_percentage = 10 - bias//10 if bias != 100 else 1

		if self.iteration%bias_percentage == 0:
			self.x_rand = self.x_goal

		return self.x_rand

	def metric(self, p1, p2):
		"""Metric defined for the configuration space (C-space) of a car-like robot.

		The C-space of a car-like robot can be represented as C = R^2 x S^1. 
		Therefore, a valid metric can be defined for such C-space.

		Parameters
		----------
		p1 : int
			Start configuration.
		p2 : int 
			End configuration.

		Returns
		-------
		float
			R^2 x S^1 metric.
		"""
		theta = self.x_rand[2] # rad
		alpha = min(abs(self.last_orientation - theta), 2*math.pi - 
			abs(self.last_orientation - theta))

		return  math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + alpha**2)

	def nearest_neighbor(self, tree, x_rand):
		"""Returns the index of the nearest neighbor.
		
		The nearest neighbor from all the configurations in the tree
		to the randomly generated configuration.

		Parameters
		----------
		tree : list
			Tree containing all the robot configurations.
		x_rand : tuple 
			Configuration of the random node generated.

		Returns
		-------
		tuple
			Nearest node to the random node generated.	
		"""
		distances = []

		for state in tree:
			distance = self.metric(state, x_rand)
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
		time = robot.dt

		self.is_forward_simulation_finished = False

		if time <= self.max_simulation_time:
			if self.show_tree_building:
				robot.draw(map=environment.map) # Draw robot at each forward in time simulation
			robot.move(event=event, u=self .u) # Move the robot 
			environment.trail(position=(robot.x, robot.y), draw_trail=self.show_tree_building)
			self.x_positions.append(robot.x)
			self.y_positions.append(robot.y)
			self.theta_orientations.append(robot.theta)
		else:
			self.is_forward_simulation_finished = True
			self.max_simulation_time = 4	
			robot.dt = 0 
			time = robot.dt

			# Sample control input
			self.u1 = random.uniform(-0.4, 0.4)
			self.u2 = random.uniform(-0.5, 0.5)
			self.u = [self.u1, self.u2]


			# Simulation time restarted 
			collision_free = self.is_free(configuration=robot.rect, obstacles=obstacles)

			if collision_free:
				# Store the last trails of the robot, and empty the piecewise trail list
				environment.store_trails()
				environment.empty_trails()

				# Store and last configuration of the robot
				self.store_configuration(position=(robot.x, robot.y), orientation=robot.theta)
				self.store_controls(controls=self.u)

				self.x_position.append(self.x_positions)
				self.y_position.append(self.y_positions)
				self.theta_orientation.append(self.theta_orientations)

				self.x_positions = []
				self.y_positions = []
				self.theta_orientations = []

				return self.robot_last_position

			else:
				# Empty the piecewise trail list
				environment.empty_trails()

				self.x_positions = []
				self.y_positions = []
				self.theta_orientations = []				

				# Reconfigure the robot position and orientation
				robot.x, robot.y = self.last_position[:2]
				robot.theta = self.last_orientation

				return None

	def new_state(self, x_rand, robot, event, environment, obstacles=None):
		"""Advances a small step towards the random node.

		Given the forward simulations in time, it selects
		the more suitable to advance towards the x_rand
		node.

		Parameters
		----------
		x_rand : tuple
			Coordinate of the random node generated.
		robot : robot object
			The robot where the control inputs will be simualated.
		event : Event
			All events happenning in the screen.
		environment : environment object
			Environment to draw the trails and the simulations forward in time.
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

		self.obstacles = obstacles

		# Simulation forward in time
		simulation = self.simulate(robot=robot, event=event, environment=environment,
			obstacles=obstacles)

		if simulation is not None:			
			self.iteration += 1 # One tree expansion done

			# Robot position, and orientation with the minimum distance to x_rand  
			self.u_new = self.nearest_neighbor(simulation, x_rand) # New control input
			self.theta_new = self.robot_last_orientation[self.min_distance]

			# Append new controls to a list 
			self.u_news.append(self.u_new)
			self.theta_news.append(self.theta_new)

			if self.is_goal_reached():
				self.goal_configuration = self.number_of_nodes-1 
				self.is_goal_found = True
			
			self.last_position = self.u_new
			self.last_orientation = self.theta_new

			robot.x, robot.y = self.last_position[:2]
			robot.theta = self.last_orientation

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


	def path_to_goal(self):
		"""Collects the parents of each node.
		
		Given the x_goal node, it searches the next parent
		continously until it reaches the x_init node.

		Parameters
		----------
		None

		Returns
		-------
		None
		"""
		self.path = []
		self.path.append(self.goal_configuration)
		new_configuration = self.parent[self.goal_configuration] # Parent of the x_goal node

		while new_configuration != 0:
			# Append the parent of the parent and update the configuration
			self.path.append(new_configuration)
			new_configuration = self.parent[new_configuration]

		# Append the parent 0 (correspondant to the x_init node)
		self.path.append(0)

	def get_path_coordinates(self):
		"""Collects the correspondant coordinates.

		Given a list of the nodes it searches the correspondant
		coordinates in the tree.

		Parameters
		----------
		None

		Returns
		-------
		None
		"""
		self.path_coordinates = []

		for node in self.path:
			x, y = self.tree[node]
			self.path_coordinates.append((x, y))

		for i in range(len(self.x_position)):
			for j in range(len(self.path_coordinates)):
				x_position = self.x_position[i][-1] == self.path_coordinates[j][0]
				y_position = self.y_position[i][-1] == self.path_coordinates[j][1]
				if  x_position and y_position:
					self.x_interpolation.append(self.x_position[i])
					self.y_interpolation.append(self.y_position[i])
					self.theta_interpolation.append(self.theta_orientation[i])

		return self.path_coordinates

	def is_goal_reached(self):
		"""Checks if the tree has reached the goal."""
		goal_position, goal_orientation = self.x_goal[:2], self.x_goal[2]
		last_position_x, last_position_y = self.last_position[0], self.last_position[1]
		goal_position_x, goal_position_y = goal_position[0], goal_position[1]
		last_orientation = math.radians(self.last_orientation)

		# Upper and lower allowed offsets
		upper_offset_x = goal_position_x+self.position_boundary
		lower_offset_x = goal_position_x-self.position_boundary
		upper_offset_y = goal_position_y+self.position_boundary
		lower_offset_y = goal_position_y-self.position_boundary
		upper_offset_theta = goal_orientation+self.orientation_boundary
		lower_offset_theta = goal_orientation-self.orientation_boundary

		if last_orientation >= upper_offset_theta:
			last_orientation = -2*math.pi + last_orientation
		elif last_orientation <= lower_offset_theta:
			last_orientation = 2*math.pi + last_orientation

		# Allowed position and orientation region for the robot to reach goal
		upper_condition_x = last_position_x <= upper_offset_x
		lower_condition_x = last_position_x >= lower_offset_x
		upper_condition_y = last_position_y <= upper_offset_y
		lower_condition_y = last_position_y >= lower_offset_y
		upper_condition_theta = last_orientation <= upper_offset_theta 
		lower_condition_theta = last_orientation >= lower_offset_theta

		position_conditions = upper_condition_x and lower_condition_x and upper_condition_y and \
			lower_condition_y
		orientation_conditions = upper_condition_theta and lower_condition_theta

		if position_conditions and orientation_conditions:
			return True

		return False

	def store_controls(self, controls):
		"""Simply stores the controls."""
		self.controls.append(controls)

	def draw_initial_node(self, map):
		"""Draws the x_init node."""
		pygame.draw.circle(surface=map, color=self.BLUE, 
			center=self.x_init[:2], radius=3)

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
		for i in range(len(self.u_news)):
			pygame.draw.circle(surface=map, color=self.BROWN, 
				center=self.u_news[i], radius=3)

	def draw_initial_robot_configuration(self, robot_img, environment):
		"""Draws the x_init configuration."""
		position = self.x_init[:2]
		orientation = math.degrees(self.x_init[2])

		self.draw_robot_configuration(image=robot_img, position=position, orientation=orientation,
			environment=environment)

	def draw_goal_robot_configuration(self, robot_img, environment):
		"""Draws the x_goal configuration."""
		position = self.x_goal[:2]
		orientation = math.degrees(self.x_goal[2])

		self.draw_robot_configuration(image=robot_img, position=position, orientation=orientation,
			environment=environment)

	def draw_random_robot_configuration(self, robot_img, environment):
		"""Draws the x_rand configuration."""
		position = self.x_rand[:2]
		orientation = math.degrees(self.x_rand[2])

		rect = self.draw_robot_configuration(image=robot_img, position=position,
		 	orientation=orientation, environment=environment)

		return rect

	def draw_new_robot_configuration(self, robot_img, environment):
		"""Draws the x_goal configuration."""
		for i in range(len(self.u_news)):
			position = self.u_news[i]
			orientation = self.theta_news[i]
			self.draw_robot_configuration(image=robot_img, position=position,
			 	orientation=orientation, environment=environment)

	def draw_path_to_goal(self, map):
		"""Draws the path from the x_goal node to the x_init node."""
		for i in range(len(self.path_coordinates)-1):
			pygame.draw.circle(surface=map, color=self.BLACK, center=self.path_coordinates[i],
				radius=3)

	def draw_interpolation(self, robot, environment):
		"""Draws the interpolation from the initial configuration to the goal configuration."""
		x_interpolation = self.x_interpolation[1:]
		y_interpolation = self.y_interpolation[1:]
		theta_interpolation = self.theta_interpolation[1:]

		for x, y, theta in zip(x_interpolation, y_interpolation, theta_interpolation):
			for interpolation, (i, j, k) in enumerate(zip(x, y, theta)):
				if self.obstacles != []:
					environment.draw_obstacles()

				if interpolation%self.interpolation_factor == 0:
					self.draw_robot_configuration(image=robot.img, position=(i, j), orientation=k,
						environment=environment)

		self.draw_initial_robot_configuration(robot_img=robot.init, environment=environment)
		self.draw_goal_robot_configuration(robot_img=robot.goal, environment=environment)
		
		pygame.display.update()
		pygame.time.delay(5000)
		environment.map.fill(self.WHITE) 

	def draw_trajectory(self, robot, environment):
		"""Draws the interpolation from the initial configuration to the goal configuration."""
		x_interpolation = self.x_interpolation[1:]
		y_interpolation = self.y_interpolation[1:]
		theta_interpolation = self.theta_interpolation[1:]

		for x, y, theta in zip(x_interpolation, y_interpolation, theta_interpolation):
			for interpolation, (i, j, k) in enumerate(zip(x, y, theta)):
				if self.obstacles != []:
					environment.draw_obstacles()
				
				self.draw_initial_robot_configuration(robot_img=robot.init, environment=environment)
				self.draw_goal_robot_configuration(robot_img=robot.goal, environment=environment)
				self.draw_robot_configuration(image=robot.img, position=(i, j), orientation=k,
					environment=environment)				

				# Refresh the screen
				pygame.display.update()
				pygame.time.delay(3) 
				environment.map.fill(self.WHITE)

	def draw_robot_configuration(self, image, position, orientation, environment):
		"""Creates the translation and rotation animation.""" 
		rotated = pygame.transform.rotozoom(surface=image, angle=orientation, scale=1)
		rect = rotated.get_rect(center=position)			
		environment.map.blit(source=rotated, dest=rect)  

		return rect
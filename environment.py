import pygame 

class Environment():
	"""
	A class of the map where the car-like robot will be moving around.

	Attributes
	----------
	dimensions : tuple
		The X and Y window dimensions.
	show_tree_building : bool
		Flag to show the RRT building.
	has_obstacles : bool
		Flag to show the obstacles.
	"""
	
	def __init__(self, dimensions, show_tree_building, has_obstacles):
		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)
		self.YELLOW = (255, 255, 0)
		self.GRAY = (105, 105, 105)

		self.has_obstacles = has_obstacles
		self.trail_set = []
		self.trail_sets = []
		self.obstacles = []

		# Map dimensions
		self.WIDTH, self.HEIGHT = dimensions 

		# Window settings
		self.FPS = 120
		pygame.display.set_caption('Kinodynamic RRT - Car-like Robot')

		if show_tree_building:
			self.show_screen()
		else:
			self.hide_screen()


	def trail(self, position, draw_trail):
		"""Draws the robot trail.
		
		Given two immediate different positions of the robots a straight
		line is draw between the positions. The same action is done so on
		and so forth until the robot stops.

		Parameters
		----------
		positon : tuple
			The robots position in X and Y at each time.
		draw_trail : bool
			Shows the trail traveled by the robot.
		
		Returns
		-------
		None
		"""		
		for i in range(len(self.trail_set)-1):
			trail_set_start = self.trail_set[i][0], self.trail_set[i][1]
			trail_set_end = self.trail_set[i+1][0], self.trail_set[i+1][1]
			if draw_trail: 
				pygame.draw.line(surface=self.map, color=self.BROWN, start_pos=trail_set_start,
					end_pos=trail_set_end)

		self.trail_set.append(position)

	def empty_trails(self):
		"""Empty the list so does not appear a straight line."""	
		self.trail_set = []


	def store_trails(self):
		"""Stores the set of trails.

		Stores the trail of the robot each time the function is
		called. 

		Parameters
		----------
		None

		Returns
		-------
		None
		"""
		# Append each simulated trail 
		self.trail_sets.append(self.trail_set)

	def compare(self, position):
		"""Compares the last position with the set of all the
		trails.

		Given a position, it is compared with the last stored
		position of the set of trails so that if it matches, 
		such trail is drawn.

		Parameters
		----------
		position : tuple
			Nearest last position of the robot after the simulation
			forward in time.

		Returns
		-------
		None
		"""
		for i in range(len(self.trail_sets)):
			if self.trail_sets[i][-1] == position:
				self.draw_trajectory_trail()

	def draw_trajectory_trail(self):
		"""Draws the trajectory given a trail.

		Given a trail, it draws the best trail in the RRT tree from
		the candidate nodes to the random generated node.

		Parameters
		----------
		None
		
		Returns
		-------
		None
		"""
		for trail in self.trail_sets:
			for i in range(len(trail)-1):
				trail_set_start = trail[i][0], trail[i][1]
				trail_set_end = trail[i+1][0], trail[i+1][1]
				pygame.draw.line(surface=self.map, color=self.RED, start_pos=trail_set_start,
					end_pos=trail_set_end)

	def make_obstacles_T(self, initial_point):
		"""
		Given a initial point, it makes a obstacle with shape of T.
		
		Parameters
		----------
		initial_point : tuple
			X and Y coordinates, starting from the top-left most part where
			the obstacle will be placed.
		
		Returns
		-------
		list
			A collection of sides composing the T obstacle.			
		"""
		x, y = initial_point[0], initial_point[1]
		width, height = 50, 150

		side1 = pygame.Rect(x, y, height, width)
		side2 = pygame.Rect((x+height//2) - width//2, y, width, height)

		obstacle = [side1, side2]

		return obstacle

	def make_obstacles_L(self, initial_point):
		"""
		Given a initial point, it makes a obstacle with shape of L.
		
		Parameters
		----------
		initial_point : tuple
			X and Y coordinates, starting from the top-left most part where
			the obstacle will be placed.
		
		Returns
		-------
		list
			A collection of sides composing the L obstacle.
		"""	
		x, y = initial_point[0], initial_point[1]
		width, height = 50, 150

		side1 = pygame.Rect(x, y, width, height)
		side2 = pygame.Rect(x, y+height-width, height, width)

		obstacle = [side1, side2]

		return obstacle

	def make_obstacles(self):
		"""Generate the obstacles to be placed on the final map."""
		obstacle1 = self.make_obstacles_T(initial_point=(350, 200))
		obstacle2 = self.make_obstacles_L(initial_point=(150, 20))

		self.obstacles.append(obstacle1)
		self.obstacles.append(obstacle2)

		return self.obstacles

	def draw_obstacles(self):
		"""Draw each side of the obstacles."""
		obstacles = []

		for obstacle in self.obstacles:
			for side in obstacle:
				pygame.draw.rect(surface=self.map, color=self.GRAY, rect=side)
				obstacles.append(side)

		return obstacles

	def show_screen(self):
		"""Shows the screen by setting the display flag to SHOWN."""
		self.map = pygame.display.set_mode(size=(self.WIDTH, self.HEIGHT), flags=pygame.SHOWN)
		self.map.fill(self.WHITE)

		if self.has_obstacles:
			self.draw_obstacles()

	def hide_screen(self):
		"""Hides the screen by setting the display flag to HIDDEN."""
		self.map = pygame.display.set_mode(size=(self.WIDTH, self.HEIGHT), flags=pygame.HIDDEN)
		self.map.fill(self.WHITE)

	def clear_screen(self):
		""""""
		self.map.fill(self.WHITE)		
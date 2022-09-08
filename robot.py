import pygame 
import math

class Robot():
	"""
	A class to represent a car-like robot dimensions, start position, 
	heading angle, and velocity.

	Attributes
	----------
	start : tuple
		Initial configuration of the robot in X, Y, and theta, respectively.
	robot_img : str
		The robot image path.
	length : float
		The length between the rear wheels and front wheels of the
		car-like robot.
	"""	

	def __init__(self, start, robot_img, length):
		# Robot settings
		self.x = start[0] # X position
		self.y = start[1] # Y position
		self.theta = math.degrees(start[2]) # Initial heading angle (rad)
		self.phi = 0 # Initial steering angle (rad)
		self.v = self.meters_to_pixels(meters=0.01) # m/s 
		self.L = self.meters_to_pixels(meters=length) # m (meters)
		self.max_speed = self.meters_to_pixels(meters=0.02) # m/s
		self.min_speed = self.meters_to_pixels(meters=-0.02) # m/s

		# Graphics
		self.init = pygame.image.load(robot_img[0])
		self.goal = pygame.image.load(robot_img[1])
		self.img = pygame.image.load(robot_img[2])
		self.rotated = self.img
		self.rect = self.rotated.get_rect(center=(self.x, self.y))
		
		# Time variant 
		self.dt = 0.0 # Delta time
		self.last_time = pygame.time.get_ticks() # Last time recorded
		self.x_position = []
		self.y_position = []
		self.theta_orientation = []

	def meters_to_pixels(self, meters):
		"""Converts from meters to pixels.
		
		Parameters
		----------
		meters: float
			The meters to be converted into pixels.

		Returns
		-------
		float
			The meters in pixels.
		"""
		return meters*3779.52

	def draw(self, map):
		"""Draws the robot on map.
		
		Parameters
		----------
		map : pygame.Surface
			The where the robot will be drawn.
		"""
		map.blit(source=self.rotated, dest=self.rect)

	def move(self, event=None, u=None):
		"""Moves the robot with key strokes.
		
		Takes the screen only if it is available.

		Parameters
		----------
		event : Event
			All events happenning in the screen.
		u : list
			Linear and angular velocity.
		"""
		# Stablish the control input u 
		self.u1, self.u2 = u

		# Car-like kinematic robot model
		self.x += self.u1 * math.cos(math.radians(self.theta)) * self.dt
		self.y -= self.u1 * math.sin(math.radians(self.theta)) * self.dt 			
		self.theta += self.u2 * self.dt
		
		# Create the translation and rotation animation 
		self.rotated = pygame.transform.rotozoom(surface=self.img,
			angle=self.theta, scale=1)
		self.rect = self.rotated.get_rect(center=(self.x, self.y))

		self.x_position.append(self.x)
		self.y_position.append(self.y)
		self.theta_orientation.append(self.theta)
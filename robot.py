import pygame 
import math

class Robot():
	"""
	A class to represent a car-like robot dimensions, start position, 
	heading angle, and velocity.

	Attributes
	----------
	start_pos : tuple
		Initial position of the robot in X and Y respectively.
	robot_img : str
		The robot image path.
	length : float
		The length between the rear wheels and front wheels of the
		car-like robot.
	"""	

	def __init__(self, start_pos, robot_img, length):
		# Robot settings
		self.x = start_pos[0] # X position
		self.y = start_pos[1] # Y position
		self.theta = 0 # Initial heading angle (rad)
		self.phi = 0 # Initial steering angle (rad)
		self.v = self.meters_to_pixels(meters=0.01) # m/s 
		self.L = self.meters_to_pixels(meters=length) # m (meters)
		self.max_speed = self.meters_to_pixels(meters=0.02) # m/s
		self.min_speed = self.meters_to_pixels(meters=-0.02) # m/s

		# Graphics 
		self.img = pygame.image.load(robot_img)
		self.rotated = self.img
		self.rect = self.rotated.get_rect(center=(self.x, self.y))
		
		# Time variant 
		self.dt = 0 # Delta time
		self.last_time = pygame.time.get_ticks() # Last time recorded
	
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
		"""
		if event is not None:
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_RIGHT:
					self.v += self.meters_to_pixels(0.0001)
				elif event.key == pygame.K_LEFT:
					self.v -= self.meters_to_pixels(0.0001)
				elif event.key == pygame.K_UP:
					self.phi += math.radians(0.03)
				elif event.key == pygame.K_DOWN:
					self.phi -= math.radians(0.03)

		# Stablish the control input u to be the steering angle phi
		self.u1 = u[0]
		self.u2 = u[1]

		# print(f'u1: {self.u1}m/s')
		# print(f'u2: {math.radians(self.u2)}rad/s')

		# Car-like kinematic robot model
		self.x += self.u1 * math.cos(math.radians(self.theta)) * self.dt
		self.y -= self.u1 * math.sin(math.radians(self.theta)) * self.dt 			
		self.theta += self.u2 * self.dt
		
		# Create the translation and rotation animation 
		self.rotated = pygame.transform.rotozoom(surface=self.img,
			angle=self.theta, scale=1)
		self.rect = self.rotated.get_rect(center=(self.x, self.y))
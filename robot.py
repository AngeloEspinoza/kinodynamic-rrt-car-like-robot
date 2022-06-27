import pygame 

class Robot:
	"""
	A class to represent a car-like robot dimensions, start position, 
	heading angle, and velocity.

	Attributes
	----------
	start_pos : tuple
		Initial position of the robot in X and Y respectively.
	robot_img :

	width :  
	"""
	def __init__(self, start_pos, robot_img, width):
		print(width)
		# Robot settings
		self.w = self.pixel_to_meters(width) # m (meters)
		self.x = start_pos[0] # X position
		self.y = start_pos[1] # Y position
		self.theta = 0 # Initial heading angle
		self.velocity = self.pixel_to_meters(meters=0.01) # m/s
		self.max_speed = self.pixel_to_meters(meters=0.02) # m/s
		self.min_speed = self.pixel_to_meters(meters=-0.02) # m/s

		# Graphics 
		self.img = pygame.image.load(robot_img)
		self.rotated = self.img
		self.rect = self.rotated.get_rect(center=(self.x, self.y))

	
	def pixel_to_meters(self, meters):
		"""Converts from pixel to meters."""
		return meters*3779.52

	def draw(self, map):
		map.blit(source=self.rotated, dest=self.rect)


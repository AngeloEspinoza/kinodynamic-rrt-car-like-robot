import pygame 

class Environment(object):
	"""docstring for Environment"""
	def __init__(self, dimentions):
		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)

		# Map dimentions
		self.WIDTH, self.HEIGHT = dimentions[0], dimentions[1] 

		# Window settings
		self.FPS = 60
		pygame.display.set_caption('Kynodinamic RRT - Car-like Robot')
		self.map = pygame.display.set_mode(size=(self.WIDTH,
			self.HEIGHT))
import pygame
import environment
import robot

# Constants
ROBOT_IMG_PATH = r'robot.png'

# Initialization 
pygame.init()

# Initial position of the robots
start = 50, 50

# Map dimentions
map_dimentions = 640, 480

# Instantiating the environment and robot
environment = environment.Environment(dimentions=map_dimentions)
robot = robot.Robot(start_pos=start, robot_img=ROBOT_IMG_PATH,
	length=0.01)

def main():
	run = True

	while run:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False
			robot.move(event)

		# dt in seconds
		robot.dt = (pygame.time.get_ticks() - robot.last_time) / 1000 
		robot.last_time = pygame.time.get_ticks()
		robot.draw(map=environment.map)
		pygame.display.update()
		environment.map.fill(environment.BLACK)
		robot.move(event)

if __name__ == '__main__':
	main()
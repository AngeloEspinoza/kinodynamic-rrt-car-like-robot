import pygame
import environment
import robot

# Constants
ROBOT_IMG_PATH = r'robot.png'

# Initialization 
pygame.init()

# Initial position of the robot
start = 50, 50

# Map dimentions
map_dimentions = 640, 480

# Instantiating the environment and robot
environment = environment.Environment(dimentions=map_dimentions)
robot = robot.Robot(start_pos=start, robot_img=ROBOT_IMG_PATH,
	width=0.01)

def main():
	run = True
	while run:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		robot.draw(map=environment.map)
		pygame.display.update()
		environment.map.fill(environment.BLACK)

if __name__ == '__main__':
	main()
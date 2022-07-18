import pygame
import random
import math
import environment
import robot
import RRT

# Constants
ROBOT_IMG_PATH = r'robot.png'

# Initialization 
pygame.init()

# Initial position of the robots
x_init = 50, 50

# Map dimentions
map_dimensions = 640, 480

# Instantiating the environment and robot
environment = environment.Environment(dimensions=map_dimensions)
robot = robot.Robot(start_pos=x_init, robot_img=ROBOT_IMG_PATH,
	length=0.01)

graph = RRT.Graph(start=x_init, goal=(540, 380), 
		map_dimensions=map_dimensions)

def main():
	run = True
	# Initial positibon of the robot
	x_init = 50, 50
	clock = pygame.time.Clock()	
	tree = []
	parent = []
	values = []
	graph.tree = tree
	tree.append(x_init) # Append initial node
	parent.append(0)
	obstacles = environment.draw_obstacles()
	node_value = 0
	iteration = 0

	while run:
		# Make sure the loop runs at 60 FPS
		clock.tick(environment.FPS) # CHECK IF TOO SLOW SIMULATION
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		# dt in seconds
		robot.dt = (pygame.time.get_ticks() - robot.last_time) / 1000 
		robot.last_time = pygame.time.get_ticks()

		graph.draw_initial_node(map=environment.map)
		graph.draw_goal_node(map=environment.map)

		if not graph.is_simulation_finished:
			x_rand = graph.generate_random_node()
			# x_near = graph.nearest_neighbor(tree, x_rand)
			graph.draw_random_node(node=x_rand, map=environment.map)			


		x_new = graph.new_state(x_rand, robot, event,
			environment, obstacles)	

		if graph.collision_free:
			print(parent)
			values.append(node_value)
			parent = graph.generate_parents(values, parent)
			node_value += 1
			graph.collision_free = False

		pygame.display.update()

if __name__ == '__main__':
	main()
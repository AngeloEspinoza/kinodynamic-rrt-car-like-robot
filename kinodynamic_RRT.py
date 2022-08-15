import pygame
import environment
import robot
import RRT
import math

# Constants
ROBOT_IMG_PATH = r'robot0.png'

# Initialization 
pygame.init()

# Initial and final configuration of the robot
x_init = 50, 50, 0.17 # px, px, rads
x_goal = 100, 100, -0.17 # px, px, rads

# Map dimensions
map_dimensions = 640, 480

# Instantiating the environment and robot
environment = environment.Environment(dimensions=map_dimensions)
robot = robot.Robot(start_pos=x_init, robot_img=ROBOT_IMG_PATH,
	length=0.01)

graph = RRT.Graph(start=x_init, goal=x_goal, 
		map_dimensions=map_dimensions)

def main():
	run = True
	clock = pygame.time.Clock()	
	tree = []
	parent = []
	values = []
	tree.append(x_init[:2]) # Append initial node
	parent.append(0)
	values.append(0)
	node_value = 0
	iteration = 0

	while run:
		# obstacles = environment.draw_obstacles()
		obstacles = []
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
		graph.draw_initial_robot_configuration(robot_img=ROBOT_IMG_PATH, environment=environment)
		graph.draw_goal_robot_configuration(robot_img=ROBOT_IMG_PATH, environment=environment)
		environment.draw_trajectory_trail()

		if not graph.is_goal_found:
			if not graph.is_forward_simulation_finished:
				x_rand = graph.generate_random_node()	
				print(f'goal position: {graph.x_goal[:2]}, goal orientation: {graph.x_goal[2]}')
				print(f'robot last position: {graph.last_position[:2]} pxs, robot last orientation: {math.radians(graph.last_orientation)} rads')
				
			x_new = graph.new_state(x_rand, robot, event,
				environment, obstacles)	

			if graph.collision_free:
				# Append in function of the simulations done
				for _ in range(graph.max_simulations):
					values.append(node_value)
				parent = graph.generate_parents(values, parent)
				node_value += 1
				graph.collision_free = False

		graph.draw_random_robot_configuration(robot_img=ROBOT_IMG_PATH, environment=environment)
		graph.draw_random_node(map=environment.map)

		iteration += 1
		pygame.display.update()
		environment.map.fill(environment.WHITE)

if __name__ == '__main__':
	main()
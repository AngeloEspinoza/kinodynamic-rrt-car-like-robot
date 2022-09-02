import pygame
import environment
import robot
import RRT
import math

# Initialization 
pygame.init()

# Constants
ROBOT_IMG_PATH = r'robot.png'
MAP_DIMENSIONS = 640, 480

# Initial and final configuration of the robot
x_init = 50, 50, 0.17 # px, px, rads
x_goal = 200, 200, -0.17 # px, px, rads

# Instantiating the environment, robot, and RRT 
environment = environment.Environment(dimensions=MAP_DIMENSIONS)
robot = robot.Robot(start_pos=x_init, robot_img=ROBOT_IMG_PATH,
	length=0.01)
graph = RRT.Graph(start=x_init, goal=x_goal, map_dimensions=MAP_DIMENSIONS)

def main():
	run = True
	clock = pygame.time.Clock()	
	tree = []
	orientation_tree = []
	parent = []
	values = []
	tree.append(x_init[:2]) # Append initial node
	orientation_tree.append(x_init[2]) # Append initial node
	parent.append(0)
	values.append(0)
	node_value = 0
	iteration = 0
	environment.make_obstacles()

	while run:
		# Make sure the loop runs at 60 FPS
		clock.tick(environment.FPS) # CHECK IF TOO SLOW SIMULATION
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		obstacles = environment.draw_obstacles()
		graph.draw_initial_node(map=environment.map)
		graph.draw_goal_node(map=environment.map)
		graph.draw_initial_robot_configuration(robot_img=robot.img, environment=environment)
		graph.draw_goal_robot_configuration(robot_img=robot.img, environment=environment)
		environment.draw_trajectory_trail()

		if not graph.is_goal_found:
			if not graph.is_forward_simulation_finished:
				x_rand = graph.generate_random_node()
				random_rect = graph.draw_random_robot_configuration(robot_img=robot.img, environment=environment)
				rand_collision_free = graph.is_free(configuration=random_rect, obstacles=obstacles)

			if rand_collision_free:
				x_new = graph.new_state(x_rand, robot, event,
					environment, obstacles)	

				if x_new is not None:
					tree.append(x_new[0])
					orientation_tree.append(x_new[1])
					
					values.append(node_value)
					parent = graph.generate_parents(values, parent)

					node_value += 1
					graph.number_of_nodes = len(tree)
					graph.num_free_collision = 0					

				if graph.is_goal_found:
					graph.parent = parent
					graph.tree = tree
					graph.orientation_tree = orientation_tree
					graph.path_to_goal()
					graph.get_path_coordinates()

		if graph.is_goal_found:
			graph.draw_path_to_goal(map_=environment.map)
			graph.draw_trajectory(robot_img=robot.img, environment=environment)
			graph.draw_interpolation(robot_img=robot.img, environment=environment)
			pygame.display.update()
			pygame.time.delay(5000) 

		if rand_collision_free:
			random_rect = graph.draw_random_robot_configuration(robot_img=robot.img, environment=environment)
			graph.draw_random_node(map=environment.map)

		iteration += 1
		robot.dt += 0.05
		pygame.display.update()
		environment.map.fill(environment.WHITE)

if __name__ == '__main__':
	main()
import pygame
import environment
import robot
import RRT
import math
import argparse
import sys
import os

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the kinodynamic RRT algorithm for a \
	car-like robot.')
parser.add_argument('-o', '--obstacles', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Obstacles on the map')
parser.add_argument('-n', '--nodes', type=int, metavar='', required=False, default=5000,
	help='Maximum number of nodes')
parser.add_argument('-dt', '--delta', type=float, metavar='', required=False, default=0.05,
	help='Fixed time interval')
parser.add_argument('-init', '--x_init', nargs='+', type=float, metavar='', required=False,
	default=(50, 50, 0.17), help='Initial node configuration in X, Y, and theta in pixels and \
	radians, respectively')
parser.add_argument('-goal', '--x_goal', nargs='+', type=float, metavar='', required=False,
	default=(540, 380, -0.17), help='Goal node configuration in X, Y, and theta in pixels and \
	radians, respectively')
parser.add_argument('-src', '--show_random_configurations', type=bool,
	action=argparse.BooleanOptionalAction, metavar='', required=False, 
	help='Show random configurations on screen')
parser.add_argument('-snc', '--show_new_configurations', type=bool,
 	action=argparse.BooleanOptionalAction, metavar='', required=False,
 	help='Show new configurations on screen')
parser.add_argument('-bp', '--bias_percentage', type=int, metavar='', required=False, default=50,
	help='Amount of bias the RRT from 1 to 100')
parser.add_argument('-ptg', '--path_to_goal', type=bool, action=argparse.BooleanOptionalAction, 
	metavar='', required=False, default=False, help='Draws the milestones from path to goal')
parser.add_argument('-si', '--show_interpolation', type=bool, action=argparse.BooleanOptionalAction, 
	metavar='', required=False, default=True, help='Draws the configurations needed to reach the \
	 	goal')
parser.add_argument('-mr', '--move_robot', type=bool, action=argparse.BooleanOptionalAction, 
	metavar='', required=False, default=True,
	help='Shows the movements of the robot from start to end')
parser.add_argument('-pb', '--position_boundary', type=int, metavar='', required=False, default=25,
	help='Allowed position region of the pixels for the robot to reach the goal')
parser.add_argument('-ob', '--orientation_boundary', type=float, metavar='', required=False,
 	default=math.pi, help='Allowed orientation region of the angle for the robot to reach the goal')
parser.add_argument('-stb', '--show_tree_building', type=bool, 
	action=argparse.BooleanOptionalAction,	metavar='', required=False, default=True,
	help='Shows the simulation forward in time')
args = parser.parse_args()

# Initialization 
pygame.init()

# Constants
ROBOT_INIT_PATH = 'images/initial.png'
ROBOT_GOAL_PATH = 'images/goal.png'
ROBOT_IMG_PATH = 'images/robot.png'
MAP_DIMENSIONS = 640, 480

robot_images = [ROBOT_INIT_PATH, ROBOT_GOAL_PATH, ROBOT_IMG_PATH]

# Initial and final configuration of the robot
x_init = args.x_init # px, px, rad
x_goal = args.x_goal # px, px, rad

# Instantiating the environment, robot, and RRT 
environment = environment.Environment(dimensions=MAP_DIMENSIONS, 
	show_tree_building=args.show_tree_building, has_obstacles=args.obstacles)
robot = robot.Robot(start=x_init, robot_img=robot_images, length=0.01)
graph = RRT.Graph(start=x_init, goal=x_goal, map_dimensions=MAP_DIMENSIONS)

def main():
	run = True
	clock = pygame.time.Clock()	
	tree = [] # Positions in pixels
	orientation_tree = [] # Orientations in degrees
	parent = []
	values = []
	tree.append(x_init[:2]) # Append initial node
	orientation_tree.append(x_init[2]) # Append initial node
	parent.append(0)
	values.append(0)
	node_value = 0
	iteration = 0	
	k = 0
	graph.position_boundary = args.position_boundary
	graph.orientation_boundary = args.orientation_boundary
	graph.show_tree_building = args.show_tree_building
	environment.make_obstacles()

	if not args.show_tree_building:
		print('[INFO] Simulating the system... Please, wait.')

	while run and k < args.nodes:		
		# Make sure the loop runs at 60 FPS
		clock.tick(environment.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		obstacles = environment.draw_obstacles() if args.obstacles else []
		graph.draw_initial_node(map=environment.map)
		graph.draw_goal_node(map=environment.map)
		graph.draw_initial_robot_configuration(robot_img=robot.init, environment=environment)
		graph.draw_goal_robot_configuration(robot_img=robot.goal, environment=environment)

		if args.show_tree_building:
			environment.draw_trajectory_trail()

		if not graph.is_goal_found:
			if graph.is_forward_simulation_finished:
				x_rand = graph.generate_random_node(bias=args.bias_percentage)
				random_rect = graph.draw_random_robot_configuration(robot_img=robot.img,
					environment=environment)
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

					k += 1 # One more configuration sampled

				if graph.is_goal_found:
					graph.parent = parent
					graph.tree = tree
					graph.orientation_tree = orientation_tree
					graph.path_to_goal()
					graph.get_path_coordinates()

		if args.show_new_configurations and args.show_tree_building:
			graph.draw_new_robot_configuration(robot_img=robot.img, environment=environment)
			graph.draw_new_node(map=environment.map)

		if graph.is_goal_found:
			if not args.show_tree_building:
				environment.show_screen() # Show screen back
				graph.draw_initial_robot_configuration(robot_img=robot.init, environment=environment)
				graph.draw_goal_robot_configuration(robot_img=robot.goal, environment=environment)
				environment.draw_trajectory_trail()
				pygame.display.update()
				pygame.time.delay(3000)

			if args.path_to_goal:
				graph.draw_path_to_goal(map=environment.map)
			if args.move_robot:
				graph.draw_trajectory(robot=robot, environment=environment)
			if args.show_interpolation:
				graph.draw_interpolation(robot=robot, environment=environment)

		if rand_collision_free and args.show_random_configurations and args.show_tree_building:
			random_rect = graph.draw_random_robot_configuration(robot_img=robot.img,
				environment=environment)
			graph.draw_random_node(map=environment.map)

		iteration += 1
		robot.dt += args.delta

		if not graph.is_goal_found:
			pygame.display.update()
			environment.map.fill(environment.WHITE)

	pygame.quit()
	sys.exit()

if __name__ == '__main__':
	main()
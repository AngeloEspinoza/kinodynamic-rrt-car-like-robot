import pygame
import environment
import robot
import RRT

# Constants
ROBOT_IMG_PATH = r'robot.png'

# Initialization 
pygame.init()

# Initial and final position of the robots
x_init = 50, 50
x_goal = 540, 380

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
	tree.append(x_init) # Append initial node
	parent.append(0)
	values.append(0)
	obstacles = environment.draw_obstacles()
	node_value = 0
	iteration = 0
	print(obstacles)
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

		if not graph.is_goal_found:
			if not graph.is_forward_simulation_finished:
				x_rand = graph.generate_random_node()
				# graph.draw_random_node(node=x_rand, map=environment.map)			

			x_new = graph.new_state(x_rand, robot, event,
				environment, obstacles)	

			if graph.collision_free:
				# Append in function of the simulations done
				for _ in range(graph.max_simulations):
					values.append(node_value)
				parent = graph.generate_parents(values, parent)
				node_value += 1
				graph.collision_free = False
				
		iteration += 1
		pygame.display.update()

if __name__ == '__main__':
	main()
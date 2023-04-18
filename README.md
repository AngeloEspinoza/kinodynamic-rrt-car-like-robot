# Randomized Kinodynamic RRT for a Car-like Robot

<p align="center">
  <img src="https://user-images.githubusercontent.com/40195016/232264148-40b57472-fc54-4024-9c9e-ebc3cf4324b2.gif" alt="drawing" width="450"/>
</p>

## Description
A 2D simulation in the framework Pygame of the paper [Randomized Kinodynamic Planning](https://skat.ihmc.us/rid=1K7WQT337-XQJP8C-1YHM/Randomized%20Kinodynamic%20Planning.pdf)
for a car-like robot. The environment has 2 non-convex obstacles that can be used. It shows different elements of the tree while building it, like the $\mathbf{x_{\mathit{new}}}$ or $\mathbf{x_{\mathit{rand}}}$ states.
Also, the forward simulation time step $\Delta t$ and the maximum allowed nodes variables can be adjusted. 

### System Model
<p align="center">
  <img src="https://user-images.githubusercontent.com/40195016/232273221-3dc8757b-e20c-40c6-9371-56cb3ec64f5a.png" alt="drawing" width="350"/>
</p>

<p align = "center">
  Fig. 1. The car-like robot model.
</p>

As the state transition equation of car-like robot can be described as

$$
  \begin{aligned}
    \dot{x} &= u_1\cos{\theta}, \\
    \dot{y} &= u_1\sin{\theta}, \\
    \dot{\theta} &= u_2,
  \end{aligned}
$$

where $u_1$ and $u_2$ are the control inputs of the system, i.e, the car-like robot. 

### Numerical Method to Integrate the Transition Equation
In order to simulate the robot movements, we need to, somehow, integrate the transition equation. In this simulation, we use the [Newton's Method](https://en.wikipedia.org/wiki/Newton%27s_method).
By integrating with this method, we get both the position and orientation such that the continuous transition equations turns into a discrete equation as

$$
\begin{aligned}
  x_{k+1} &= x_k + u_1\cos{\theta}\Delta t, \\
  y_{k+1} &= y_k + u_1\sin{\theta}\Delta t, \\
  \theta_{k+1} &= \theta_k + u_2\Delta t. \\  
\end{aligned}
$$

Note here that the greater the $\Delta t$ is, the less time in one time step will be simulated forward in time.   

### Metric to Measure the Distance Between States
Given that configuration space (C-space) of the car-like robot is $\mathcal{C} = \mathbb{R}^2 \times S^1$, the way to measure distances between any two configurations, 
and the one used fort his project is

$$
d(\mathbf{x}_1, \mathbf{x}_2) = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2 + \alpha^2},
$$

where $\alpha = \min\left\lbrace|\theta_1 - \theta_2|, 2\pi - |\theta_1 - \theta_2|\right\rbrace$.

## Constraints
Since the kinodynamic RRT (Rapidly-exploring Random Tree) is a [sampling-based algorithm](http://lavalle.pl/planning/node181.html) the probability that the car-like
robot reaches the exact $\mathbf{x}_{goal}$ state is very low. That is why although in the first GIF the car-like robot gets closer and closer to the goal it never 
really reaches it.  To adress this, an offset boundary has been set for the position and orientation to make the searching of the final path easier, they can be 
used with the command line and are named as ```--position_boundary``` and ```--orientation_boundary```. Just be wise choosing these offsets as it may affect the 
performance of the simulation, and therefore make it harder to find a solution to the motion planning problem. The greater the position offset and the closer to $\pi$
the orientation is, the faster a solution will be found.

## Usage

```
usage: kinodynamic_RRT.py [-h] [-o | --obstacles | --no-obstacles] [-n] [-dt] [-init  [...]]
                          [-goal  [...]]
                          [-src | --show_random_configurations | --no-show_random_configurations]
                          [-snc | --show_new_configurations | --no-show_new_configurations] [-bp]
                          [-ptg | --path_to_goal | --no-path_to_goal]
                          [-si | --show_interpolation | --no-show_interpolation]
                          [-mr | --move_robot | --no-move_robot] [-pb] [-ob]
                          [-stb | --show_tree_building | --no-show_tree_building]

Implements the kinodynamic RRT algorithm for a car-like robot.

options:
  -h, --help            show this help message and exit
  -o, --obstacles, --no-obstacles
                        Obstacles on the map
  -n , --nodes          Maximum number of nodes
  -dt , --delta         Fixed time interval
  -init  [ ...], --x_init  [ ...]
                        Initial node configuration in X, Y, and theta in pixels and radians,
                        respectively
  -goal  [ ...], --x_goal  [ ...]
                        Goal node configuration in X, Y, and theta in pixels and radians,
                        respectively
  -src, --show_random_configurations, --no-show_random_configurations
                        Show random configurations on screen
  -snc, --show_new_configurations, --no-show_new_configurations
                        Show new configurations on screen
  -bp , --bias_percentage
                        Amount of bias the RRT from 1 to 100
  -ptg, --path_to_goal, --no-path_to_goal
                        Draws the milestones from path to goal (default: False)
  -si, --show_interpolation, --no-show_interpolation
                        Draws the configurations needed to reach the goal (default: True)
  -mr, --move_robot, --no-move_robot
                        Shows the movements of the robot from start to end (default: True)
  -pb , --position_boundary
                        Allowed position region of the pixels for the robot to reach the goal
  -ob , --orientation_boundary
                        Allowed orientation region of the angle for the robot to reach the goal
  -stb, --show_tree_building, --no-show_tree_building
                        Shows the simulation forward in time (default: True)
```

## Examples
Generate obstacles in the map and place an initial configuration to be $\mathbf{x}_{init} = (50, 100, 0.15)$

 ```python3 kinodynamic_RRT.py --obstacles --x_init 50, 100, 0.15```

Set a maximum orientation offset and position for the robot to reach the goal

 ```python3 kinodynamic_RRT.py --position_boundary 20 --orientation_boundary 0.2```

## License 
MIT License

Copyright (c) [2023] [Angelo Espinoza]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

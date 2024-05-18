# Optimal-Route-Planning

This project requires MATLAB and the YALMIP toolbox. Ensure you have the following installed:
- MATLAB R2020a or later
- YALMIP (available at: https://github.com/yalmip/YALMIP)

## Functions

### main.m
Defines the cities and calls the necessary functions to solve the ETSP with obstacles, using A* for obtaining the distance matrix between each pair of points (d) while avoiding obstacles and either YALMIP optimization or a greedy heuristic to solve the ETSP given d.
You can uncomment the set of cities you want to use.

### ProblemDefinition.m
Defines the problem grid and obstacles.

### Astar.m
Computes the distance matrix between cities using A* algorithm to avoid obstacles.

### ETSP_Opt_Yalmip.m
Solves the ETSP as an optimizaion challenge using YALMIP toolbox.

### ETSP_Greedy.m
Solves the ETSP using the Greedy heuristic.

### Trajectory_plot.m
Plots the trajectory of the solution on a grid with obstacles.

### DubinsCircles.m
Implements DubinsCircles to smooth the trajectories. 
Only implemented for the 6 cities example case

### path_cost.m
Calculates the cost of a given path for the Greedy heuristic.

### path_greedy.m
Generates a path using a greedy heuristic.


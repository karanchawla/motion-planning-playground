# Algorithms for Classical Planning

Last Edited: May 14, 2018 4:34 PM
Tags: motion planning,classical

# What is a Plan?

In the order of increasing generality: 

**Sequence** of instantiated actions. 

**Partial** **order** of instantiated actions. 

**Set** of instantiated actions. 

**Policy**: 

Mapping from states to actions. 

## Planning algorithms

Progression: forward state space search

Regression: backward state space search 

## Properties of planning algorithms

1. Soundness: if all solns are legal paths 
2. Completeness: if a solution can be found if one exists
3. Optimality

## Forward Expansion

1. State reachability – “until” goal
  1. Can find all goals reachable from initial state
  2. Exponential in time and memory

## GraphPlan

1. Preprocessing before searching 
2. Forward search combined with backward search 
3. Construct a planning graph to reveal constraints 
4. Two stages: 
  1. Extend 
  2. Search 

5. Finds a plan or proves that there's no plan with fewer time steps

## Planning as a graph search problem

1. Construct a graph representing the planning problem 
2. Search the graph for a hopefully close to optimal path

These are often interleaved.  

Graph search using an implicit graph.

1. Instantiate start state
2. Start searching with the start state using functions 
  1. Getsuccessors 
  2. ComputeEdgeCost 
- Configuration spaces are useful for planning in 2D space, expand obstacles by radius R for an object with base radius R and complete planning for a point. Only disadvantage being configuration space needs to be updated each time map is updated.
- That's why configuration spaces aren't usually constructed and obstacle checking is done on demand for higher dimensionality planning problems.

## Graph construction

1. **Cell decomposition** - overly expensive for non trivial environments and/or over 2D
  1. X Connected grids 
  2. Lattice based graphs -

  Pivtoraiko & Kelly ’05 

  For partially blocked cells - make the grid really fine. Expensive especially in high dimensional spaces. 

2. **Skeletonization** of the environment 

1. Visibility graphs - based on idea that the shortest path consists of obstacle-free straight line segments connecting all obstacle vertices and start and goal

  Advantages: Independent of the size of the env

  Disadvantages: 

  Path is too close to the obstacles

  Hard to deal with non uniform cost function 

  Hard to deal with non polygon obstacles

2. **Voronoi** **diagrams** - based on the idea of maximizing clearance than minimizing travel distance 
  1. Compute voronoi diagram in O(n log n)
  2. Add a shortest path segment to the nearest voronoi segment
  3. Compute shortest distance on the graph

  Advantages: 

  - Tends to stay away from obstacles
  - Independent of the size of the environment

  Disadvantages: 

  - Can result in highly suboptimal paths
3. **Probabilistic roadmaps** - Generate a sparse representation of the free space
  1. Search the generated representation for a solution
  2. Can interleave the above two
  3. Generally probabilistically complete. 
  4. In many domains is much faster and requires much less memory 
  5. Well suited for high dimensional planning 

## Planning, Execution and Learning - A* and Weighted A* search

Once a graph is constructed we need to search it for the least cost path. 

## Heuristic Function and A*

1. Computes optimal g values for relevant states at any time, where g is the cost of a shortest path from s_start to s found so far. 
2. Heuristic function must be admissible, i.e. h(s) ≤ c*(s,s_goal) ← min. cost from s to s_goal 
3. Heuristic function must satisfy triangle inequality - h(s) ≤ c(s, succ(s)) + h(succ(s))
4. **Weighted A*** expands states in the order of f = g + e*h
5. Weighted A* trades off optimality for speed. It's epsilon-optimal i.e. the cost(solution) ≤ epsilon * cost(optimal solution)
6. Refer to domain specific research papers to which focus on developing the right heuristic functions which shallow local minima
7. Computing policy with backward A* search — Run Backward A* search until all states of interest have been expanded
8. Memory issues of A* can be solved using **Iterative Deepening A***
# Online Safe Trajectory Generation For Quadrotors
Using Fast Marching Method and Bernstein Basis Polynomial

Last Edited: May 27, 2018 8:31 PM

Paper: **Online Safe Trajectory Generation For Quadrotors Using Fast Marching Method and Bernstein Basis Polynomial**
*Fei Gao, William Wu, Yi Lin and Shaojie Shen*

Main contributions: 

1. Fast marching based method applied on a Euclidean signed distance field based velocity field, for searching a time indexed path which provides reasonable time allocation for trajectory optimization. 
2. A trajectory optimization framework for generating trajectories guaranteed to be safe, smooth and dynamically feasible using Bernstein basis. 
3. Real time implementation of the solution

**Motion planning = front end feasible path finding + back end trajectory optimization** 

Front end path finding can be done in the following ways: 

1. Sampling based - RRT, RRT*, PRM*, RRG etc. 
2. Search based - these discretize the configuration space and convert the path finding problem to a graph search problem. Recent methods include JPS, ARA*, hybrid A* etc. 

Fast Marching Method: 

1. Calculates the time when a wavefront first arrives at a point from the source by assuming the wavefront propagates in the direction normal to speed. 
2. Define velocity map for the robot. After simulating a wave expanded from the start point, arrival time to each point is calculated on the map and gradient descent along the descent direction of the arrival time is used to obtain a path from start to the end with minimal time. 
3. Has no local minima, is complete and consistent. The path searched by fast marching method is naturally parametrized to t.

Fast Marching in Distance Field: 

1. Finding path with properly allocated times, reasonable velocity field needs to be generated. 
2. Euclidean Signed Distance Field is the ideal reference for designing velocity field map - shifted hyperbolic tangent. 
3. Flight corridor generation: Once the time indexed minimal arrival path is obtained, a free space corridor is extracted from the environment for back end optimization.  

![](https://www.notion.so/file/https%3A%2F%2Fs3-us-west-2.amazonaws.com%2Fsecure.notion-static.com%2F7c03368f-0497-4046-8c88-835842d15008%2FScreenShot2018-05-27at8.18.46PM.png)

**Safe and Dynamically feasible trajectory generation**

1. Bernstein Basis Piecewise Trajectory formulation - has some nice properties we want from our trajectory 
  1. Endpoint interpolation property 
  2. Fixed interval property 
  3. Convex hull property 
  4. Hodograph property: Derivative is called a hodograph and is a bezier curve as well. 

  Cost function:

  $$J = \sum \int_0^T \frac{d^k f_\mu}{d t^k} dt$$

  $$p^TQ_o p. p$$

  Q_o is the Hessian of the objective function.

  **Enforcing Constraints**

  1. Waypoints constraints - equality constraints on corresponding control points
  2. Continuity constraints - equality constraints on corresponding control points
  3. Safety constraints - due to the convex hull property of the bezier curve, trajectory is confined in the convex hull of the control points. Safety constraints are applied by adding boundary limit on the control points. 
  4. Dynamic feasibility constraints - Constraints are added on the derivatives of the curve in each dimension
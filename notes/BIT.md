- In a Random Geometric Graph, the connection between the states is defined their relative geometric position in space.
- 2 types: k nearest where RGGs have edges to a specific number of each state's nearest neighbors or to all neighbors within a specific distance.
- Sampling based planners can be viewed as constructing an implicit RGG and an explicit spanning tree in the free space of the planning problem.
- Pseudocode:
```
      Initialize start as a vertex and goal in samples
      Initialize the vertex and edge queues to empty
      Repeat
      	If edge queue and vertex queue are empty:
      		Prune the graph
      		Resample
      		Add vertices to the vertex queue
      	Expand best vertex until there is a vertex in the vertex queue better than an edge in the edge queue
      	Get the best edge from the edge queue
      	If it can improve the solution:
      		Add the edge
      	If it cannot:
      		Clear the queues
      Run for n iterations
 ```

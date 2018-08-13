# A Star based grid motion planning
# Author: Karan Chawla @thekaranchawla
# Ref: See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

class Node:
	def __init__(self, x, y, cost, parent):
		self.x = x
		self.y = y
		self.cost = cost 
		self.parent = parent 

def AStarPlanner(start, goal, obstacles, res, robot_radius):
	n_start = Node(round(start[0]/res), round(start[1]/res), 0, -1)
	n_goal = Node(round(goal[0]/res), round(goal[1]/res), float("inf"), -1)

	ox = [iox/res for iox in obstacles[0]]
	oy = [ioy/res for ioy in obstacles[1]]

	obstacle_map, min_x, min_y, max_x, max_y, xw, yw = calculate_obstacle_map(ox, oy, res, r)

	motion = get_motion_model()

	openset, closedset = dict(), dict()
	openset[calc_index(nstart, xw, minx, miny)] = n_start

	while True: 
		c_id = min(openset, key=lambda o:openset[o].cost + heuristic_cost(ngoal, openset[o]))
		current = openset[c_id]
		# show graph
        if show_animation:
            plt.plot(current.x * res, current.y * res, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)
        if current.x == goal.x and current.y == goal.y: 
        	print("Goal Found")
        	n_goal.cost = current.cost
        	n_goal.parent = current 
        	break 

        # Remove the item from the open set 
        del openset[c_id]
        # Add it to the closed set 
        closedset[c_id] = current 

       	for i in range(len(motion)):
       		# compute different nodes we can reach from the current position
       		node = Node(current.x + motion[i][0], current.y + motion[i][1], current.cost + motion[i][2], c_id)
       		# compute the node id for this node
       		node_id = calc_index(node, xw, minx, miny)

       		# if this node has already been added to closed set cont.
       		if node_id in closedset:
       			continue

       		# if node position is obstacle free
       		if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            # if node is not in open set — add it
            if node_id not in openset:
                openset[node_id] = node  # Discover a new node

            # compute temp cost to the node from the current node
            tcost = current.cost + heuristic_cost(current, node)

            # if this is greater than the current cost we don't care about
            # this node
            if tcost >= node.cost: 
            	continue 

            # else add it to the openset
            node.cost = tcost
            openset[node_id] = node 

    # compute final path to goal 
    rx, ry = calculate_final_path(ngoal, closedset, res)
    
    return rx, ry


def calculate_obstacle_map(ox, oy, res, r):

	minx = min(round(ox))
	miny = min(round(oy))
	maxx = max(round(ox))
	maxy = max(round(oy))

	xwidth = round(maxx - minx)
	ywidth = round(maxy - miny)

	# generate obstacle map 
	# If the robot can go the position x and y the obstacle map stores false
	# at that grid point
	obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
	for i in range(xwidth):
		x = i + minx 
		for j in range(ywidth):
			y = j + miny 
			for k, l in zip(ox, oy):
				d = math.sqrt((k-x)**2 + (l-y)**2)
				if d <= r/res:
					# This results in a collision so we save true
					obmap[k][l] = True 
					break 

	return obmap, minx, miny, maxx, maxy, xwidth, ywidth


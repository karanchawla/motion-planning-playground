# A Star based grid motion planning
# Author: Karan Chawla @thekaranchawla
# Ref: See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

import matplotlib.pyplot as plt 
import numpy as np 
import math

show_animation = True

class Node:
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


def AStarPlanner(start, goal, obstacles, res, robot_radius):
    n_start = Node(round(start[0] / res), round(start[1] / res), 0, -1)
    n_goal = Node(round(goal[0] / res), round(goal[1] / res), float("inf"), -1)

    ox = [iox / res for iox in obstacles[0]]
    oy = [ioy / res for ioy in obstacles[1]]

    obstacle_map, min_x, min_y, max_x, max_y, xw, yw = calculate_obstacle_map(
        ox, oy, res, res)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(n_start, xw, min_x, min_y)] = n_start

    while True:
        c_id = min(
            openset, key=lambda o: openset[o].cost + heuristic_cost(n_goal, openset[o], 1.0))

        current = openset[c_id]
        # show graph
        if show_animation:
            plt.plot(current.x * res, current.y * res, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.0001)
        if current.x == n_goal.x and current.y == n_goal.y:
            print("Goal Found")
            n_goal.cost = current.cost
            n_goal.parent = current.parent
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        for i in range(len(motion)):
            # compute different nodes we can reach from the current position
            node = Node(current.x + motion[i][0], current.y +
                        motion[i][1], current.cost + motion[i][2], c_id)
            # compute the node id for this node
            node_id = calc_index(node, xw, min_x, min_y)

            # if this node has already been added to closed set cont.
            if node_id in closedset:
                continue

            # if node position is obstacle free
            if not verify_node(node, obstacle_map, min_x, min_y, max_x, max_y):
                continue

            # if node is not in open set — add it
            if node_id not in openset:
                openset[node_id] = node  # Discover a new node

            # compute temp cost to the node from the current node
            tcost = current.cost + heuristic_cost(current, node, 1.0)

            # if this is greater than the current cost we don't care about
            # this node
            if tcost >= node.cost: 
                continue 

            # else add it to the openset
            node.cost = tcost
            openset[node_id] = node 

    # compute final path to goal 
    rx, ry = calculate_final_path(n_goal, closedset, res)

    return rx, ry

def calc_index(node, xw, minx, miny):
    return (node.y - miny) * xw + (node.x - minx)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion

def verify_node(node, obmap, minx, miny, maxx, maxy):
    if node.x < minx or node.x > maxx:
        return False 

    if node.y < miny or node.y > maxy:
        return False 

    if obmap[node.x][node.y]:
        return False 

    return True 

def heuristic_cost(current, node, weight):
    dx = abs(node.x - current.x)
    dy = abs(node.y - current.y)
    return weight * (dx + dy)

def calculate_final_path(ngoal, closedset, res):
    rx, ry = [ngoal.x], [ngoal.y]
    parent = ngoal.parent
    while parent!= -1:
        node = closedset[parent]
        rx.append(node.x)
        ry.append(node.y)
        parent = node.parent

    return rx, ry 

def calculate_obstacle_map(ox, oy, res, r):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))

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
                    obmap[i][j] = True 
                    break 

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth

def main():
    print(__file__ + " start!!")

    # start and goal position
    start = [10.0, 10.0]
    goal = [50.0, 50.0]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

    ox, oy = [], []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "xr")
        plt.plot(goal[0], goal[1], "xb")
        plt.grid(True)
        plt.axis("equal")

    obstacles = [ox, oy]
    rx, ry = AStarPlanner(start, goal, obstacles, grid_size, robot_size)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()


import random
import numpy as np 
import math 
import copy 
import matplotlib.pyplot as plt

show_animation = True 

class RRTFamilyPlanners():

	def __init__(self, start, goal, obstacleList, randArea, expandDis=0.2, goalSampleRate=10, maxIter=200):

		self.start = Node(start[0], start[1])
		self.goal = Node(goal[0], goal[1])
		self.minrand = randArea[0]
		self.maxrand = randArea[1]
		self.expandDis = expandDis
		self.goalSampleRate = goalSampleRate
		self.maxIter = maxIter
		self.obstacleList = obstacleList

	def RRTSearch(self, animation=True):
		self.nodeList = [self.start]
		while True: 
			# get random point in the free space
			rnd = self.sampleFreeSpace()
			# find closest node in the tree
			nind  = self.getNearestListIndex(self.nodeList, rnd)
			nearestNode = self.nodeList[nind]
			theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
			# compute the position of the new node
			newNode = self.getNewNode(theta, nind, nearestNode)
			# collision check
			if not self.__CollisionCheck(newNode, self.obstacleList):
				continue 
			# if collision doesn't happen in extending the nearest node to the new node
			# add it to the tree
			self.nodeList.append(newNode)

			#check if we reached the goal 
			if self.isNearGoal(newNode):
				break

			if animation:
				self.drawGraph(rnd)

		# compute the path 
		path = [[self.goal.x, self.goal.y]]
		lastIndex = len(self.nodeList) -1 
		while self.nodeList[lastIndex].parent is not None: 
			node = self.nodeList[lastIndex]
			path.append([node.x, node.y])
			lastIndex = node.parent
		path.append([self.start.x, self.start.y])

		return path	

	def RRTStarSearch(self, animation=True):
		self.nodeList = [self.start]
		while True:
			rnd = self.sampleFreeSpace()
			nind = self.getNearestListIndex(self.nodeList, rnd)
			nearestNode = self.nodeList[nind]
			# steer 
			theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
			newNode = self.getNewNode(theta, nind, nearestNode)

			if self.__CollisionCheck(newNode, self.obstacleList):
				nearinds = self.findNearNodes(newNode)
				newNode = self.chooseParent(newNode, nearinds)
				self.nodeList.append(newNode)
				self.rewire(newNode, nearinds)

			if animation:
				self.drawGraph(rnd)

			if self.isNearGoal(newNode):
				break

		# get path 

		# lastIndex = self.getBestLastIndex()
		# if lastIndex is None: 
			# return None

		# path = self.getFinalCourse(lastIndex)
		path = [[self.goal.x, self.goal.y]]
		lastIndex = len(self.nodeList) -1 
		while self.nodeList[lastIndex].parent is not None: 
			node = self.nodeList[lastIndex]
			path.append([node.x, node.y])
			lastIndex = node.parent
		path.append([self.start.x, self.start.y])

		return path	

	def getFinalCourse(self, lastIndex):
		path = [[self.goal.x, self.goal.y]]
		while self.nodeList[lastIndex].parent is not None:
			node = self.nodeList[lastIndex]
			path.append([node.x, node.y])
			lastIndex = node.parent
		path.append([self.start.x, self.start.y])
		return path

	def getBestLastIndex(self):
		disgList = [self.calcDistToGoal(node.x, node.y) 
					for node in self.nodeList]
		goalInds = [disgList.index(i) for i in disgList if i <= self.expandDis]

		if len(goalInds) == 0:
			return None 

		minCost = min([self.nodeList[i].cost for i in goalInds])
		for i in goalInds:
			if self.nodeList[i].cost == minCost:
				return i

		return None 

	def calcDistToGoal(self, x, y):
		return np.linalg.norm([x - self.goal.x, y - self.goal.y])


	def getNewNode(self, theta, nind, nearestNode):
		newNode = copy.deepcopy(nearestNode)

		newNode.x += self.expandDis * math.cos(theta)
		newNode.y += self.expandDis * math.sin(theta)
		newNode.parent = nind 

		return newNode

	def sampleFreeSpace(self):
		if random.randint(0,100) > self.goalSampleRate:
				rnd = [random.uniform(self.minrand, self.maxrand),
					   random.uniform(self.minrand, self.maxrand)]
		else:
			rnd = [self.goal.x, self.goal.y]
		return rnd

	def getNearestListIndex(self, nodes, rnd):
		dList = [(node.x - rnd[0])**2 + 
				 (node.y - rnd[1])**2 for node in nodes]
		minIndex = dList.index(min(dList))
		return minIndex

	def InformedRRTStarSearch(self, animation=True):

		self.nodeList = [self.start]
		# max length we expect to find in our 'informed' sample space, starts as infinite
		cBest = float('inf')
		pathLen = float('inf')
		treeSize = 0
		pathSize = 0
		solutionSet = set()
		path = None

		# Computing the sampling space 
		cMin = math.sqrt(pow(self.start.x - self.goal.x, 2) + pow(self.start.y - self.goal.y, 2))
		xCenter = np.matrix([[(self.start.x + self.goal.x) / 2.0], [(self.start.y + self.goal.y) / 2.0], [0]])
		a1 = np.matrix([[(self.goal.x - self.start.x) / cMin], [(self.goal.y - self.start.y) / cMin], [0]])
		id1_t = np.matrix([1.0, 0.0, 0.0]) # first column of idenity matrix transposed
		M = np.dot(a1 , id1_t)
		U, S, Vh = np.linalg.svd(M, 1, 1)
		C = np.dot(np.dot(U, np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)
		for i in range(self.maxIter):
			# Sample space is defined by cBest 
			# cMin is the minimum distance between the start point and the goal 
			# xCenter is the midpoint between the start and the goal 
			# cBest changes when a new path is found 

			randomNode = self.sample(cBest, cMin, xCenter, C)
			nearestNode = self.findNearestPoint(randomNode)
			newNode = self.steer(nearestNode, randomNode)

			if animation:
				rnd = [randomNode.x, randomNode.y]
				self.drawGraph(rnd)

			if self.check_collision_extend(nearestNode, newNode):
				# nearestSet = self.findNearestSet(newNode)
				nearInds = self.findNearNodes(newNode)
				# minNode = self.findMinPoint(nearestSet, nearestNode, newNode)
				newNode = self.chooseParent(newNode, nearInds)

				self.nodeList.append(newNode)
				self.rewire(newNode, nearInds)

				if self.isNearGoal(newNode):
					solutionSet.add(newNode)
					tempPath, tempPathLen = self.findPath(newNode)
					if tempPathLen < pathLen:
						path = tempPath
						cBest = tempPathLen
			
		return path

	def findPath(self, goalNode):
		goalInd = self.nodeList.index(goalNode)
		path = [[goalNode.x, goalNode.y]]
		pathLen = 0
		while self.nodeList[goalNode].parent is not None:
			node = self.nodeList[goalInd]
			path.append([node.x, node.y])
			goalInd = node.parent
			pathLen += self.lineCost(node, self.nodeList[goalInd])
		path.append([self.start.x, self.start.y])
		return path, pathLen

	def rewire(self, newNode, nearInds):
		nnode = len(self.nodeList)
		for i in nearInds:
			nearNode = self.nodeList[i]

			d = math.sqrt((nearNode.x - newNode.x)**2 +
						  (nearNode.y - newNode.y)**2)
			scost = newNode.cost + d
			if nearNode.cost > scost:
				if self.check_collision_extend(nearNode, newNode):
					nearNode.parent = nnode - 1
					nearNode.cost = scost


	def chooseParent(self, newNode, nearInds):
		if len(nearInds) == 0:
			return newNode 
		dList = []
		for i in nearInds:
			if self.check_collision_extend(self.nodeList[i], newNode):
				dList.append(self.nodeList[i].cost 
					+ self.lineCost(self.nodeList[i], newNode))
			else:
				dList.append(float('inf'))

		minCost = min(dList)
		minInd = nearInds[dList.index(minCost)]

		if minCost == float('inf'):
			print("mincost is inf")
			return newNode

		newNode.cost = minCost
		newNode.parent = minInd

		return newNode

	def findNearNodes(self, newNode):
		numNodes = len(self.nodeList)

		r = 50.0 * math.sqrt((math.log(numNodes)/numNodes))

		dList = [(node.x - newNode.x)**2 +
				 (node.y - newNode.y)**2 for node in self.nodeList]
		nearInds = [dList.index(i) for i in dList if i <= r**2]
		return nearInds

	def isNearGoal(self, node):
		d = self.lineCost(node, self.goal)
		if d < self.expandDis:
			return True 
		return False  

	def sample(self, cMax, cMin, xCenter, C):
		if cMax < float('inf'):
			temp = math.sqrt(cMax**2 - cMin**2) / 2.0
			r = [cMax/2.0, temp, temp]
			L = np.diag(r)
			xBall = self.sampleUnitBall()
			randomNode = no.dot(np.dot(C, L), xBall) + xCenter
			randomNode = Node(randomNode[(0,0)], randomNode[(1,0)])
		else:
			randomNode = self.getCollisionFreeRandomNode()

		return randomNode

	def getCollisionFreeRandomNode(self):
		while True:
			x = random.uniform(self.minrand, self.maxrand)
			y = random.uniform(self.minrand, self.maxrand)
			tempNode = Node(x, y)
			if self.__CollisionCheck(tempNode, self.obstacleList):
				return tempNode


	def sampleUnitBall(self):
		a = random.random()
		b = random.random()

		if b < a:
			a, b = b, a

		sample = (b * math.cos(2 * math.pi * a / b), 
				  b * math.sin(2 * math.pi * a / b))
		return np.array([[sample[0], sample[1], [0]]])

	
	def findNearestPoint(self, randomNode):
		closestPoint = None
		minDist = float('inf')
		for vertex in self.nodeList:
			eucDist = math.sqrt((vertex.x - randomNode.x)**2 + (vertex.y - randomNode.y)**2)
			if eucDist < minDist: 
				minDist = eucDist
				closestPoint = vertex
		return closestPoint

	def steer(self, fromNode, toNode):
		theta = math.atan2(toNode.y - fromNode.y, toNode.x - fromNode.x)
		x = fromNode.x + math.cos(theta) * self.expandDis 
		y = fromNode.y + math.sin(theta) * self.expandDis
		newPoint = Node(x, y)
		return newPoint

	def __CollisionCheck(self, newNode, obstacleList):
		for (ox, oy, size) in obstacleList:
			dx = ox - newNode.x 
			dy = oy - newNode.y 
			d = dx**2 + dy**2
			if d <= size**2:
				return False 
		return True 

	def findNearestSet(self, newNode):
		points = set()
		numNodes = len(self.nodeList)
		ballRadius = 50.0 * math.sqrt((math.log(numNodes) / numNodes))
		for vertex in self.nodeList:
			eucDist = eucDist = math.sqrt((vertex.x - newNode.x)**2 
				+ (vertex.y - newNode.y)**2)
			if eucDist < ballRadius: 
				points.add(vertex)
		return points 

	def findMinPoint(self, nearestSet, nearestNode, newNode):
		minVertex = nearestNode
		minCost = nearestNode.cost + self.lineCost(nearestNode, newNode)
		for vertex in nearestSet:
			if self.check_collision_extend(vertex, newNode):
				tempCost = vertex.cost + self.lineCost(vertex, newNode)
				if tempCost < minCost:
					minVertex = vertex
					minCost = tempCost
		return minVertex

	def lineCost(self, node1, node2):
		return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

	def check_collision_extend(self, node1, node2):

		tempNode = copy.deepcopy(node1)
		d = self.lineCost(node1, node2)
		theta = math.atan2(node2.y - node1.y, node2.x - node1.x)
		for i in range(int(d / self.expandDis)):
			tempNode.x += self.expandDis * math.cos(theta)
			tempNode.y += self.expandDis * math.sin(theta)
			if not self.__CollisionCheck(tempNode, self.obstacleList):
				return False 

		return True 

	def drawGraph(self, rnd=None):

		plt.clf()
		if rnd is not None: 
			plt.plot(rnd[0], rnd[1], "^k")
		for node in self.nodeList:
			if node.parent is not None: 
				if node.x or node.y is not None: 
					plt.plot([node.x, self.nodeList[node.parent].x], [
						  node.y, self.nodeList[node.parent].y], "-g")

		for (ox, oy, size) in self.obstacleList:
			plt.plot(ox, oy, "ok", ms = 30 * size)

		plt.plot(self.start.x, self.start.y, "xr")
		plt.plot(self.goal.x, self.goal.y, "xr")
		plt.axis([-2, 15, -2, 15])
		plt.grid(True)
		plt.pause(0.01)


class Node():

	def __init__(self, x, y):
		self.x = x 
		self.y = y
		self.cost = 0.0 
		self.parent = None 


def main():
    print("Start rrt planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1)
        # (3, 6, 2),
        # (3, 8, 2),
        # (3, 10, 2),
        # (7, 5, 2),
        # (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt = RRTFamilyPlanners(start = [0, 0], goal = [5, 10],
              randArea = [-2, 15], obstacleList = obstacleList)
    path = rrt.RRTStarSearch(animation = show_animation)

    # Draw final path
    if show_animation:
        rrt.drawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()
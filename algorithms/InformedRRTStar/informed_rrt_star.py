import random
import numpy as np 
import math 
import copy 
import matplotlib.pyplot as plt 

show_animation = True 

class InformedRRTStar():

	def __init__(self, start, goal, obstacleList, randArea, expandDis=0.5, goalSampleRate=20, maxIter=100):

		self.start = Node(start[0], start[1])
		self.end = Node(goal[0], goal[1])
		self.minrand = randArea[0]
		self.maxrand = randArea[1]
		self.expandDis = expandDis
		self.goalSampleRate = goalSampleRate
		self.maxIter = maxIter
		self.obstacleList = obstacleList

	def InformedRRTStarSearch(self):

		self.nodeList = [self.start]
		# max length we expect to find in our 'informed' sample space, starts as infinite
		cBest = float('inf')
		pathLen = float('inf')
		treeSize = 0
		pathSize = 0
		solutionSet = set()

		# Computing the sampling space 
		cMin = math.sqrt(pow(self.start[0] - self.goal[0], 2) + pow(self.start[1] - self.goal[1], 2))
		xCenter = np.matrix([[(self.start[0] + self.goal[0]) / 2.0], [(self.start[1] + self.goal[1]) / 2.0], [0]])
		a1 = np.matrix([[(self.goal[0] - self.start[0]) / cMin], [(self.goal[1] - self.start[1]) / cMin], [0]])
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

			if self.isCollisionFree(nearestNode, newNode):
				nearestSet = self.findNearestSet(newNode)
				minNode = self.findMinPoint(nearestSet, nearestNode, newNode)
				self.nodeList.append(newNode)
				self.rewire(nearestSet, minNode, newNode)

				if self.isNearGoal(newNode):
					solutionSet.add(newNode)
					tempPath, tempTreeSize, tempPathSize, tempPathLen = self.findPath(self.start, newNode)
					if tempPathLen < pathLen:
						pathLen = tempPathLen
						path = tempPath
						treeSize = tempTreeSize
						pathSize = tempPathSize
						cBest = tempPathLen

	def sample(self, cMax, cMin, xCenter, C):
		if cMax < float('inf'):
			temp = math.sqrt(cMax**2 - cMin**2) / 2.0
			r = [cMax/2.0, temp, temp]
			L = np.diag(r)
			xBall = self.sampleUnitBall()
			randomNode = no.dot(np.dot(C, L), xBall) + xCenter
			randomNode = (randomNode[(0,0)], randomNode[(1,0)])
		else:
			randomNode = self.getCollisionFreeRandomNode()
		return randomNode

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
		newPoint.x = fromNode.x + math.cos(theta) * self.expandDis 
		newPoint.y = fromNode.y + math.sin(theta) * self.expandDis

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
		ballRadius = 50.0 * math.sqrt((math.log(nnode) / nnode))
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

	def rewire(self, nearestSet, minNode, newNode):
		numNodes = len(self.nodeList)
		for i in range(nearestSet):
			nearNode = self.nodeList[i]

			dx = newNode.x - nearNode.x 
			dy = newNode.y - nearNode.y
			d = math.sqrt(dx**2 + dy**2)

			scost = newNode.cost + d 

			if nearNode.cost > scost:
				if self.check_collision_extend(nearNode, newNode):
					nearNode.parent = nnode - 1 
					nearNode.cost = scost

	def drawGraph(self, rnd=None):

		plt.clf()
		if rnd is not None: 
			plt.plot(rnd[0], rnd[1], "^k")
		for node in nodeList:
			if node.parent is not None: 
				plt.plot([node.x, self.nodeList[node.parent].x], [
						  node.y, self.nodeList[node.parent].y], "-g")

		for (ox, oy, size) in self.obstacleList:
			plt.plot(ox, oy, "ok", ms = 30 * size)

		plt.plot(self.start.x, self.start.y, "xr")
		plt.plot(self.end.x, self.end.y, "xr")
		plt.axis([-2, 15, -2, 15])
		plt.grid(True)
		plt.pause(0.01)


class Node():

	def __init__(self, x, y):
		self.x = x 
		self.y = y
		self.cost = 0.0 
		self.parent = None 
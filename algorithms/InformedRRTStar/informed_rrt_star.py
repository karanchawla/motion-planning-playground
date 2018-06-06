import random
import numpy as np 
import math 

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
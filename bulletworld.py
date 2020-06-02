import pybullet as p
import time
import pybullet_data
import glob
from car import car
import math
import random
import numpy as np

"""
carSettings = p.getBasePositionAndOrientation(self.car.id)
		cubeSettings = p.getBasePositionAndOrientation(self.cube)
		if not([carSettings,cubeSettings] in self.states):
			self.states.append([carSettings, cubeSettings])
return self.states.index([carSettings, cubeSettings])
"""


class bulletworld:
	def __init__(self, argv, test_train):
		self.debug = False
		self.test_train = test_train
		argv_counter = 0
		if len(argv) > 0:
			if argv[argv_counter] == "d":
				self.debug = True
		self.physicsClient = p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.setGravity(0,0,-10, physicsClientId=self.physicsClient)
		self.loadPlane()
		self.loadCar()
		self.stepCounter =0
		#self.cameraTopDown()
		if self.test_train == 0:
			self.createObject()
		if self.debug:
			print("finished constructing bullet world")
		self.states = []

	def __del__(self):
		p.disconnect()
		if self.debug:
			print("finished deconstructing bullet world")

	def loadPlane(self):
		if self.debug:
			print("loading plane")
		#p.loadURDF("plane.urdf")
		planeId = p.loadURDF(glob.glob("planeTrack.urdf")[0])
		if self.test_train < 2:
			texUid=p.loadTexture("track2.png")
		elif self.test_train == 2:
			texUid=p.loadTexture("track.png")
		p.changeVisualShape(planeId, -1, textureUniqueId=texUid)

	def loadCar(self):
		if self.test_train < 2:
			xPossible = [0,1.75,-1.75]
			xStart = random.choice(xPossible)
			if xStart == 1.75 or xStart == -1.75:
				yStart = 0
			else:
				yStart = random.choice([1.75, -1.75])
			self.carPos = [xStart,yStart,0.05]
			self.carOrien = 0
			if xStart == 1.75:
				self.carOrien = math.pi/2
			elif xStart == -1.75:
				self.carOrien = -math.pi/2
			if yStart == 1.75:
				self.carOrien = -math.pi
			elif yStart == -1.75:
				self.carOrien = 0
		elif self.test_train == 2:
			self.carPos = [0,-0.4,0.05]
			self.carOrien = 0
		carStartOrien = [0, 0, self.carOrien]
		self.car = car(self.debug, p, self.carPos, carStartOrien)

	def createObject(self):
		if self.carOrien == -math.pi/2:
			cubePos = [self.carPos[0], self.carPos[1]-1.1,0.1]
		elif self.carOrien == math.pi/2:
			cubePos = [self.carPos[0], self.carPos[1]+1.1,0.1]
		elif self.carOrien == 0:
			cubePos = [self.carPos[0]+1.1, self.carPos[1],0.1]
		else:
			cubePos = [self.carPos[0]-1.1, self.carPos[1],0.1]
		self.cube = p.loadURDF("cube_small.urdf", cubePos)
		
	def cameraFollowCar(self):
		carPos, carOrien = p.getBasePositionAndOrientation(self.car.id)
		p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=20,
							cameraPitch=-30, cameraTargetPosition=carPos)
						
	def cameraTopDown(self):
		p.resetDebugVisualizerCamera(cameraDistance=4, cameraYaw=0,
								cameraPitch=90, cameraTargetPosition=[0,0,0])

	def getDistanceCarCube(self):
		posCar, _ = p.getBasePositionAndOrientation(self.car.id)
		posCube, orCube = p.getBasePositionAndOrientation(self.cube)
		dist = math.sqrt((posCube[0]-posCar[0])**2 + (posCube[1]-posCar[1])**2)
		return dist

	def adjust_cube_position(self):
		posCube, orCube = p.getBasePositionAndOrientation(self.cube)
		dist = self.getDistanceCarCube()
		#print("distance",dist)
		radius = 1.9
		if dist < 0.8:
			if posCube[1] < 0.0: #increase x
				newX = posCube[0] + 0.06
				if newX > radius:
					newX = posCube[0]
					newY = posCube[1]*-1
				else:
					newY = -1*math.sqrt(radius**2-newX**2)
				#print(newX, newY)
			elif posCube[1] > 0.0:
				newX = posCube[0] - 0.06
				if newX < 0-radius:
					newX = posCube[0]
					newY = posCube[1]*-1
				else:
					newY = math.sqrt(radius**2-newX**2)
			p.resetBasePositionAndOrientation(self.cube,[newX,newY,0.1],orCube)
			#print(p.getBasePositionAndOrientation(self.cube))
	
	def reset(self):
		self.stepCounter = 0
		p.removeBody(self.car.id)
		p.removeBody(self.cube)
		self.cameraTopDown()
		del self.car
		del self.cube
		self.loadCar()
		self.createObject()
		dist = self.getDistanceCarCube()
		angle, _ = self.car.getAngleFromFront()
		state = [dist, angle]
		return state
		#return p
	
	def calc_reward(self):
		return (1/self.getDistanceCarCube())*100
	
	def step(self, action):
		self.cameraFollowCar()
		#self.cameraTopDown()
		self.car.giveAction(action)
		for i in range(20):
			p.stepSimulation()
			self.adjust_cube_position()
		reward = self.calc_reward()
		self.adjust_cube_position()
		info = "some kind of information about system?"
		dist = self.getDistanceCarCube()
		angle, done = self.car.getAngleFromFront()
		if done:
			reward = 0
		state = [dist, angle]	
		return state, reward, done, info





	def reset_state(self):
		self.stepCounter = 0
		p.removeBody(self.car.id)
		#p.removeBody(self.cube)
		self.cameraTopDown()
		del self.car
		#del self.cube
		self.loadCar()
		#self.createObject()
		state = self.car.getState()
		state = np.array(state, dtype='uint8')
		return state
		#return p

	def calc_reward_camera(self):
		reward = (self.stepCounter)
		done = False
		pos, _ = p.getBasePositionAndOrientation(self.car.id)
		#if pos[0] == -1*self.carPos[0] and pos[1] == -1*self.carPos[1]:
		#	done = True
		#	reward = 10000
		if self.stepCounter > 50:
			reward = 1500
			done = True
		if self.car.run_topCamera() >= 25: #FIXME define green allowed
			done=True
			reward = -200
		if self.car.getTrackSatAndAngle()[0] == -1:
			done = True
			reward = -200
		return reward, done

	def step_pureCamera(self, action):
		self.stepCounter += 1
		self.cameraFollowCar()
		self.car.giveAction(action)
		for i in range(20):
			p.stepSimulation()
		info = "some information about situation?"
		state = self.car.getState()
		state = np.array(state, dtype='uint8')
		reward, done = self.calc_reward_camera()
		return state, reward, done, info
		
	def run(self):
		if self.debug:
			print("commencing running simulations")
		for i in range (10000):
			self.cameraFollowCar()
			self.car.run()
			p.stepSimulation()
			self.adjust_cube_position()
			time.sleep(1./500)

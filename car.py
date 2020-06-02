import pybullet
import pybullet_data
import math
from image_checker import image_checker
import numpy as np
import PIL.Image
import os
import random

class car:
	def __init__(self, debug, p, carStartPos, carStartOrien):
		self.debug = debug
		self.p = p
		if self.debug:
			print("loading car")
		carStartOrien = self.p.getQuaternionFromEuler(carStartOrien)
		self.id = self.p.loadURDF("racecar/racecar.urdf", carStartPos, carStartOrien)
		if self.debug:
			for i in range(self.p.getNumJoints(self.id)):
				print("car joint: ", self.p.getJointInfo(self.id, i))
		self.inactive_wheels = [2,3]
		self.wheels = [5,7]
		for wheel in self.inactive_wheels:
			p.setJointMotorControl2(self.id, wheel, self.p.VELOCITY_CONTROL,
										targetVelocity=0, force=0)
		self.steering = [4,6]
		"""
		self.targetVelocitySlider=self.p.addUserDebugParameter("wheelVelocity",
																-10, 150, 0)
		self.maxForceSlider=self.p.addUserDebugParameter("maxForce", 0, 150, 0)
		self.steeringSlider=self.p.addUserDebugParameter("steering",-0.5,0.5,0)
		"""
		argv = []
		if self.debug:
			argv.append("d")
		self.img_checker = image_checker(argv, self.p)

	def defineCarCamera(self):
		carpos, carorien = self.p.getBasePositionAndOrientation(self.id)
		carorien = self.p.getEulerFromQuaternion(carorien)
		lengthCar = 0.40
		heightCar = 0.8
		yaw=carorien[2] #orientation around z-axis = theta
		yPos = math.sin(yaw)*lengthCar+carpos[1]
		xPos = math.cos(yaw)*lengthCar+carpos[0]
		cep=[xPos,yPos,heightCar] #cameraEyePosition
		xTar = math.cos(yaw)*lengthCar*2+carpos[0]
		yTar = math.sin(yaw)*lengthCar*2+carpos[1]
		ctp=[xTar, yTar, heightCar/4] #cameraTargetPosition
		cuv=[-(xTar-xPos), -(yTar-yPos), heightCar*2]
		self.viewMatrix=self.p.computeViewMatrix(cameraEyePosition=cep,
												cameraTargetPosition=ctp,
												cameraUpVector=cuv)
		fov=60.0
		aspect=1.0
		nearVal=0.1
		farVal=3.1
		self.projectionMatrix = self.p.computeProjectionMatrixFOV(
												fov=fov,
												aspect=aspect,
												nearVal=nearVal,
												farVal=farVal)

	def defineTopCamera(self):
		carpos, carorien = self.p.getBasePositionAndOrientation(self.id)
		carorien = self.p.getEulerFromQuaternion(carorien)
		ctp=[carpos[0],carpos[1],0] #cameraTargetPosition
		cep=[carpos[0],carpos[1],0] #cameraEyePosition
		cuv=[0,1,0] #cameraUpVector
		self.viewMatrixTop = self.p.computeViewMatrix(cameraEyePosition=cep,
													cameraTargetPosition=ctp,
													cameraUpVector=cuv)
		self.viewMatrixTop = self.p.computeViewMatrixFromYawPitchRoll(
													cameraTargetPosition=ctp,
													distance=-0.75,
													yaw=180+carorien[2]*(180/math.pi),
													pitch=90,
													roll=0,
													upAxisIndex=2)
		fov=45.0
		aspect=1.0
		nearVal=0.1
		farVal=3.1
		self.projectionMatrixTop = self.p.computeProjectionMatrixFOV(
													fov=fov,
													aspect=aspect,
													nearVal=nearVal,
													farVal=farVal)

	def cannotSeeGoal(self):
		self.defineCarCamera()
		width, height, rgbImg, depthImg, segImg = self.p.getCameraImage(
											width=224,
											height=224,
											viewMatrix=self.viewMatrix,
											projectionMatrix=self.projectionMatrix)
		rgbImg = np.reshape(rgbImg, (224,224,4)).astype(np.uint8)
		newimg = PIL.Image.fromarray(rgbImg)#np.reshape(rgbImg, (224,224,4))*1./255.
		newimg.save('seeGoal.png')
		_, done = self.img_checker.findCubeDirection('seeGoal.png')
		#print("cannot see goal: ", done)
		return done
	
	def getTrackSatAndAngle(self):
		self.defineCarCamera()
		width, height, rgbImg, depthImg, segImg = self.p.getCameraImage(
											width=224,
											height=224,
											viewMatrix=self.viewMatrix,
											projectionMatrix=self.projectionMatrix)
		rgbImg = np.reshape(rgbImg, (224,224,4)).astype(np.uint8)
		newimg = PIL.Image.fromarray(rgbImg)#np.reshape(rgbImg, (224,224,4))*1./255.
		filename = 'trackView.png'
		newimg.save(filename)
		#newimg.save('trackBeforeCheck.png')
		xAvg, yAvg, saturation = self.img_checker.seeTrack(filename)
		return xAvg, yAvg, saturation
		
	def getState(self):
		state = self.getTrackSatAndAngle()
		print("getting state", state)
		return state
	
	def getCameraState(self):
		self.defineCarCamera()
		width, height, rgbImg, depthImg, segImg = self.p.getCameraImage(
											width=224,
											height=224,
											viewMatrix=self.viewMatrix,
											projectionMatrix=self.projectionMatrix,
											renderer=self.p.ER_TINY_RENDERER)
		rgbImg = np.reshape(rgbImge, (224, 224,4)).astype(np.uint8)
		newimg = PIL.Imagefromarray(rgbImg)
		filename = "state.png"
		newimg.save(filename)
		#newimg.save('stateBeforeCheck.png')
		state = self.img_checker.track2array(filename)
		os.remove(filename)
		return state
				

	def run_topCamera(self):
		self.defineTopCamera()
		width, height, rgbImg, depthImg, segImg = self.p.getCameraImage(
											width=224,
											height=224,
											viewMatrix=self.viewMatrixTop,
											projectionMatrix=self.projectionMatrixTop,
											renderer=self.p.ER_TINY_RENDERER)
		rgbImg = np.reshape(rgbImg, (224,224,4)).astype(np.uint8)
		newimg = PIL.Image.fromarray(rgbImg)
		#newimg = np.reshape(rgbImg, (224,224,4))*1./255.
		newimg.save('check_green.png')
		newimg.save('greenBeforeCheck.png')
		g_val = self.img_checker.greenPercentage('check_green.png')
		#print("green sat = ", g_val)
		return g_val
	
	def getAngleFromFront(self):
		self.defineCarCamera()
		width, height, rgbImg, depthImg, segImg = self.p.getCameraImage(
											width=224,
											height=224,
											viewMatrix=self.viewMatrix,
											projectionMatrix=self.projectionMatrix)
		rgbImg = np.reshape(rgbImg, (224,224,4)).astype(np.uint8)
		newimg = PIL.Image.fromarray(rgbImg)#np.reshape(rgbImg, (224,224,4))*1./255.
		newimg.save('check_white.png')
		angle, done = self.img_checker.findCubeAngle('check_white.png')
		return angle, done
	
	def run_frontCamera(self):
		self.defineCarCamera()
		width, height, rgbImg, depthImg, segImg = self.p.getCameraImage(
											width=224,
											height=224,
											viewMatrix=self.viewMatrix,
											projectionMatrix=self.projectionMatrix)
		rgbImg = np.reshape(rgbImg, (224,224,4)).astype(np.uint8)
		newimg = PIL.Image.fromarray(rgbImg)#np.reshape(rgbImg, (224,224,4))*1./255.
		newimg.save('check_white.png')
		w_val, _ = self.img_checker.findCubeDirection('check_white.png')
		return w_val

	def giveAction(self, action):
		maxForce = 40
		targetVelocity = 40
		angles = [0.5,0.4,0.3,0.2,0.1,0.0,-0.1,-0.2,-0.3,-0.4,-0.5]
		steeringAngle = angles[action-1]
		for wheel in self.wheels:
			self.p.setJointMotorControl2(self.id, wheel, self.p.VELOCITY_CONTROL,
											targetVelocity=targetVelocity,
											force=maxForce)
		for steer in self.steering:
			self.p.setJointMotorControl2(self.id, steer, self.p.POSITION_CONTROL,
											targetPosition=steeringAngle)


	def run(self):
		"""
		maxForce = self.p.readUserDebugParameter(self.maxForceSlider)
		targetVelocity = self.p.readUserDebugParameter(self.targetVelocitySlider)
		steeringAngle = self.p.readUserDebugParameter(self.steeringSlider)
		"""
		self.run_topCamera()
		direction = self.run_frontCamera()
		maxForce = 50
		targetVelocity = 50
		#print(direction)
		if direction[0] < 1.0: #moving left
		 	steeringAngle = 0.5*direction[0]+0.5
		elif direction [1] < 1.0: #moving right
			steeringAngle = -1*(0.5*direction[0]+0.5)
		else:
			steeringAngle = 0

		for wheel in self.wheels:
			self.p.setJointMotorControl2(self.id, wheel, self.p.VELOCITY_CONTROL,
											targetVelocity=targetVelocity,
											force=maxForce)
		for steer in self.steering:
			self.p.setJointMotorControl2(self.id, steer, self.p.POSITION_CONTROL,
											targetPosition=steeringAngle)


import os
import sys
import cv2
import numpy as np
import math

class image_checker:
	def __init__(self, argv, p):
		self.p = p
		self.debug = False
		argv_count = 0
		if len(argv) > 0:
			if argv[argv_count] == 'd':
				self.debug = True

	#function is based on code found on stackOverflow
	#thread: How to count the pixels of a certain color with OpenCV?
	def greenPercentage(self, img):
		image = cv2.imread(img)
		boundaries = [([0,0,40], [100,255,225])]
		for (lower, upper) in boundaries:
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
			mask = cv2.inRange(image, lower, upper)
			output = cv2.bitwise_and(image, image, mask=mask)
		tot_pixels = image.size
		greenPixels = np.count_nonzero(output)
		print("pix count green", greenPixels)
		#cv2.imwrite("greenAfterCheck.png", output)
		percentage = round(greenPixels*100/tot_pixels,2)
		print("greenpix percentage", percentage)
		os.remove(img)
		return percentage



	def track_mask(self, hsvimg):
		lower = np.array([0,0,0])
		upper = np.array([30,30,30])
		color_mask = cv2.inRange(hsvimg, lower, upper)
		return color_mask > 0, color_mask

	def seeTrack(self, img):
		image = cv2.imread(img, cv2.IMREAD_COLOR)
		blur = cv2.GaussianBlur(image, (3,3), 2)
		hsvblur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		track, mask = self.track_mask(hsvblur)
		#output = cv2.bitwise_and(image, image, mask=mask)
		#cv2.imwrite("trackAfterCheck.png", blur)
		summ = np.array([0,0])
		count = 0.0
		idx = np.argwhere(track)
		#print(summ)
		if idx.any():
			x_avg = np.mean(idx[0, :])
			if len(idx) > 1:
				y_avg = np.mean(idx[1, :])
			else:
				y_avg = 0
		else:
			y_avg = -1
			x_avg = -1
		#point = np.array([summ[0],summ[1]])
		#point = np.array([summ[0], summ[1]-len(blur)/2.0])
		print("see track ", x_avg, y_avg)
		tot_pixels = image.size
		trackPixels = np.count_nonzero(track)
		saturation = round(trackPixels*100/tot_pixels,2)
		os.remove(img)
		print("track staturation: ", saturation)
		return x_avg, y_avg, saturation
		
		
	def track2array(self, img):
		image = cv2.imread(img)
		boundaries = [([0,0,0],[30,30,30])]
		for (lower, upper) in boundaries:
			lower = np.array(lower, dtype="uint8")
			upper = np.array(upper, dtype="uint8")
			mask = cv2.inRange(image, lower, upper)
			output = cv2.bitwise_and(image, image, mask=mask)
		os.remove(img)
		return output

	def idcube(self, hsvblur):
		lower_range = np.array([0,0,60/100*225])
		upper_range = np.array([255, 10/100*255, 255])
		color_mask = cv2.inRange(hsvblur, lower_range, upper_range)
		return color_mask > 0
	
	def findVectorCube(self, cube, blur):
		if not cube.any():
			print("no clube in sight")
			return [1.0,1.0], True
		else:
			summ = np.array([0,0])
			count = 0.0
			for i in range(len(cube)):
				for j in range(len(cube[i])):
					if cube[i,j]==True:
						summ[0]+=i
						summ[1]+=j
						count += 1.0
			summ = (summ/count)
			point = np.array([summ[0], summ[1]-len(blur)/2.0])
			angle = math.atan2(point[0],point[1])
			beta = abs(math.pi/2-angle)/(math.pi/2)
			if angle <= math.pi /2:
				#print("move right")
				return [1.0, 1-beta], False
			else:
				#print("move left")
				return [1-beta, 1.0], False
	
	def findCubeAngle(self, img):
		image = cv2.imread(img)
		blur = cv2.GaussianBlur(image, (3,3),2)
		hsvblur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		avg_saturation = np.mean(hsvblur[:,:,1])
		cube = self.idcube(hsvblur)
		blur[cube] = (255,0,255)
		filename= "angle.png"
		cv2.imwrite(filename, blur)
		if not cube.any():
			print("no clube in sight")
			return 0, True
		else:
			summ = np.array([0,0])
			count = 0.0
			for i in range(len(cube)):
				for j in range(len(cube[i])):
					if cube[i,j]==True:
						summ[0]+=i
						summ[1]+=j
						count += 1.0
			summ = (summ/count)
			point = np.array([summ[0], summ[1]-len(blur)/2.0])
			angle = math.atan2(point[0],point[1])
			return angle, False
	
	
	def findCubeDirection(self, img):
		image = cv2.imread(img)
		blur = cv2.GaussianBlur(image, (3,3),2)
		hsvblur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		avg_saturation = np.mean(hsvblur[:,:,1])
		white_cube = self.idcube(hsvblur)
		blur[white_cube] = (255,0,255)
		filename= "tester.png"
		cv2.imwrite(filename, blur)
		return self.findVectorCube(white_cube, blur)



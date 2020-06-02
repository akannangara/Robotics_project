from __future__ import division

import time
import os
import sys
import ZeroBorg
import importimport threading
import picamera
import picamera.array
import cv2
import numpy as numpy
import math

from ImageCapture import ImageCapture
from StreamProcessor import StreamProcessor

from dqn import Agent

global running
global ZB
global camera
global processor

class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print 'Start the stream using the video port'
        camera.capture_sequence(self.TriggerStream(), format='bgr', use_video_port=True)
        print 'Terminating camera processing...'
        processor.terminated = True
        processor.join()
        print 'Processing terminated.'

    # Stream delegation loop
    def TriggerStream(self):
        global running
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()


class StreamProcessor(threading.Thread):
    def __init__(self):
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.reportTick = 0
        self.start()
        self.begin = 0
        self.last_photo_taken = 0
        self.agent = Agent(gamma=0.99, epsilon=1.0, alpha=0.0005, input_dims=3,
                            n_actions=11, mem_size=1000000, batch_size=2,
                            epsilon_end=0.1)
        filename = "dqn_model.h5"#name of stored model file
        self.agent.load_model(filename)

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()

    #setting driving direction
    def setDriveDirection(self, x_avg, y_avg, saturation)
        state = np.array((x_avg, y_avg, saturation), dtype='uint8')
        action = agent.choose_action(state)
        self.move(action)


    def track_mask(self, hsvblur):
        lower = np.array([0,0,0])
        upper = np.array([30,30,30])#define max black range
        color_mask = cv2.inRange(hsvblur, lower, upper)
        return color_mask > 0

    # Image processing function
    def ProcessImage(self, img):
        image = cv2.imread(img, cv2.IMREAD_COLOR)
        blur = cv2.GaussianBlur(image, (3,3), 2)
        hsvblur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        track = self.track_mask(hsvblur)
        summ = np.array([0,0])
        count = 0.0
        idx = np.argwhere(track)
        if idx.any():
            y_avg = np.mean(idx[0, :])
            x_avg = np.mean(idx[1, :])
        else:
            y_avg = 0
            x_avg = 0
        tot_pixels = image.size
        trackPixels = np.count_nonzero(track)
        saturation = round(trackPixesl*100/tot_pixels, 2)
        self.setDriveDirection(x_avg, y_avg, saturation)



    # Set the motor speed from the motion detection
    def move(self, action):
        global ZB

        angles=[-45,-36,-27,-18,-9,0,9,18,27,36,45]
        steeringAngle = angles[action-1]
        print("steeringAngle: ", steeringAngle, "degrees")
        if steeringAngle == 0:
            driveLeft = 1
            driveRight = 1
        elif steeringAngle < 0:
            driveLeft = 1 - ((abs(steeringAngle)/45)/2)
        else: #steeringAngle > 0
            driveRight = 1 - ((steeringAngle/45)/2)

        ZB.SetMotor1(-driveRight * maxPower) # Rear right
        ZB.SetMotor2(-driveRight * maxPower) # Front right
        ZB.SetMotor3(-driveLeft  * maxPower) # Front left
        ZB.SetMotor4(-driveLeft  * maxPower) # Rear left




##############################################################################################
#################   MAIN RUNNING PR0GRAM    ##################################################
##############################################################################################
running = True
ZB = ZeroBorg.ZeroBorg()
ZB.Init()
if not ZB.foundChip:
    boards = ZeroBorg.ScanForZeroBorg()
    if len(boards) == 0:
        print 'No ZeroBorg found, check you are attached :)'
    else:
        print 'No ZeroBorg at address %02X, but we did find boards:' % (ZB.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I�C address change the setup line so it is correct, e.g.'
        print 'ZB.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
failsafe = failsafe
for i in range(5):
    ZB.SetCommsFailsafe(True)
    failsafe = ZB.GetCommsFailsafe()
    if failsafe:
        break
if not failsafe:
    print ('Board %02X failed to report in failsafe mode!', % (ZB.i2cAddress))
    sys.exit()
ZB.ResetEpo()

voltageIn = 8.4
voltageOut = 6.0

imageWidth = 224
imageHeight = 224
frameRate = 10

if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)

print("setup camera")
camera = picamera.PiCamera()
camera.resolution = (imageWidth, imageHeight)
camera.framerate = frameRate
imageCentreX = imageWidth/2.0
imageCentreY = imageHeight/2.0

print("setup stream processing thread")
processor = StreamProcessor()

print("waiting 2 units")
time.sleep(2)
captureThread = ImageCapture()

try:
    print("press CTRL+C to quit")
    ZB.MotorsOff()
    while running:

        time.sleep(0.1)
    ZB.MotorsOff()
except KeyboardInterrupt:
    print("\n User Shutdown")
    ZB.MotorsOff()
except:
    e = sys.exc_info()[0]
    print("")
    print("\n Unexpected error, shutting down!")
    ZB.MotorsOff
running = False
captureThread.join()
processor.terminated = True
processor.join()
del camera
ZB.SetLed(False)
print("program terminated correctly")
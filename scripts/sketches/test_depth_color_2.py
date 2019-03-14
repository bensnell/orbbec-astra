#!/usr/bin/python

# This script works with the Orbbec Astra series RGBD cameras.
# This script only works with x64 (not x86) architecture.
# This script only works on Windows.


# If the Orbbec drivers haven't been installed, download the Orbbec OpenNI SDK here:
# https://orbbec3d.com/develop
# In this directory, you will find the executable 
# OpenNI_2.3.0.55 > Windows > Astra OpenNI2 Development Instruction(x64)_V1.3 > Driver > SensorDriver_V4.3.0.9.exe
# Run this exe file and follow the installation prompts. 
# When you plug in an orbbec sensor, it should be recognized without any error in the DeviceManager.
# If there is an error, try uninstalling and reinstalling the driver.

# Run setlib_environ.bat in the directory here:
# OpenNI_2.3.0.55 > Windows > Astra OpenNI2 Development Instruction(x64)_V1.3 > OpenNI2 > OpenNI-Windows-x64-2.3.0.55 > Redist

# Ensure that python is installed and that the following python modules
# are installed using pip:
# opencv-python, numpy, primesense

# Make sure to pass the correct path to the openni2 Redist folder. 
# The correct Redist folder should be located in
# OpenNI_2.3.0.55 > Windows > Astra OpenNI2 Development Instruction(x64)_V1.3 > OpenNI2 > OpenNI-Windows-x64-2.3.0.55 > Redist
# This folder should contain 
# OpenNI2
#	Drivers
#		OniFile.dll
#		OniFile.ini
#		orbbec.dll
#		Orbbec.ini
# OpenNI.ini
# OpenNI2.dll
# OpenNI2.jni.dll
# org.openni.jar


# For more information about how to read both the rgb and the depth 
# at once, see this link:
# https://github.com/kanishkaganguly/OpenNIMultiSensorCapture/blob/master/capture_cam.py

# possible pixel formats are listed here:
# https://github.com/tomerfiliba/PrimeSense/blob/master/primesense/openni2.py


# Blob tracking reference:
# https://raw.githubusercontent.com/spmallick/learnopencv/master/BlobDetector/blob.py
# https://www.learnopencv.com/blob-detection-using-opencv-python-c/

# ===================================================
# ================ USER PARAMETERS ==================
# ===================================================

# ======= Streaming Parameters ========
# These three parameters must be set to one of the following 
# tuples: (30, 640, 480), (60, 320, 240)
frameRate = 30
width = 640 		# Width of image
height = 480		# height of image

# ======= Computer Vision Parameters =======
# Intensity Threshold [0, 255]
minIntensity = 200

# Blob Area Thresholds (in pixel area)
minArea = 100
maxArea = width*height

# Circularity Threshold [0, 1]
minCircularity = 0.1


# ======= Noise Reduction Parameters =======
# Easing Parameter



# ===================================================
# ================= IMPLEMENTATION ==================
# ===================================================

# Import modules of interest
import cv2
import numpy as np
from primesense import openni2
from primesense import _openni2 as c_api
import time

def rgb2gray(rgb):
    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    return gray.astype(np.uint8)

# Initialize OpenNI with its libraries
openni2.initialize("Redist/") #The OpenNI2 Redist folder

# Open a device
dev = openni2.Device.open_any()
# Check to make sure it's not None

# Open two streams: one for color and one for depth
depth_stream = dev.create_depth_stream()
depth_stream.start()
depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX = width, resolutionY = height, fps = frameRate))

color_stream = dev.create_color_stream()
color_stream.start()
color_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX = width, resolutionY = height, fps = frameRate))

# Register both streams so they're aligned
dev.set_image_registration_mode(c_api.OniImageRegistrationMode.ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR);
# Wait for these settings to take effect
time.sleep(1)

# Set blob detection parameters
params = cv2.SimpleBlobDetector_Params()
params.minThreshold = minIntensity
params.maxThreshold = 255
params.filterByArea = True
params.minArea = minArea
params.maxArea = maxArea
params.filterByCircularity = True
params.minCircularity = minCircularity
params.maxCircularity = 1.0
params.filterByConvexity = True
params.minConvexity = 0.5 #0.87
params.filterByInertia = True
params.minInertiaRatio = 0.01
# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else : 
	detector = cv2.SimpleBlobDetector_create(params)

# Main loop
lastUpdateTime = time.time()
while True:

	# startTime = time.time()

	# Get the depth pixels
	frame = depth_stream.read_frame()
	frame_data = frame.get_buffer_as_uint16()
	depthPix = np.frombuffer(frame_data, dtype=np.uint16)
	depthPix.shape = (1, height, width)
	depthPix = np.concatenate((depthPix, depthPix, depthPix), axis=0)
	depthPix = np.swapaxes(depthPix, 0, 2)
	depthPix = np.swapaxes(depthPix, 0, 1)
	# This depth is in 100 micrometer units

	# Get the color pixels
	frame = color_stream.read_frame()
	frame_data = frame.get_buffer_as_uint8()
	colorPix = np.frombuffer(frame_data, dtype=np.uint8)
	colorPix.shape = (height, width, 3)
	colorPix = np.flip(colorPix, 2)

	# cv2.imshow("Color Image", colorPix)
	# cv2.imshow("Registered Depth Image", depthPix.astype(np.uint8))
	# cv2.waitKey(int(1000.0/float(frameRate)))

	# stopTime = time.time()
	# print(stopTime - startTime)

	# Do blob detection to get the sphere light
	grayPix = rgb2gray(colorPix)
	# print(grayPix.dtype)
	keypoints = detector.detect(grayPix)
	print(keypoints)
	
	img2 = colorPix.copy()
	for marker in keypoints:
		img2 = cv2.drawMarker(img2, tuple(int(i) for i in marker.pt), color=(0, 0, 255), thickness=5)

	# im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
	cv2.imshow("Keypoints", img2)
	cv2.waitKey(34)


		# Print the number of millimeters
	# print(depthPix[320,240,0]/10.0)




openni2.unload()

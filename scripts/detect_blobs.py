#!/usr/bin/python

# ===================================================
# ================ USER PARAMETERS ==================
# ===================================================

# ======= Debugging ========
# View the camera feed and blob detection
bViewFeed = True

# ======= Streaming Parameters ========
# These three parameters must be set to one of the following 
# tuples: (30, 640, 480), (60, 320, 240)
frameRate = 30		# frames per second
width = 640 		# Width of image
height = 480		# height of image


# ======= Computer Vision Parameters =======
# Intensity Threshold [0, 255]
minIntensity = 200

# Blob Area Thresholds (in pixel area)
minArea = 100
maxArea = width*height

# Circularity Threshold [0, 1]
minCircularity = 0.5

# Depth Threshold (mm)
minDepth = 10
maxDepth = 3000

# Edge pixels that we'll ignore in the color image
edgePixels = 35

# Remove color pixels that don't have any depth associated with them
bZeroMask = True
nErosions = 3

# Camera Exposure
bAutoExposure = False
exposure = 100

# ======= Smoothing Parameters =======
# How much smoothing (and latency) you want
# 0 --> no smoothing, no latency
# 0.9 --> lots of smoothing, lots of latency
easingParam = 0.3

# ======= OSC UDP Parameters =======
ipAddress = '127.0.0.1'
port = 8888
header = "/orbbec"


# ======= OPENNI2 Paths =======
redistPath = "../lib/Redist/"


# ===================================================
# ================= IMPLEMENTATION ==================
# ===================================================

# Import modules of interest
import cv2
import numpy as np
from primesense import openni2
from primesense import _openni2 as c_api
import time
import imutils
from pythonosc import osc_message_builder
from pythonosc import udp_client
import ctypes

def rgb2gray(rgb):
    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    return gray.astype(np.uint8)

# Initialize OpenNI with its libraries
openni2.initialize(redistPath) #The OpenNI2 Redist folder

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
# Set exposure settings
if (color_stream.camera != None):
	color_stream.camera.set_auto_exposure(bAutoExposure)
	color_stream.camera.set_exposure(exposure)

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

def within(val, lo, hi):
	return val >= lo and val <= hi

def minRatio(a, b):
	if a == 0 or b == 0:
		return 0
	elif a < b:
		return float(a)/float(b)
	else:
		return float(b)/float(a)

def contourWithinInertia(ctr, lo, hi):
	rect = cv2.minAreaRect(ctr)
	return within(minRatio(rect[1][0], rect[1][1]), lo, hi)

# Create an OSC client
client = udp_client.SimpleUDPClient(ipAddress, port)

# Main loop
lastUpdateTime = time.time()
bPrevData = False
px = 0
py = 0
pz = 0
while True:

	# Get the depth pixels
	frame = depth_stream.read_frame()
	frame_data = frame.get_buffer_as_uint16()
	depthPix = np.frombuffer(frame_data, dtype=np.uint16)
	depthPix.shape = (height, width)
	# This depth is in 100 micrometer units

	# Cull out areas that are outside of the depth	
	zeroMask = np.ones((height,width))
	if bZeroMask:
		zeroMask = (np.equal(depthPix, 0)).astype(np.uint8)
		erodeKernel = np.ones((5,5),np.uint8)
		zeroMask = cv2.erode(zeroMask, erodeKernel, iterations=nErosions)
		zeroMask = np.subtract(1, zeroMask)

	depthMask = np.invert(np.logical_or(np.logical_and(np.greater(depthPix,0),np.less(depthPix,minDepth*10.0)), np.greater(depthPix,maxDepth*10.0)))
	# optionally remove the outer edge
	edgeMask = np.zeros((height,width))
	cv2.rectangle(edgeMask, (edgePixels, edgePixels), (width-edgePixels,height-edgePixels),(1),-1)
	depthMask = (cv2.blur(depthMask.astype(np.uint8) * edgeMask * zeroMask * 255, (11, 11))).astype(np.float32)/255.0

	# Get the color pixels
	frame = color_stream.read_frame()
	frame_data = frame.get_buffer_as_uint8()
	colorPix = np.frombuffer(frame_data, dtype=np.uint8)
	colorPix.shape = (height, width, 3)
	colorPix = np.flip(colorPix, 2)

	# Find the location of the sphere light
	grayPix = cv2.cvtColor(colorPix, cv2.COLOR_RGB2GRAY)
	grayPix = (np.multiply(grayPix, depthMask)).astype(np.uint8)
	grayPix = cv2.bilateralFilter(grayPix, 11, 17, 17)
	ret,thresh = cv2.threshold(grayPix, minIntensity, 255,cv2.THRESH_BINARY)
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# contours = imutils.grab_contours(contours)
	contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
	contours = [x for x in contours if within(cv2.contourArea(x), minArea, maxArea)] 
	contours = [x for x in contours if contourWithinInertia(x, minCircularity, 1.0)] 

	# Choose the largest contour
	if contours: contours = [contours[0]]

	# Get the average distance to this contour
	contourMask = np.zeros_like(grayPix)
	cv2.drawContours(contourMask, contours, -1, color=1, thickness = -1)
	validDepthMask = (depthPix != 0).astype(np.uint8)
	contourMask = np.multiply(validDepthMask, contourMask)
	
	depthPix = np.multiply(depthPix, contourMask)
	sumDepth = np.sum(depthPix) 
	sumMask = np.sum(contourMask)
	avgDist = 0  # millimeters
	if (sumMask > 0.0001):
		avgDist = sumDepth / sumMask
	avgDistMM = avgDist / 10.0;

	# Create a debug image
	img2 = cv2.cvtColor(grayPix, cv2.COLOR_GRAY2RGB)
	cv2.drawContours(img2, contours, -1, (0,255,0), -1)

	# Get the screen coordinates
	x = px
	y = py
	z = pz
	bNewData = False
	if (avgDist > 0):
		M = cv2.moments(contours[0])
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])

		cv2.drawMarker(img2, tuple([int(cx),int(cy)]), color=(0, 0, 255), thickness=5)

		# Convert screen to world coordinates
		x,y,z = openni2.convert_depth_to_world(depth_stream, cx, cy, avgDist)
		z /= 10.0

		# Smooth the data 
		if not bPrevData:
			bPrevData = True
			px = x
			py = y
			pz = z
		x = x*(1.0-easingParam) + px*easingParam
		y = y*(1.0-easingParam) + py*easingParam
		z = z*(1.0-easingParam) + pz*easingParam
		bNewData = True

	# now, x,y,z are all in mm
	# Send this over osc
	builder = osc_message_builder.OscMessageBuilder(address=header)
	builder.add_arg(int(bNewData))
	builder.add_arg(x)
	builder.add_arg(y)
	builder.add_arg(z)
	msg = builder.build()
	client.send(msg)

	# Save these values
	px = x
	py = y
	pz = z
	
	# Debug the image
	if (bViewFeed):
		cv2.imshow("Image", img2)
		cv2.waitKey(34)

openni2.unload()

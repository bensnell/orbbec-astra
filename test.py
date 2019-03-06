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

import cv2
import numpy as np
from primesense import openni2
from primesense import _openni2 as c_api
openni2.initialize("Redist/") #The OpenNI2 Redist folder
dev = openni2.Device.open_any()
depth_stream = dev.create_depth_stream()
depth_stream.start()
depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX = 640, resolutionY = 480, fps = 30))
while True:
	frame = depth_stream.read_frame()
	frame_data = frame.get_buffer_as_uint16()
	img = np.frombuffer(frame_data, dtype=np.uint16)
	img.shape = (1, 480, 640)
	img = np.concatenate((img, img, img), axis=0)
	img = np.swapaxes(img, 0, 2)
	img = np.swapaxes(img, 0, 1)

	img = img.astype(np.uint8) # This is required to be able to draw it
	cv2.imshow("image", img)
	cv2.waitKey(34)
openni2.unload()

# Orbbec Astra Blob Detection

These scripts attempt to extract a round white blob from the field of view of the Orbbec Astra RGBD camera.

### System Support

This script only supports Windows 64 bit architectures.

### Dependencies / Installation

- Orbbec Drivers

  1. [Download the Orbbec OpenNI SDK here](https://orbbec3d.com/develop).
  2. In this directory, you will find the executable 
     `OpenNI_2.3.0.55 > Windows > Astra OpenNI2 Development Instruction(x64)_V1.3 > Driver > SensorDriver_V4.3.0.9.exe`. Run this exe file and follow the installation prompts. 
  3. Run `setlib_environ.bat` in the directory here:
     `OpenNI_2.3.0.55 > Windows > Astra OpenNI2 Development Instruction(x64)_V1.3 > OpenNI2 > OpenNI-Windows-x64-2.3.0.55 > Redist`
  4. When you plug in an Orbbec sensor, it should be recognized without any error in the DeviceManager. If there is an error, try uninstalling and reinstalling the driver.

- Python 3.7.X

  - To check and see if python is installed, open up Git-Bash and type `python --version`. If python is not installed or if python 3.7 is not installed, [download and install from this link](https://www.python.org/downloads).

- A number of Python modules must be installed. Open up Git Bash and type in these commands, each followed by the return key.

  ```python
  pip install numpy
  pip install opencv-python
  pip install primesense
  pip install python-osc
  pip install imutils
  ```

- The scripts require that the correct path to the "Redist" folder be passed. The "Redist" folder is located at ` OpenNI_2.3.0.55 > Windows > Astra OpenNI2 Development Instruction(x64)_V1.3 > OpenNI2 > OpenNI-Windows-x64-2.3.0.55 > Redist`, or for x86 archiectures, ` OpenNI_2.3.0.55 > Windows > Astra OpenNI2 Development Instruction(x86)_V1.3 > OpenNI2 > OpenNI-Windows-x86-2.3.0.55 > Redist`. The path to this folder can be specified in the "User Parameters" section of the script. However, it has been included within this repo for convenience. For reference, the "Redist" folder should contain these files:
  - Redist
    - OpenNI2
      - Drivers
        - OniFile.dll
        - OniFile.ini
        - orbbec.dll
        - Orbbec.ini
    - OpenNI.ini
    - OpenNI2.dll
    - OpenNI2.jni.dll
    - org.openni.jar
If you get a Win 193 error upon trying to run the python script, try using the Redist folder for the other architecture.

### Running This Script

1. Plug the Orbbec into the USB port of your computer

2. Open Git-Bash and navigate to the directory "scripts". This would utilize a command similar to:

   `cd /c/Users/your_name/Desktop/orbbec-astra/scripts`

3. Type the following command into Git-Bash to begin doing computer vision and sending the OSC messages:

   `python detect_blobs.py`

4. To stop the script, focus on the Git-Bash window and type in **Ctrl+c**

### How to Get the Output of the Script

The script sends x, y, z coordinates of the blob over OSC using the header "/orbbec", the address for localhost, and the port 8000, by default. These parameters can be changed within the script.

To easily visualize these messages, use [this 3rd party app](https://www.kasperkamperman.com/blog/processing-code/osc-datamonitor/). 

### Script Parameters

Successful blob detection requires tuning a few parameters of the script. These parameters are listed within the script under the section "User Parameters". To change a parameter, stop running the script, then change the script, save it, and re-run it.

### TODO

The coordinates are sometimes noisy, especially the z. This is likely because small changes in the blob outline contribute to significant changes in the z, but only marginal changes in the x and y. To attenuate this noise, use [filterpy](https://github.com/rlabbe/filterpy) module to implement either realtime Kalman or G-H filters.

### References

Blob Tracking:

https://raw.githubusercontent.com/spmallick/learnopencv/master/BlobDetector/blob.py

https://www.learnopencv.com/blob-detection-using-opencv-python-c/

https://docs.opencv.org/3.0.0/d4/d73/tutorial_py_contours_begin.html

https://docs.opencv.org/3.2.0/d7/d4d/tutorial_py_thresholding.html

Orbbec API:

https://github.com/kanishkaganguly/OpenNIMultiSensorCapture/blob/master/capture_cam.py

https://github.com/tomerfiliba/PrimeSense/blob/master/primesense/openni2.py

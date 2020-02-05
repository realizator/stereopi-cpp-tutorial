# stereopi-cpp-tutorial
 Basic C++ code for Python vs C++ OpenCV performance comparison

This code is a port from Python scripts (https://github.com/realizator/stereopi-fisheye-robot)
for this article:

https://stereopi.com/blog/opencv-comparing-speed-c-and-python-code-raspberry-pi-stereo-vision

Draft notice:

1. To compile script1.cpp run:

g++ /home/pi/stereopi-cpp-tutorial/src/script1.cpp -o /home/pi/stereopi-cpp-tutorial/bin/script1.bin -I/usr/local/include/opencv4 -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d

2. To run script1 use this command:

raspividyuv -3d sbs -w 640 -h 240 -fps 90 --luma -t 0 -n -o - | /home/pi/stereopi-cpp-tutorial/bin/script1.bin

(use rpi-update to add 3D support to raspiyuv)

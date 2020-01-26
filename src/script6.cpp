// Copyright (C) 2019 Eugene a.k.a. Realizator, stereopi.com, virt2real team
// Ported from Python to C by Konstantin Ozernov on 10/10/2019.
//
// This file is part of StereoPi С tutorial scripts, and has been
// ported from Pyton version (https://github.com/realizator/stereopi-fisheye-robot)
//
// StereoPi tutorial is free software: you can redistribute it 
// and/or modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation, either version 3 of the 
// License, or (at your option) any later version.
//
// StereoPi tutorial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with StereoPi tutorial.  
// If not, see <http://www.gnu.org/licenses/>.
//

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>


int SWS = 5;
int PFS = 5;
int preFiltCap = 29;
int minDisp = -25;
int numOfDisp = 16;
int TxtrThrshld = 100;
int unicRatio = 10;
int SpcklRng = 15;
int SpklWinSze = 100;
float actualFPS = 0.0;

// Global settings
std::string folder_name = "home/pi/stereopi-cpp-tutorial/"";
std::string calibration_data_folder = folder_name + "calibration_data/"; 

long long getTimestamp() {
    const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    return  epoch.count();
}


void loadParams()
{
    fprintf(stderr, "Loading params...\n");
    std::string filename = folder_name + "3dmap_set.yml";
    cv::FileStorage fs;
    if (fs.open(filename, cv::FileStorage::READ))
    {
        fs["SWS"] >> SWS;
        fs["PFS"] >> PFS;
        fs["preFiltCap"] >> preFiltCap;
        fs["minDisp"] >> minDisp;
        fs["numOfDisp"] >> numOfDisp;
        fs["TxtrThrshld"] >> TxtrThrshld;
        fs["unicRatio"] >> unicRatio;
        fs["SpcklRng"] >> SpcklRng;
        fs["SpklWinSze"] >> SpklWinSze;
    }


}

bool stereo_depth_map(cv::Mat &left, cv::Mat &right, cv::Ptr<cv::StereoBM> bm,
		      float &time1, float &time2, float &time3, float &time4)
{
    cv::Mat disp, disp8, colored;
    long long startT = getTimestamp();
    bm->compute( left, right, disp);
    long long computeTime = getTimestamp();
    time1 += computeTime - startT;
    disp.convertTo(disp8, CV_8U);
    long long ucharConvertTime = getTimestamp();
    time2 += ucharConvertTime - computeTime;
    cv::applyColorMap(disp8, colored, cv::COLORMAP_JET);
    long long colorizeTime = getTimestamp();
    time3 += colorizeTime - ucharConvertTime;
    cv::imshow("map", colored);
    long long showtime = getTimestamp();
    time4 += showtime - colorizeTime;
    char k = cv::waitKey(1);
    if (k == 'q' || k == 'Q')
      return false;

    return true;    
}

int main()
{
    int imgHeight = 240;
    int imgWidth = 640;

    FILE *fp;
    if ((fp = fopen("/dev/stdin", "rb")) == NULL)
    {
        fprintf(stderr, "Cannot open input file!\n");
        return 1;
    }

    int bufLen = imgWidth * imgHeight;
    char *buf = (char *)malloc(bufLen);
    int count = 0;
    
    cv::FileStorage fsStereo(calibration_data_folder + "stereo_camera_calibration" + std::to_string(imgHeight) + ".yml", cv::FileStorage::READ);
    if (!fsStereo.isOpened())
    {
        fprintf(stderr, "Camera calibration data not found in cache\n");
        exit(1);
    }
    cv::Mat leftMapX, leftMapY, rightMapX, rightMapY;
    fsStereo["leftMapX"] >> leftMapX;
    fsStereo["leftMapY"] >> leftMapY;
    fsStereo["rightMapX"] >> rightMapX;
    fsStereo["rightMapY"] >> rightMapY;

    loadParams();

    cv::namedWindow("map");
    cv::moveWindow("map", 50, 100);
    cv::namedWindow("Left");
    cv::moveWindow("Left", 450, 100);
    cv::namedWindow("Right");
    cv::moveWindow("Right", 850, 100);

   cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);

    if (SWS < 5)
        SWS = 5;
    if (SWS % 2 == 0)
        SWS += 1;
    if (SWS > 80)
        SWS = 79;

    if (numOfDisp % 16 != 0)
    {
        numOfDisp -= (numOfDisp % 16);
    }

    bm->setPreFilterCap(preFiltCap);
    bm->setBlockSize(SWS);
    bm->setMinDisparity(minDisp);
    bm->setNumDisparities(numOfDisp);
    bm->setTextureThreshold(TxtrThrshld);
    bm->setUniquenessRatio(unicRatio);
    bm->setSpeckleWindowSize(SpklWinSze);
    bm->setSpeckleRange(SpcklRng);
    bm->setDisp12MaxDiff(1);

    float  time1 = 0, time2 = 0, time3 = 0, time4 = 0, time5 = 0, time6 = 0, time7 = 0, time8 = 0, time9 = 0, time10 = 0, time11 = 0;
    int frameNumber = 0;
    
    while (true)
    {
        fseek(fp, -bufLen, SEEK_END);
        count = fread(buf, sizeof(*buf), bufLen, fp);
    	if (count == 0)
    	    break;
        long long starttime = getTimestamp();
   
        cv::Mat frame(imgHeight, imgWidth, CV_8UC1, buf);

	long long timeReadFrame = getTimestamp();
	time1 += timeReadFrame - starttime;
	
        cv::Mat left = cv::Mat(frame, cv::Rect(0, 0, imgWidth / 2, imgHeight));
        cv::Mat right = cv::Mat(frame, cv::Rect(imgWidth / 2, 0, imgWidth / 2, imgHeight));

	long long timeSplitLeftRight = getTimestamp();
	time2 += timeSplitLeftRight - timeReadFrame;

	
        // Rectifying left and right images
        cv::remap(left, left, leftMapX, leftMapY, cv::INTER_LINEAR);
        cv::remap(right, right, rightMapX, rightMapY, cv::INTER_LINEAR);

	long long timeRectify = getTimestamp();
	time3 += timeRectify - timeSplitLeftRight;
       
        cv::imshow("Left", left);
        cv::imshow("Right", right);

	long long timeShow = getTimestamp();
	time4 += timeShow - timeRectify;

	
	// Taking a strip from our image for lidar-like mode (and saving CPU)
        // cv::Mat imgLCut = cv::Mat(left, cv::Rect(0, 80, left.cols, 80));
        // cv::Mat imgRCut = cv::Mat(right, cv::Rect(0, 80, right.cols, 80));

	// cv::resize(left, left, cv::Size(176, 132));
	// cv::resize(right, right, cv::Size(176, 132));
	
	long long timeStrip = getTimestamp();
	time5 += timeStrip - timeShow;
	
        bool depth = stereo_depth_map(left, right, bm, time8, time9, time10, time11);

	long long timeDepth = getTimestamp();
	time6 += timeDepth - timeStrip;

	time7 += timeDepth - starttime;
	
	frameNumber++;
	if (!depth)
	  break;
    }

    time1 = time1 / (frameNumber * 1000);
    time2 = time2 / (frameNumber * 1000);
    time3 = time3 / (frameNumber * 1000);
    time4 = time4 / (frameNumber * 1000);
    time5 = time5 / (frameNumber * 1000);
    time6 = time6 / (frameNumber * 1000);
    time7 = time7 / (frameNumber * 1000);
    time8 = time8 / (frameNumber * 1000);
    time9 = time9 / (frameNumber * 1000);
    time10 = time10 / (frameNumber * 1000);
    time11 = time11 / (frameNumber * 1000);
    actualFPS = 1000.0/time7;

    fprintf(stderr, "Avg time (milliseconds):\nread frame: %f\nsplit to left and right: %f\nrectify: %f\nshow left and right: %f\nget strips: %f\nget depth map: %f\ncompute map: %f\nconvert to uchar: %f\ncolorize: %f\nshow map: %f\ntotal: %f\nactual FPS: %f\n", time1, time2, time3, time4, time5, time6, time8, time9, time10, time11, time7, actualFPS);
    
    return 0;
}

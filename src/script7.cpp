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

int SWS = 7;
int PFS = 7;
int preFiltCap = 5;
int minDisp = 1;
int numOfDisp = 32;
int TxtrThrshld = 13;
int unicRatio = 19;
int SpcklRng = 0;
int SpklWinSze = 0;

// Global settings
std::string folder_name = "/home/pi/stereopi-c-tutorial/";
std::string calibration_data_folder = folder_name + "calibration_data/";


// Visualization settings
bool showDisparity = true;
bool showUndistortedImages = true;
bool showColorizedDistanceLine = true;


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

cv::Mat stereo_depth_map(cv::Ptr<cv::StereoBM> bm, cv::Mat left, cv::Mat right)
{
    cv::Mat disparity;
    bm->compute( left, right, disparity);
    disparity.convertTo(disparity, CV_8U);
    return disparity;
}

int main()
{
    int imgHeight = 240;
    int imgWidth = 640;

  
    int map_width = 320;
    int map_height = 240;

    FILE *fp;
    if ((fp = fopen("/dev/stdin", "rb")) == NULL)
    {
        fprintf(stderr, "Cannot open input file!\n");
        return 1;
    }

    int bufLen = imgWidth * imgHeight;
    char *buf = (char *)malloc(bufLen);
    int count = 0;
    long long framesNumber = 0;

    cv::FileStorage fsStereo(calibration_data_folder + "stereo_camera_calibration" + std::to_string(imgHeight) + ".yml", cv::FileStorage::READ);
    if (!fsStereo.isOpened())
    {
        fprintf(stderr, "Camera calibration data not found in cache\n");
        exit(1);
    }
    cv::Mat leftMapX, leftMapY, rightMapX, rightMapY, QQ;
    fsStereo["leftMapX"] >> leftMapX;
    fsStereo["leftMapY"] >> leftMapY;
    fsStereo["rightMapX"] >> rightMapX;
    fsStereo["rightMapY"] >> rightMapY;
    fsStereo["disparityToDepthMap"] >> QQ;

    loadParams();

    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);

    if (SWS < 5)
        SWS = 5;
    if (SWS % 2 == 0)
        SWS += 1;
    if (SWS > map_height)
        SWS = map_height - 1;

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
    
    // while ((count = fread(buf, sizeof(*buf), bufLen, fp)) != 0)
    // {
    while (true)
    {
        fseek(fp, -bufLen, SEEK_END);
        count = fread(buf, sizeof(*buf), bufLen, fp);
        if (count == 0)
	    break;
	cv::Mat frame(imgHeight, imgWidth, CV_8UC1, buf);
        cv::Mat left = cv::Mat(frame, cv::Rect(0, 0, imgWidth / 2, imgHeight));
        cv::Mat right = cv::Mat(frame, cv::Rect(imgWidth / 2, 0, imgWidth / 2, imgHeight));

        // Rectifying left and right images
        cv::remap(left, left, leftMapX, leftMapY, cv::INTER_LINEAR);
        cv::remap(right, right, rightMapX, rightMapY, cv::INTER_LINEAR);

        // Taking a strip from our image for lidar-like mode (and saving CPU)
        cv::Mat imgLCut = cv::Mat(left, cv::Rect(0, 80, left.cols, 80));
        cv::Mat imgRCut = cv::Mat(right, cv::Rect(0, 80, right.cols, 80));

	cv::Mat native_disparity = stereo_depth_map(bm, imgLCut, imgRCut);

	cv::imshow("depth", native_disparity);
   	
        cv::Mat maximized_line = native_disparity.clone();

        cv::Mat maxInColumns = cv::Mat::zeros(native_disparity.cols, 1, CV_8UC1);
        int matMax = INT_MIN;
        int matMin = INT_MAX;
        for (int i = 0; i < native_disparity.cols; i++)
        {
            int max = INT_MIN;
            int min = INT_MAX;
            for (int j = 0; j < native_disparity.rows; j++)
            {
	        int val = (int)(native_disparity.at<uchar>(j, i));
                if (val > max)
                    max = val;
                if (val < min)
                    min = val;
            }
	    //fprintf(stderr, "%d\n", max);
            maxInColumns.at<uchar>(i, 0) = max;
            if (max > matMax)
                matMax = max;
            if (min < matMin)
                matMin = min;
        }

	cv::Mat points;
        cv::reprojectImageTo3D(maxInColumns, points, QQ);
        cv::Mat xy_projection = cv::Mat::zeros(map_height, map_width, CV_8UC1);

	
        //choose "closest" points in each column
        for (int i = 0; i < maximized_line.rows; i++)
        {
            for (int j = 0; j < maximized_line.cols; j++)
            {
                maximized_line.at<uchar>(i, j) = maxInColumns.at<uchar>(j, 0);
            }
        }

        // Put all points to the 2D map

        // Change map_zoom to adjust visible range!
        float map_zoom = 50.0;

        for (int i = 0; i < points.rows; i++)
        {
	    float cur_x = points.at<cv::Vec3f>(i, 0)[0];
	    float cur_y = points.at<cv::Vec3f>(i, 0)[1];

            int xx = (int)(cur_y * map_zoom) + (int)(map_width/2); // zero point is in the middle of the map
            int yy = (int)(cur_x * map_zoom) + map_height;     // zero point is at the bottom of the map

	    // If the point fits on our 2D map - let's draw it!
            if (yy < map_width && yy >= 0 && xx < map_height && yy >= 0)
	      xy_projection.at<uchar>(yy, xx) = maxInColumns.at<uchar>(i, 0);//maximized_line.at<uchar>(i, 0);
        }
        cv::Mat xy_projection_color, max_line_color, disparity_color;
	cv::applyColorMap(native_disparity, disparity_color, cv::COLORMAP_JET);
        cv::applyColorMap(xy_projection, xy_projection_color, cv::COLORMAP_JET);
        cv::applyColorMap(maximized_line, max_line_color, cv::COLORMAP_JET);


        // show the frame
        if (showUndistortedImages)
        {
            cv::imshow("left", imgLCut);
            cv::imshow("right", imgRCut);
        }
        if (showColorizedDistanceLine)
            cv::imshow("Max distance line", max_line_color);
        cv::imshow("XY projection", xy_projection_color) ;
	cv::imshow("image", disparity_color);
	char k = cv::waitKey(1);
	if (k == 'q' || k == 'Q')
	 exit(1);
    }

    return 0;
}

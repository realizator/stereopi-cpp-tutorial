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
int numOfDisp = 128;
int TxtrThrshld = 100;
int unicRatio = 10;
int SpcklRng = 15;
int SpklWinSze = 100;

// Global settings
std::string folder_name = "home/pi/stereopi-cpp-tutorial/";
std::string calibration_data_folder = folder_name + "calibration_data/"; 

cv::Mat left, right;

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

    fprintf(stderr, "pfs = %d\n", PFS);

}

void saveParams()
{
    fprintf(stderr, "Saving params...\n");
    std::string filename = folder_name + "3dmap_set.yml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "SWS" << SWS << "PFS" << PFS << "preFiltCap" << preFiltCap << "minDisp" << minDisp << "numOfDisp" << numOfDisp
       << "TxtrThrshld" << TxtrThrshld << "unicRatio" << unicRatio  << "SpcklRng" << SpcklRng << "SpklWinSze" << SpklWinSze;
}

void stereo_depth_map(cv::Mat left, cv::Mat right)
{
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);

    if (SWS < 5)
        SWS = 5;
    if (SWS % 2 == 0)
        SWS += 1;
    if (SWS > left.rows)
        SWS = left.rows - 1;

    if (numOfDisp < 16)
      numOfDisp = 16;
    if (numOfDisp % 16 != 0)
    {
        numOfDisp -= (numOfDisp % 16);
    }

    if (preFiltCap < 1)
      preFiltCap = 1;

    bm->setPreFilterCap(preFiltCap);
    bm->setBlockSize(SWS);
    bm->setMinDisparity(minDisp);
    bm->setNumDisparities(numOfDisp);
    bm->setTextureThreshold(TxtrThrshld);
    bm->setUniquenessRatio(unicRatio);
    bm->setSpeckleWindowSize(SpklWinSze);
    bm->setSpeckleRange(SpcklRng);
    bm->setDisp12MaxDiff(1);

    cv::Mat disp, disp8, colored;
    bm->compute( left, right, disp);
    disp.convertTo(disp8, CV_8U);
    cv::applyColorMap(disp8, colored, cv::COLORMAP_JET);
    cv::imshow("Image", colored);
    

}

void onTrackbar(int, void *)
{
   stereo_depth_map(left, right);
   // fprintf(stderr, "%d %d %d %d %d %d %d %d\n", preFiltCap, SWS, minDisp, numOfDisp, TxtrThrshld, unicRatio, SpklWinSze, SpcklRng);
}

void onMinDisp(int, void *)
{
  minDisp -= 40;
   // fprintf(stderr, "%d %d %d %d %d %d %d %d\n", preFiltCap, SWS, minDisp, numOfDisp, TxtrThrshld, unicRatio, SpklWinSze, SpcklRng);
  stereo_depth_map(left, right);
}

int main()
{
    std::string imageToDisp = "scenes/dm-tune-example.jpg";
    int photo_width = 640;
    int photo_height = 240;
    int image_width = 320;
    int image_height = 240;

    cv::Size image_size(image_width,image_height);

    FILE *fp;
    if ((fp = fopen("/dev/stdin", "rb")) == NULL)
    {
        fprintf(stderr, "Cannot open input file!\n");
        return 1;
    }

    int bufLen = photo_width * photo_height;
    char *buf = (char *)malloc(bufLen);
    int count = 0;

    cv::FileStorage fsStereo(calibration_data_folder + "stereo_camera_calibration" + std::to_string(image_height) + ".yml", cv::FileStorage::READ);
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

    cv::namedWindow("Image");
    cv::moveWindow("Image", 50, 100);
    cv::namedWindow("Left");
    cv::moveWindow("Left", 450, 100);
    cv::namedWindow("Right");
    cv::moveWindow("Right", 850, 100);

    

    cv::createTrackbar("SWS", "Image", &SWS, 255, onTrackbar);
    cv::createTrackbar("PFS", "Image", &PFS, 255, onTrackbar);
    cv::createTrackbar("PreFiltCap", "Image", &preFiltCap, 63, onTrackbar);
    cv::createTrackbar("MinDISP", "Image", &minDisp, 100, onMinDisp);
    cv::createTrackbar("NumOfDisp", "Image", &numOfDisp, 256, onTrackbar);
    cv::createTrackbar("TxtrThrshld", "Image", &TxtrThrshld, 1000, onTrackbar);
    cv::createTrackbar("UnicRatio", "Image", &unicRatio, 20, onTrackbar);
    cv::createTrackbar("SpcklRng", "Image", &SpcklRng, 40, onTrackbar);
    cv::createTrackbar("SpklWinSze", "Image", &SpklWinSze, 300, onTrackbar);
    
    long long prevTime = getTimestamp();
    float avgFps = 0.0;
    int frameNumber = 0;
    int prevFrameNumber = 0;
    while (true)
    {
          fseek(fp, -bufLen, SEEK_END);
          count = fread(buf, sizeof(*buf), bufLen, fp);
	  if (count == 0)
	      break;

          cv::Mat img(photo_height, photo_width, CV_8UC1, buf);

          left = cv::Mat(img, cv::Rect(0, 0, image_width, image_height));
	  right = cv::Mat(img, cv::Rect(image_width, 0, image_width, image_height));

	  // Rectifying left and right images
	  cv::remap(left, left, leftMapX, leftMapY, cv::INTER_LINEAR);
	  cv::remap(right, right, rightMapX, rightMapY, cv::INTER_LINEAR);

	  stereo_depth_map(left, right);

	  cv::imshow("Left", left);
	  cv::imshow("Right", right);

	  char k = cv::waitKey(1);
	  if (k == 's' || k == 'S')
	  {
	    fprintf(stderr, "k = %c\n", k);
	    saveParams();
	    break;
	  }
	  else if (k == 'q' || k == 'Q')
	    break;

	  frameNumber++;
	  long long currTime = getTimestamp();
	  if (currTime - prevTime > 1000000)
	  {
	      fprintf(stderr, "FPS: %d\n", frameNumber - prevFrameNumber);
	      prevFrameNumber = frameNumber;
	      prevTime = currTime;
	  }
	  
    }
    return 0;
}

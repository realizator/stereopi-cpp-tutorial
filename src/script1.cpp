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


#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <ctime>

long long getTimestamp()
{
    const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    return  epoch.count();
}

int main()
{
    fprintf(stderr, "You can press 'Q' to quit this script.\n");
  
    int imgHeight = 240;
    int imgWidth = 640;

    fprintf(stderr, "Camera resolution: %d x %d\n", imgWidth, imgHeight);

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
    long long startTime = getTimestamp();
    long long totalTime = 0;
    while (true)
    {
        long long frameStartTime = getTimestamp();
        fseek(fp, -bufLen, SEEK_END);
        count = fread(buf, sizeof(*buf), bufLen, fp);
        if (count == 0)
	    break;
        cv::Mat frame(imgHeight, imgWidth, CV_8UC1, buf);
        cv::imshow("video", frame);
        framesNumber++;

	long long frameTime = getTimestamp() - frameStartTime;
	totalTime += frameTime;
	
        char k = cv::waitKey(1);
        if (k == 'q' || k == 'Q')
        {
            cv::imwrite("frame.jpg", frame);
            break;
        }
    }
    //float fps = (float)( 1000000 * framesNumber) / (getTimestamp() - startTime);
    float avgTime = totalTime / 1000 / framesNumber;
    fprintf(stderr, "Average time between frames: %f ms\n", avgTime); 
    fprintf(stderr, "Average FPS: %f\n", 1000 / avgTime);
    return 0;
}


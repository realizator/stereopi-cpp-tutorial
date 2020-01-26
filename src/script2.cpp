// Copyright (C) 2019 Eugene a.k.a. Realizator, stereopi.com, virt2real team
// Ported from Python to C++ by Konstantin Ozernov on 10/10/2019.
//
// This file is part of StereoPi С++ tutorial scripts, and has been
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

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>

long long getTimestamp()
{
   const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
   const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
   return  epoch.count();
}

int main()
{ 
        //User quit oid message
    fprintf(stderr, "You can press 'Q' to quit this script.\n");

    //Photo session settings
    int total_photos = 50;//            # Number of images to take
    int countdown = 3;//                 # Interval for count-down timer, seconds
    int font = cv::FONT_HERSHEY_SIMPLEX;// # Cowntdown timer font
    std::string folder_name = "/home/pi/stereopi-cpp-tutorial/";     
    //Camera settimgs
    int cam_width = 1280;//            # Cam sensor width settings
    int cam_height = 480;//              # Cam sensor height settings

    int half_width = 640;
    int half_height = 480;
    
    //Final image capture settings
    float scale_ratio = 1;//

        // Camera resolution height must be dividable by 16, and width by 32
        
    cam_width = (int)(((cam_width+31)/32)*32);
    cam_height = (int)(((cam_height+15)/16)*16);
    fprintf(stderr, "Camera resolution: %d x %d\n", cam_width, cam_height);

        // Buffer for captured image settings
    int img_width = (int)((cam_width * scale_ratio));
    int img_height = (int)((cam_height * scale_ratio));
    //    capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
    fprintf(stderr, "Scaled image resolution: %d x %d\n", img_width, img_height);

        // Initialize the camera
    //cv::VideoCapture cap(0);//"/dev/stdin");

    FILE *fp;
    if ((fp = fopen("/dev/stdin", "rb")) == NULL)
    {
        fprintf(stderr, "Cannot open input file!\n");
        return 1;
    }

    // Check if needed dirs exist. If do not - try to create them.
    std::string scenesDir = folder_name + "scenes";
    std::string pairsDir = folder_name + "pairs";
    DIR* dir = opendir(scenesDir.c_str());
    if (dir)
    {
	closedir(dir);
    }
    else
    {
      int res = mkdir(scenesDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if ( res == -1)
	{
	    fprintf(stderr, "Cannot create Scenes dir!\n");
	    return 1;
	}
    }
    dir = opendir(pairsDir.c_str());
    if (dir)
    {
	closedir(dir);
    }
    else
    {
      int res = mkdir(pairsDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if ( res == -1)
	{
	    fprintf(stderr, "Cannot create Pairs dir!\n");
	    return 1;
	}
    }  
    

    int bufLen = cam_width * cam_height;
    char *buf = (char *)malloc(bufLen);
    int count = 0;

    
        // Lets start taking photos!
    int counter = 0;
    long long t2 = getTimestamp();
    fprintf(stderr, "Starting photo sequence\n");
    cv::Mat frame;
    //    while ((count = fread(buf, sizeof(*buf), bufLen, fp)) != 0)
    while (true)
    {
        fseek(fp, -bufLen, SEEK_END);
        count = fread(buf, sizeof(*buf), bufLen, fp);
	if (count == 0)
	  break;

        cv::Mat frame(cam_height, cam_width, CV_8UC1, buf);
        long long t1 = getTimestamp();
        int cntdwn_timer = countdown - (int)((t1-t2) / 1000000);
	
            
        // If cowntdown is zero - let's record next image
        if (cntdwn_timer == -1)
        {
            counter += 1;
            std::string filename = folder_name + "scenes/scene_" + std::to_string(img_width) + "x" + std::to_string(img_height) + "_" + std::to_string(counter) + ".png";
            cv::imwrite(filename, frame);
            fprintf(stderr, "[%d of %d] %s\n", counter, total_photos, filename.c_str());

	    cv::Mat imgLeft = cv::Mat(frame, cv::Rect(0, 0 , half_width, half_height));
	    cv::Mat imgRight = cv::Mat(frame, cv::Rect(half_width, 0, half_width, half_height));
	    std::string leftName = folder_name + "pairs/left_" + std::to_string(counter) + ".png";
	    std::string rightName = folder_name + "pairs/right_" + std::to_string(counter) + ".png";
	    cv::imwrite(leftName, imgLeft);
	    cv::imwrite(rightName, imgRight);
	    
            t2 = getTimestamp();
            sleep(1);
            cntdwn_timer = 0;      // To avoid "-1" timer display
        }
            
        // Draw cowntdown counter, seconds
        cv::putText(frame, std::to_string(cntdwn_timer), cv::Point(50,50), font, 2.0, cv::Scalar(0,0,255), 4, 16 /*CV_AA*/);
        cv::imshow("pair", frame);
        char key = cv::waitKey(1);
        // Press 'Q' key to quit, or wait till all photos are taken
        if (key == 'q' || key == 'Q' || counter == total_photos)
            break;
    }
         
    fprintf(stderr, "Photo sequence finished\n");
    
    return 0;
}

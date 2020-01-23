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


// Camera resolution
int photo_width = 1280;
int photo_height = 480;

// Image resolution for processing
int img_width = 320;
int img_height = 240;

// Global settings
std::string folder_name = "/home/pi/stereopi-c-tutorial/";
std::string calibration_data_folder = folder_name + "calibration_data/";  

void calibrate_one_camera(std::vector<std::vector<cv::Vec3f> > objpoints, std::vector<std::vector<cv::Vec2f> > imgpoints, std::string right_or_left)
{
    int N_OK = (int)objpoints.size();
    cv::Size DIM(img_width, img_height);

    cv::Mat K;// = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat D;// = cv::Mat::zeros(4, 1, CV_32FC1);

    cv::Vec3f pt(0, 0, 0);
    //    std::vector<cv::Vec3f> rvecs(N_OK, pt);
    //    std::vector<cv::Vec3f> tvecs(N_OK, pt);
    cv::Mat rvecs = cv::Mat::zeros(N_OK, 1, CV_32FC3);
    cv::Mat tvecs = cv::Mat::zeros(N_OK, 1, CV_32FC3);

    cv::TermCriteria calib_criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1e-6);
    int calibration_flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |  cv::fisheye::CALIB_FIX_SKEW;
    double rms = cv::fisheye::calibrate(objpoints, imgpoints, DIM, K, D, rvecs, tvecs, calibration_flags, calib_criteria);
    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(K, D, pt, K, DIM, CV_16SC2, map1, map2);

    // Now we'll write our results to the file for the future use
    cv::FileStorage fs(calibration_data_folder + "calibration_camera_" + std::to_string(img_height) + "_" + right_or_left + ".yml", cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "map1" << map1 << "map2" << map2 << "objpoints" << objpoints << "imgpoints" <<
              imgpoints << "camera_matrix" << K << "distortion_coeff" << D;
    }


}

bool calibrate_stereo_cameras(int res_x = img_width, int res_y = img_height)
{
    std::vector<std::vector<cv::Vec3f> > objectPoints;

    std::vector<std::vector<cv::Vec2f> >rightImagePoints;
    cv::Mat rightCameraMatrix;
    cv::Mat rightDistortionCoefficients;

    std::vector<std::vector<cv::Vec2f> > leftImagePoints;
    cv::Mat leftCameraMatrix;
    cv::Mat leftDistortionCoefficients;

    cv::Mat rotationMatrix;
    cv::Mat translationVector;

    cv::Size imageSize(res_x, res_y);

    cv::TermCriteria TERMINATION_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
    std::string right_or_left = "";

    for (int i = 0; i <= 1; i++)
    {
        if (i == 0)
            right_or_left = "left";
        else
            right_or_left = "right";

        cv::FileStorage fs(calibration_data_folder + "calibration_camera_" + std::to_string(img_height) + "_" + right_or_left + ".yml", cv::FileStorage::READ);
        if (fs.isOpened())
        {
            cv::Mat map1, map2;
            fs["map1"] >> map1;
            fs["map2"] >> map2;
            fs["objpoints"] >> objectPoints;
            if (right_or_left == "left")
            {
                fs["imgpoints"] >> leftImagePoints;
                fs["camera_matrix"] >> leftCameraMatrix;
                fs["distortion_coeff"] >> leftDistortionCoefficients;
            }
            else
            {
                fs["imgpoints"] >> rightImagePoints;
                fs["camera_matrix"] >> rightCameraMatrix;
                fs["distortion_coeff"] >> rightDistortionCoefficients;
            }
            fs.release();
        }
        else
        {
            fprintf(stderr, "Camera calibration data not found in cache.\n");
            return false;
        }
    }
    fprintf(stderr, "Calibrating cameras together...\n");
    int fisheyeFlags = 0;
    fisheyeFlags |= cv::fisheye::CALIB_FIX_INTRINSIC;
    //fisheyeFlags &= cv::fisheye::CALIB_CHECK_COND;
    double rms = cv::fisheye::stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints, leftCameraMatrix, leftDistortionCoefficients,
                                              rightCameraMatrix, rightDistortionCoefficients, imageSize, rotationMatrix, translationVector,
                                              fisheyeFlags, TERMINATION_CRITERIA);

    fprintf(stderr, "<><><><><><><><><><><><><><>\n");
    fprintf(stderr, "<><><> RMS is %f <><><><><><>\n", rms);
    fprintf(stderr, "<><><><><><><><><><><><><><>\n");

    cv::Mat R1, R2, P1, P2, Q;
    cv::fisheye::stereoRectify(leftCameraMatrix, leftDistortionCoefficients, rightCameraMatrix, rightDistortionCoefficients,
                               imageSize, rotationMatrix, translationVector, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, cv::Size(0, 0), 0, 0);
    fprintf(stderr, "Saving calibration...\n");
    cv::Mat leftMapX, leftMapY, rightMapX, rightMapY;
    cv::fisheye::initUndistortRectifyMap(leftCameraMatrix, leftDistortionCoefficients, R1, P1, imageSize, CV_16SC2, leftMapX, leftMapY);
    cv::fisheye::initUndistortRectifyMap(rightCameraMatrix, rightDistortionCoefficients, R2, P2, imageSize, CV_16SC2, rightMapX, rightMapY);

    cv::FileStorage fsWrite(calibration_data_folder + "stereo_camera_calibration" + std::to_string(img_height) + ".yml", cv::FileStorage::WRITE);
    if (fsWrite.isOpened())
        fsWrite << "imageSize" << imageSize << "leftMapX" << leftMapX << "leftMapY" << leftMapY << "rightMapX" << rightMapX <<
                   "rightMapY" << rightMapY << "disparityToDepthMap" << Q;


    return true;

}

int main()
{
    // Global variables preset
    int total_photos = 50;


    cv::Size image_size(img_width,img_height);

    // Chessboard parameters
    int rows = 6;
    int columns = 9;
    int square_size = 2.5;

    // Visualization options
    bool drawCorners = false;
    bool showSingleCamUndistortionResults = true;
    bool showStereoRectificationResults = true;
    bool writeUdistortedImages = true;
    std::string imageToDisp = folder_name + "scenes/scene_1280x480_1.png";

    //Calibration settings
    cv::Size CHECKERBOARD(6,9);

    cv::Mat gray_small_left, gray_small_right;

    cv::TermCriteria subpix_criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1);
    // calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_FIX_SKEW

    cv::Vec3f pt(0, 0, 0);
    std::vector<cv::Vec3f> objp;//(CHECKERBOARD.width * CHECKERBOARD.height, pt);
    for (int i = 0; i < CHECKERBOARD.height; i++)
    {
        for (int j = 0; j < CHECKERBOARD.width; j++)
        {
            objp.push_back(cv::Vec3f(j, i, 0));
        }
    }
    fprintf(stderr, "size: %d\n", objp.size());
    std::vector<std::vector<cv::Vec3f> > objpointsLeft;
    std::vector<std::vector<cv::Vec2f> > imgpointsLeft;
    std::vector<std::vector<cv::Vec3f> > objpointsRight;
    std::vector<std::vector<cv::Vec2f> > imgpointsRight;
    // _img_shape;

    if (drawCorners)
        fprintf(stderr, "You can press 'Q' to quit this script.\n");

    for (int photo_counter = 1; photo_counter <= total_photos; photo_counter++)
    {
        fprintf (stderr, "Import pair No %d\n", photo_counter);
        std::string leftName = folder_name + "pairs/left_";
        leftName += std::to_string(photo_counter) + ".png";
        std::string rightName = folder_name + "pairs/right_";
        rightName += std::to_string(photo_counter) + ".png";

        cv::Mat imgL = cv::imread(leftName);
        cv::Mat imgR = cv::imread(rightName);
        if (imgR.empty() || imgL.empty())
        {
            fprintf(stderr, "There are no images in pair No %d\n", photo_counter);
	    continue;
        }

        // If stereopair is complete - go to processing
        cv::Mat grayL;
        cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
        cv::resize (grayL, gray_small_left, cv::Size(img_width,img_height), cv::INTER_AREA);
        cv::Mat grayR;
        cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);
        cv::resize(grayR, gray_small_right, cv::Size(img_width, img_height), cv::INTER_AREA);


        // Find the chessboard corners
        std::vector<cv::Vec2f> cornersL;
        bool retL = cv::findChessboardCorners(grayL, CHECKERBOARD, cornersL, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);
        std::vector<cv::Vec2f> cornersR;
        bool retR = cv::findChessboardCorners(grayR, CHECKERBOARD, cornersR, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);

        // Draw images with corners found
        if (drawCorners)
        {
            cv::drawChessboardCorners(imgL, CHECKERBOARD, cornersL, retL);
            cv::imshow("Corners LEFT", imgL);
            cv::drawChessboardCorners(imgR, CHECKERBOARD, cornersR, retR);
            cv::imshow("Corners RIGHT", imgR);
            char key = cv::waitKey();
            if (key == 'q' || key == 'Q')
                exit(1);
        }

        //       Here is our scaling trick! Hi res for calibration, low res for real work!
        //       Scale corners X and Y to our working resolution
        if ((retL && retR) && (img_height <= photo_height))
        {
            float scale_ratio = (float)img_height/photo_height;
            for (int i = 0; i < cornersL.size(); i++)
                cornersL[i] *= scale_ratio;
            for (int i = 0; i < cornersR.size(); i++)
                cornersR[i] *= scale_ratio;
        }
        else if (img_height > photo_height)
        {
            fprintf(stderr, "Image resolution is higher than photo resolution, upscale needed. Please check your photo and image parameters!\n");
            exit (0);
        }

        // Refine corners and add to array for processing
        if (retL && retR)
        {
            objpointsLeft.push_back(objp);
            cv::cornerSubPix(gray_small_left,cornersL,cv::Size(3,3), cv::Size(-1,-1),subpix_criteria);
            imgpointsLeft.push_back(cornersL);
            objpointsRight.push_back(objp);
            cv::cornerSubPix(gray_small_right,cornersR, cv::Size(3,3), cv::Size(-1,-1), subpix_criteria);
            imgpointsRight.push_back(cornersR);
        }
        else
        {
            fprintf(stderr, "Pair No %d ignored, as no chessboard found\n", photo_counter);
            continue;
        }
        fprintf(stderr, "End cycle\n");
    }

    // Let's calibrate each camera, and than calibrate them together
    fprintf (stderr, "Left camera calibration...\n");
    calibrate_one_camera(objpointsLeft, imgpointsLeft, "left");
    fprintf(stderr, "Right camera calibration...\n");
    calibrate_one_camera(objpointsRight, imgpointsRight, "right");
    fprintf(stderr, "Stereoscopic calibration...\n");
    bool result = calibrate_stereo_cameras();
    fprintf(stderr, "Calibration complete!\n");

    // The following code just shows you calibration results
    if (showSingleCamUndistortionResults)
    {
        int width = 320;
        int height = 240;

        cv::Mat map1, map2;

        fprintf(stderr, "Undistorting picture with width = %d, height = %d\n", width, height);
        cv::FileStorage fsLeft(calibration_data_folder + "calibration_camera_" + std::to_string(img_height) + "_left" + ".yml", cv::FileStorage::READ);
        if (fsLeft.isOpened())
        {
            fsLeft["map1"] >> map1;
            fsLeft["map2"] >> map2;
            fsLeft.release();
        }
        else
        {
            fprintf(stderr, "Left camera calibration data not found in cache.\n");
            return false;
        }

        cv::Mat undistorted_left;
        cv::remap(gray_small_left, undistorted_left, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        cv::FileStorage fsRight(calibration_data_folder + "calibration_camera_" + std::to_string(img_height) + "_right" + ".yml", cv::FileStorage::READ);
        if (fsRight.isOpened())
        {
            fsRight["map1"] >> map1;
            fsRight["map2"] >> map2;
            fsRight.release();
        }
        else
        {
            fprintf(stderr, "Right camera calibration data not found in cache.\n");
            return false;
        }

        cv::Mat undistorted_right;
        cv::remap(gray_small_right, undistorted_right, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        cv::imshow("Undistorted LEFT", undistorted_left);
        cv::imshow("Undistorted RIGHT", undistorted_right);
        cv::waitKey(0);
        if (writeUdistortedImages)
        {
            cv::imwrite(folder_name + "undistorted_left.jpg", undistorted_left);
            cv::imwrite(folder_name + "undistorted_right.jpg", undistorted_right);
        }

    }

    if (showStereoRectificationResults)
    {
        // lets rectify pair and look at the result

        // NOTICE: we use 320x240 as a working resolution, regardless of
        // calibration images resolution. So 320x240 parameters are hardcoded
        // now.
        cv::FileStorage fsStereo(calibration_data_folder + "stereo_camera_calibration" + std::to_string(img_height) + ".yml", cv::FileStorage::READ);
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

        cv::Mat imgPair = cv::imread(imageToDisp);
        if (imgPair.empty())
        {
            fprintf(stderr, "Cannot load image!\n");
            exit(1);
        }
        cv::resize(imgPair, imgPair, cv::Size(640, 240), cv::INTER_CUBIC);

        cv::Mat imgL = cv::Mat(imgPair, cv::Rect(0, 0, 320, 240));
        cv::Mat imgR = cv::Mat(imgPair, cv::Rect(320, 0, 320, 240));

        // Rectifying left and right images
        cv::remap(imgL, imgL, leftMapX, leftMapY, cv::INTER_LINEAR);
        cv::remap(imgR, imgR, rightMapX, rightMapY, cv::INTER_LINEAR);

        cv::imshow("Left STEREO CALIBRATED", imgL);
        cv::imshow("Right STEREO CALIBRATED", imgR);
        cv::imwrite("rectifyed_left.jpg", imgL);
        cv::imwrite("rectifyed_right.jpg", imgR);
        cv::waitKey(0);
    }


    return 0;
}

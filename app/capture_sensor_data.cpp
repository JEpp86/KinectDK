/**
 * @file capture_sensor_data.cpp
 * @author Jason Epp (jasonepp0@gmail.com)
 * @brief An application to read the sensor data from the Kinect device
 * @date 2022-08-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Kinect.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <conio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <future>


namespace chron = std::chrono;
bool running = false;

void saveImg(cv::Mat& img, std::string filename) {
    cv::imwrite(filename, img);
}

int main(int argc, char* argv[]) {
    std::cout << "Read Calibration" << std::endl;
    Kinect kinect;
    std::cout << "Finding Kinect Device" << std::endl;
    bool success = kinect.Connect();
    std::cout << "Success: " << std::to_string(success) << std::endl;
    std::string serial_num = kinect.GetSerialNumber();
    std::cout << "Serial Number: " << serial_num << std::endl;
    if (!kinect.ConfigureDevice()) {
        std::cout << "Unable to configure device" << std::endl;
        return -1;
    }
    if (!kinect.StartCapture()){ 
        std::cout << "Unable to Start Capture" << std::endl;
        return -1;
    }
    char user_in(0);
    std::cout << "Starting Capture" << std::endl;
    std::cout << "Press 'x' to stop capture." << std::endl;
    chron::time_point<chron::high_resolution_clock> finish = chron::high_resolution_clock::now();
    chron::time_point<chron::high_resolution_clock> now = chron::high_resolution_clock::now();
    img_buf_t colour_img;
    img_buf_t depth_img;
    imu_data_t imu_data;
    std::ofstream imu_file;
    imu_file.open("imu_data.txt");
    std::string fname;
    //std::thread t_colour;
    //std::thread t_depth;
    //cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
#if 0
    if (kinect.GetImages(colour_img, depth_img, 0) && (colour_img.timestamp != 0)) {
        //finish = chron::high_resolution_clock::now();
        //auto diff = chron::duration_cast<chron::microseconds>(finish - now);
        //std::cout << "Period in micrseconds: " << std::to_string(diff.count()) << std::endl;
        std::cout << "Image Timestamp: " << std::to_string(colour_img.timestamp) << std::endl;
        cv::Mat image(cv::Size(colour_img.img_width, colour_img.img_height), CV_8UC4, colour_img.data.data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(depth_img.img_width, depth_img.img_height), CV_16UC1, depth_img.data.data(), cv::Mat::AUTO_STEP);
        //cv::imshow("image", image);
        fname = std::to_string(colour_img.timestamp) + ".png";
        //cv::imwrite(fname, image);
        //cv::imshow("depth", depth_image);
        fname = std::to_string(depth_img.timestamp) + "_depth.tif";
        //cv::imwrite(fname, depth_image);
        //std::thread t_depth(&cv::imwrite, fname, depth_img );
        auto ret = std::async(std::launch::async, &cv::imwrite, std::ref(fname),std::ref(depth_img) );

        //now = finish;
    }
#endif
    while (user_in != 'x') {
        // use 1s timeout for now, go non-blocking when IMU data added
        if (kinect.GetImages(colour_img, depth_img, 0, false) && (colour_img.timestamp != 0)) {
            finish = chron::high_resolution_clock::now();
            auto diff = chron::duration_cast<chron::microseconds>(finish - now);
            now = finish;
            std::cout << "Period in micrseconds: " << std::to_string(diff.count()) << std::endl;
            std::cout << "Image Timestamp: " << std::to_string(colour_img.timestamp) << std::endl;
            cv::Mat image(cv::Size(colour_img.img_width, colour_img.img_height), CV_8UC4, colour_img.data.data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_image(cv::Size(depth_img.img_width, depth_img.img_height), CV_16UC1, depth_img.data.data(), cv::Mat::AUTO_STEP);
            //cv::imshow("image", image);
            fname = std::to_string(colour_img.timestamp) + ".bmp";
            //cv::imwrite(fname, image);
            std::async(&saveImg, std::ref(image), fname );
            //cv::imshow("depth", depth_image);
            fname = std::to_string(depth_img.timestamp) + "_depth.png";
            std::async( &saveImg, std::ref(depth_image), fname );
            //cv::imwrite(fname, depth_image);
            //if (t_depth.joinable()) {
            //    std::cout << "Waiting for thread completion" << std::endl;
            //    t_depth.join();
            //}
            //if (!t_depth.joinable()){
                //delete &t_depth;
                //std::thread t_depth(&cv::imwrite, std::ref(fname), std::ref(depth_img) );

            //} else {
            //    std::cout << "Skipping img save" << std::endl;
            //}
            
        }
        if (kinect.GetIMU(imu_data, 0)) {
            //std::cout << "Got IMU Sample" << std::endl;
            imu_file << std::to_string(imu_data.acc_time) << ' '
                     << std::to_string(imu_data.acc_x) << ' '
                     << std::to_string(imu_data.acc_y) << ' '
                     << std::to_string(imu_data.acc_z) << ' '
                     << std::to_string(imu_data.gy_x) << ' '
                     << std::to_string(imu_data.gy_y) << ' '
                     << std::to_string(imu_data.gy_z) << std::endl;
        }
        if (_kbhit()) {
            user_in = _getch();
        }
    }
    kinect.StopCapture();
    imu_file.close();
    std::cout << "Stopping Capture" << std::endl;
    return 0;
}
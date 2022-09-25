/**
 * @file process_record.cpp
 * @author Jason Epp (jasonepp0@gmail.com)
 * @brief An application to process recorded sensor data into images Ground truth and imu data
 * @date 2022-08-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "KinectUtils.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

int main (int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Usage: process_record.exe <filename.mkv>" << std::endl; 
    }

    std::string filename(argv[1]);
    k4a_playback_t h_playback;
    k4a_record_configuration_t h_config;
    k4a_capture_t capture;
    k4a_calibration_t cal;
    std::cout << "Record File: " << filename.data() << std::endl; 
    if (K4A_RESULT_SUCCEEDED != k4a_playback_open(filename.data(), &h_playback)) {
        std::cout << "Failed to open recording" << std::endl;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_record_configuration( h_playback, &h_config)) {
        std::cout << "Failed to read record configuration" << std::endl;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(h_playback, &cal)) {
        std::cout << "Failed to read calibration" << std::endl;
    }
    // Read Captures
    k4a_stream_result_t stream_results = K4A_STREAM_RESULT_SUCCEEDED;
    img_buf_t img, depth_img;
    std::string fname;
    std::cout << "Processing Captures" << std::endl;
    while (K4A_STREAM_RESULT_SUCCEEDED == stream_results ) {
        stream_results = k4a_playback_get_next_capture(h_playback, &capture);
        if (stream_results == K4A_STREAM_RESULT_EOF) {
            std::cout << "\nReached End of Captures" << std::endl;
        } else if (stream_results == K4A_STREAM_RESULT_FAILED) {
            std::cout << "\nError Reading Capture" << std::endl;
        } else {
            // process captures here
            std::cout << "." ;
            std::tie(img, depth_img) = ExtractColourWithDepthGT(capture, cal);
            try {
                fname = std::to_string(img.timestamp) + ".png";
                /*cv::Size(img.img_width, img.img_height), CV_8UC3,*/
                // if jpeg TODO add check if RGBA use below MAT creation
                cv::Mat imgbuf( 1,img.data.size(), CV_8UC1, img.data.data());
                cv::Mat image = cv::imdecode(imgbuf, cv::IMREAD_UNCHANGED);
                //cv::Mat image(cv::Size(img.img_width, img.img_height), CV_8UC4, img.data.data(), cv::Mat::AUTO_STEP);
                cv::imwrite(fname, image);
                fname = std::to_string(depth_img.timestamp) + "_depth.png";
                cv::Mat depth_image(cv::Size(depth_img.img_width, depth_img.img_height), CV_16UC1, depth_img.data.data(), cv::Mat::AUTO_STEP);
                cv::imwrite(fname, depth_image);
            } catch(...) {
                std::cout << "\nError Saving Image: Exiting" << std::endl;
                k4a_capture_release(capture);
                k4a_playback_close(h_playback);
                return -1;
            }
            k4a_capture_release(capture);
        }
    }
    stream_results = K4A_STREAM_RESULT_SUCCEEDED;
    k4a_imu_sample_t imu_sample;
    std::ofstream imu_file;
    imu_file.open("imu_data.txt");
    std::cout << "Processing IMU data" << std::endl;
    while (K4A_STREAM_RESULT_SUCCEEDED == stream_results) {
        stream_results = k4a_playback_get_next_imu_sample(h_playback, &imu_sample);
        std::cout << ".";
        if (stream_results == K4A_STREAM_RESULT_EOF) {
            std::cout << "\nReached End of IMU Samples" << std::endl;
        } else if (stream_results == K4A_STREAM_RESULT_FAILED) {
            std::cout << "\nError Reading Capture" << std::endl;
        } else { 
            imu_file << std::to_string(imu_sample.acc_timestamp_usec ) << ' '
                     << std::to_string(imu_sample.acc_sample.xyz.x) << ' '
                     << std::to_string(imu_sample.acc_sample.xyz.y) << ' '
                     << std::to_string(imu_sample.acc_sample.xyz.z) << ' '
                     << std::to_string(imu_sample.gyro_sample.xyz.x) << ' '
                     << std::to_string(imu_sample.gyro_sample.xyz.y) << ' '
                     << std::to_string(imu_sample.gyro_sample.xyz.z) << std::endl;
        }
    }
    imu_file.close();
    k4a_playback_close(h_playback);
    return 0;
}
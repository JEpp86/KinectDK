/**
 * @file read_cal_data.cpp
 * @author Jason Epp (jasonepp0@gmail.com)
 * @brief An application to read the cal data from Kinect device
 * @date 2022-08-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Kinect.h"

#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    std::cout << "Read Calibration" << std::endl;
    Kinect kinect;
    std::cout << "Finding Kinect Device" << std::endl;
    bool success = kinect.Connect();
    std::cout << "Success: " << std::to_string(success) << std::endl;
    std::string serial_num = kinect.GetSerialNumber();
    std::cout << "Serial Number: " << serial_num << std::endl;
    kinect.ConfigureDevice();
    kinect.PrintDepthCalibration();
    kinect.PrintColourCalibration();
    kinect.PrintExtrinsicCalibration();
}
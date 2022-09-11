/**
 * @file Kinect.h
 * @author Jason Epp (jasonepp0@gmail.com)
 * @brief A class fo a Kinect device to connects and stream sensor infomration, and record/save
 * @version 0.1
 * @date 2022-08-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "k4a/k4a.hpp"

#include <cstdint>
#include <string>

struct img_buf_t {
    std::time_t timestamp;
    uint32_t img_height;
    uint32_t img_width;
    uint32_t stride;
    uint8_t bits_per_pixel;
    std::vector<uint8_t> data;
};

struct imu_data_t {
    float temperature;
    std::time_t acc_time;
    float acc_x;
    float acc_y;
    float acc_z;
    std::time_t gy_time;
    float gy_x;
    float gy_y;
    float gy_z;
};

class Kinect
{
private:
    /* data */
    bool m_connected;                       ///< Whether Kinect Device is connected
    bool m_configured;                      ///< Whether the device has been configured
    bool m_capturing;                       ///< Whether the Kinect is capturing data
    //std::string img_path;                   ///< Path to image dave directory
    //std::string depth_path;                 ///< path to depth image directory
    //std::string imu_file;                   ///< path to imu data file
    std::string m_serial_number;            ///< Serial Number of Kinect Device
    k4a_device_t m_device;                  ///< Kinect Device handle
    k4a_calibration_t m_calibration;        ///< Kinect Calibration
    k4a_device_configuration_t m_config;    ///< Kinect Configuration


    bool ReadCalibration();
    bool ReadSerialNumber();

    // TODO these should probably be in Kinect Utils, they are generic static functions
    /**
     * @brief Extracts image data from kinect capture type
     * 
     * @param capture The capture type provided by Kinect SDK
     * @return img_buf_t a generic image buffer for passing image data
     */
    img_buf_t ExtractColourImg(k4a_capture_t capture);
    img_buf_t ExtractDepthImg(k4a_capture_t capture);
    img_buf_t ExtractIrImg(k4a_capture_t capture);
    /**
     * @brief 
     * 
     * @param imu_sample input IMU sample type provided by k4a SDK 
     * @return imu_data_t a generic data structure for accelerometer and gyro data
     */
    imu_data_t ExtractIMUData(k4a_imu_sample_t imu_sample);

public:
    Kinect(/* args */);
    ~Kinect();

    bool Connect();
    std::string GetSerialNumber();
    bool ConfigureDevice(/*std::string config_file*/);
    k4a_calibration_t GetCalibration();

    /**
     * @brief Functions to start and stop device capture
     * 
     * @return true Start/stop successsful
     * @return false Start/stop unsuccessful, potentially not connected or unconfigured
     */
    bool StartCapture();
    bool StopCapture();

    /**
     * @brief Get an image of format img_buf_t from Kinect
     * 
     * @param colour_img The colour image to extract
     * @param depth_img Depth image in 16 bit depth fomat (optional)
     * @param ir_img The Infrared image in 16 bit monochrome format (optional)
     * @param timeout_ms Timout 0ms is non-blocking (Note: issues with non-blocking)
     * @return true successfully read image
     * @return false failed to read image
     */
    bool GetImages(img_buf_t &colour_img, int timeout_ms = 0);
    bool GetImages(img_buf_t &colour_img, img_buf_t &depth_img, int timeout_ms = 0);
    bool GetImages(img_buf_t &colour_img, img_buf_t &depth_img, img_buf_t &ir_img, int timeout_ms = 0);

    bool GetIMU(imu_data_t &data, int timeout_ms = 0);

    /**
     * @brief Functions to print the calibration data from the Kinect
     */
    void PrintDepthCalibration();
    void PrintColourCalibration();
    void PrintExtrinsicCalibration();


};


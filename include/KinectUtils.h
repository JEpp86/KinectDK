/**
 * @file KinectUtils.h
 * @author Jason Epp (jasonepp0@gmail.com)
 * @brief Utilities related to data capture from Kinect Device
 * @version 0.1
 * @date 2022-08-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

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

/**
 * @brief Extracts image data from kinect capture type
 * 
 * @param capture The capture type provided by Kinect SDK
 * @return img_buf_t a generic image buffer for passing image data
 */
img_buf_t ExtractColourImg(k4a_capture_t capture);
img_buf_t ExtractRawDepthImg(k4a_capture_t capture);
img_buf_t ExtractIrImg(k4a_capture_t capture);

/**
 * @brief Creates Depth image from Colour Sensor perspective, creating Ground Truth Depth Image
 * 
 * @param depth_img_raw raw depth image from Kinect
 * @return k4a_image_t Kinect image type
 */
k4a_image_t TransformDepthToColour(k4a_image_t depth_img_raw,
                                          k4a_calibration_t cal,
                                          int width,
                                          int height,
                                          int stride);

std::pair<img_buf_t, img_buf_t> ExtractColourWithDepthGT(k4a_capture_t capture, 
                                                         k4a_calibration_t cal);

void SaveImageBuf(img_buf_t img, std::string filename);
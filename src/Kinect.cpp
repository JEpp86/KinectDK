/**
 * @file Kinect.cpp
 * @brief class to represent kinect device for streaming recording of sensor data
 * @author Jason Epp (jasonepp0@gmail.com)
 */

#include "Kinect.h"

#include <iostream>
#include <exception>
#include <k4a/k4a.h>

Kinect::Kinect(): m_connected(false)
                , m_configured(false)
                , m_capturing(false)
                , m_serial_number(""){
}

Kinect::~Kinect() {
    if (m_capturing) {
        this->StopCapture();
    }
    if (m_connected) {
        k4a_device_close(m_device);
        m_connected = false;
    }
}

/* Private Methods */
bool Kinect::ReadSerialNumber() {
    size_t serial_number_length = 0;
    char* serial_num = NULL;
    if (K4A_BUFFER_RESULT_TOO_SMALL != 
        k4a_device_get_serialnum(m_device, NULL, &serial_number_length)) {
        std::cout << "Failed to get serial number length" << std::endl;
        return false;
    }
    serial_num = (char*)malloc(serial_number_length);
    if (K4A_BUFFER_RESULT_SUCCEEDED != 
        k4a_device_get_serialnum(m_device, serial_num, &serial_number_length)) {
        std::cout << "Failed to get serial number" << std::endl;
        return false;
    }
    m_serial_number.assign(serial_num, serial_number_length);
    free(serial_num);
    return true;
}

bool Kinect::ReadCalibration() {
    if (!m_configured) {
        std::cout << "Device configuration not set" << std::endl;
        return false;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(m_device,
                                                           m_config.depth_mode,
                                                           m_config.color_resolution,
                                                           &m_calibration)) {
        std::cout << "Failed to get calibration" << std::endl;
        return false;
    }
    return true;
}

img_buf_t Kinect::ExtractColourImg(k4a_capture_t capture) {
    k4a_image_t img = k4a_capture_get_color_image(capture);
    img_buf_t image_buf;
    if (img == NULL) {
        image_buf.bits_per_pixel = 0;
        image_buf.img_height = 0;
        image_buf.img_width = 0;
        image_buf.timestamp = 0;
        image_buf.stride = 0;
    } else {
        uint8_t* tmp_buf = k4a_image_get_buffer(img);
        size_t im_size = k4a_image_get_size(img);
        image_buf.bits_per_pixel = 32;
        image_buf.data.assign(tmp_buf, tmp_buf + im_size);
        image_buf.timestamp = k4a_image_get_device_timestamp_usec(img);
        image_buf.img_width = k4a_image_get_width_pixels(img);
        image_buf.img_height = k4a_image_get_height_pixels(img);
        image_buf.stride = k4a_image_get_stride_bytes(img);
    }
    k4a_image_release(img);
    return image_buf;
}

img_buf_t Kinect::ExtractDepthImg(k4a_capture_t capture) {
    k4a_image_t img = k4a_capture_get_depth_image(capture);
    img_buf_t image_buf;
    if (img == NULL) {
        image_buf.bits_per_pixel = 0;
        image_buf.img_height = 0;
        image_buf.img_width = 0;
        image_buf.timestamp = 0;
        image_buf.stride = 0;
    } else {
        uint8_t* tmp_buf = k4a_image_get_buffer(img);
        size_t im_size = k4a_image_get_size(img);
        image_buf.bits_per_pixel = 16;
        image_buf.data.assign(tmp_buf, tmp_buf + im_size);
        image_buf.timestamp = k4a_image_get_device_timestamp_usec(img);
        image_buf.img_width = k4a_image_get_width_pixels(img);
        image_buf.img_height = k4a_image_get_height_pixels(img);
        image_buf.stride = k4a_image_get_stride_bytes(img);
    }
    k4a_image_release(img);
    return image_buf;
}

img_buf_t Kinect::ExtractIrImg(k4a_capture_t capture) {
    k4a_image_t img = k4a_capture_get_ir_image(capture);
    img_buf_t image_buf;
    if (img == NULL) {
        image_buf.bits_per_pixel = 0;
        image_buf.img_height = 0;
        image_buf.img_width = 0;
        image_buf.timestamp = 0;
        image_buf.stride = 0;
    } else {
        uint8_t* tmp_buf = k4a_image_get_buffer(img);
        size_t im_size = k4a_image_get_size(img);
        image_buf.bits_per_pixel = 16;
        image_buf.data.assign(tmp_buf, tmp_buf + im_size);
        image_buf.timestamp = k4a_image_get_device_timestamp_usec(img);
        image_buf.img_width = k4a_image_get_width_pixels(img);
        image_buf.img_height = k4a_image_get_height_pixels(img);
        image_buf.stride = k4a_image_get_stride_bytes(img);
    }
    k4a_image_release(img);
    return image_buf;
}

imu_data_t Kinect::ExtractIMUData(k4a_imu_sample_t imu_sample) {
    imu_data_t sample;
    sample.temperature = imu_sample.temperature;
    sample.acc_time = imu_sample.acc_timestamp_usec;
    sample.acc_x = imu_sample.acc_sample.xyz.x;
    sample.acc_y = imu_sample.acc_sample.xyz.y;
    sample.acc_z = imu_sample.acc_sample.xyz.z;
    sample.gy_time = imu_sample.gyro_timestamp_usec;
    sample.gy_x = imu_sample.gyro_sample.xyz.x;
    sample.gy_y = imu_sample.gyro_sample.xyz.y;
    sample.gy_z = imu_sample.gyro_sample.xyz.z;
    return sample;
}

/* Public Methods */
bool Kinect::Connect() {
    uint32_t device_count = k4a_device_get_installed_count();
    std::cout << "Devices Found: " << std::to_string(device_count) << std::endl;
    if (device_count == 0) {
        std::cout << "Error: No Kinect Devices Found" << std::endl;
        m_connected = false;
        return false;
    }
    // I only own one Kinect, so it hsould always be device 0
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(0, &m_device)) {
        std::cout << "Failed to open device" << std::endl;
        m_connected = false;
        return false;
    }
    m_connected = true;
    this->ReadSerialNumber();
    return true;
}

std::string Kinect::GetSerialNumber() {
    return m_serial_number;
}

bool Kinect::ConfigureDevice() {
    m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    m_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    m_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    m_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    m_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    m_config.synchronized_images_only = true;
    m_configured = true;
    bool success = this->ReadCalibration();
    return success;
}

bool Kinect::StartCapture() {
    if (!m_connected || !m_configured) {
        std::cout << "No device connectd or configured" << std::endl;
        return false;
    }
    if (m_capturing) {
        std::cout << "Already Capturing" << std::endl;
        return false;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_device, &m_config)) {
        std::cout << "Failed to start camera devices" << std::endl;
        return false;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(m_device)) {
        std::cout << "Failed to start IMU devices" << std::endl;
        return false;
    }
    m_capturing = true;
    return true;
}

bool Kinect::StopCapture() {
    if (!m_connected || !m_configured) {
        std::cout << "No device connectd or configured" << std::endl;
        return false;
    }
    if (!m_capturing) {
        std::cout << "Not capturing" << std::endl;
        return false;
    }
    k4a_device_stop_cameras(m_device);
    k4a_device_stop_imu(m_device);
    m_capturing = false;
    return true;
}

bool Kinect::GetImages(img_buf_t &colour_img, img_buf_t &depth_img, int timeout_ms) {
    // Capture a frame
    k4a_capture_t capture;
    switch (k4a_device_get_capture(m_device, &capture, timeout_ms))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        //std::cout << "Timed out waiting for a capture" << std::endl;
        return false;
    case K4A_WAIT_RESULT_FAILED:
        std::cout << "Failed to read a capture" << std::endl;
        //return false;
        throw std::runtime_error("Failed to read capture");
    }
    colour_img = this->ExtractColourImg(capture);
    depth_img = this->ExtractDepthImg(capture);
    k4a_capture_release(capture);
    return true;
}

bool Kinect::GetIMU(imu_data_t &data, int timeout_ms) {
    // Capture a imu sample
    k4a_imu_sample_t imu_sample;
    switch (k4a_device_get_imu_sample(m_device, &imu_sample, timeout_ms))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        //std::cout << "Timed out waiting for a imu sample" << std::endl;
        return false;
    case K4A_WAIT_RESULT_FAILED:
        std::cout << "Failed to read a imu sample" << std::endl;
        throw std::runtime_error("Failed to read IMU sample");
        //return false;
    }
    data = this->ExtractIMUData(imu_sample);
    return true;
}

void Kinect::PrintDepthCalibration() {
    if (!m_configured) {
        return;
    }
    auto calib = m_calibration.depth_camera_calibration;

    std::cout << "\n===== Device Depth Calibration =====\n";
    std::cout << "resolution width: " << calib.resolution_width << std::endl;
    std::cout << "resolution height: " << calib.resolution_height << std::endl;
    std::cout << "principal point x: " << calib.intrinsics.parameters.param.cx << std::endl;
    std::cout << "principal point y: " << calib.intrinsics.parameters.param.cy << std::endl;
    std::cout << "focal length x: " << calib.intrinsics.parameters.param.fx << std::endl;
    std::cout << "focal length y: " << calib.intrinsics.parameters.param.fy << std::endl;
    std::cout << "radial distortion coefficients:" << std::endl;
    std::cout << "k1: " << calib.intrinsics.parameters.param.k1 << std::endl;
    std::cout << "k2: " << calib.intrinsics.parameters.param.k2 << std::endl;
    std::cout << "k3: " << calib.intrinsics.parameters.param.k3 << std::endl;
    std::cout << "k4: " << calib.intrinsics.parameters.param.k4 << std::endl;
    std::cout << "k5: " << calib.intrinsics.parameters.param.k5 << std::endl;
    std::cout << "k6: " << calib.intrinsics.parameters.param.k6 << std::endl;
    std::cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << std::endl;
    std::cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << std::endl;
    std::cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << std::endl;
    std::cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << std::endl;
    std::cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << std::endl;

    return;
}

void Kinect::PrintColourCalibration() {
    if (!m_configured) {
        return;
    }
    auto calib = m_calibration.color_camera_calibration;

    std::cout << "\n===== Device Colour Calibration =====\n";
    std::cout << "resolution width: " << calib.resolution_width << std::endl;
    std::cout << "resolution height: " << calib.resolution_height << std::endl;
    std::cout << "principal point x: " << calib.intrinsics.parameters.param.cx << std::endl;
    std::cout << "principal point y: " << calib.intrinsics.parameters.param.cy << std::endl;
    std::cout << "focal length x: " << calib.intrinsics.parameters.param.fx << std::endl;
    std::cout << "focal length y: " << calib.intrinsics.parameters.param.fy << std::endl;
    std::cout << "radial distortion coefficients:" << std::endl;
    std::cout << "k1: " << calib.intrinsics.parameters.param.k1 << std::endl;
    std::cout << "k2: " << calib.intrinsics.parameters.param.k2 << std::endl;
    std::cout << "k3: " << calib.intrinsics.parameters.param.k3 << std::endl;
    std::cout << "k4: " << calib.intrinsics.parameters.param.k4 << std::endl;
    std::cout << "k5: " << calib.intrinsics.parameters.param.k5 << std::endl;
    std::cout << "k6: " << calib.intrinsics.parameters.param.k6 << std::endl;
    std::cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << std::endl;
    std::cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << std::endl;
    std::cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << std::endl;
    std::cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << std::endl;
    std::cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << std::endl;

    return;
}

void Kinect::PrintExtrinsicCalibration() {
        if (!m_configured) {
        return;
    }

    auto calib = m_calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH];
    std::cout << "\n===== Extrinsic: Colour Sensor to Depth Sensor  =====\n";
    std::cout << "Rotation: " << std::endl;
    std::cout << "|" << calib.rotation[0] << " " << calib.rotation[1] << " " << calib.rotation[2] << "|\n";
    std::cout << "|" << calib.rotation[3] << " " << calib.rotation[4] << " " << calib.rotation[5] << "|\n";
    std::cout << "|" << calib.rotation[6] << " " << calib.rotation[7] << " " << calib.rotation[8] << "|\n";
    std::cout << "Translation:" <<std::endl;
    std::cout << "|" << calib.translation[0] << " " << calib.translation[1] << " " << calib.translation[2] << "|\n";

    calib = m_calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_GYRO];
    std::cout << "\n===== Extrinsic: Colour Sensor to Gyro  =====\n";
    std::cout << "Rotation: " << std::endl;
    std::cout << "|" << calib.rotation[0] << " " << calib.rotation[1] << " " << calib.rotation[2] << "|\n";
    std::cout << "|" << calib.rotation[3] << " " << calib.rotation[4] << " " << calib.rotation[5] << "|\n";
    std::cout << "|" << calib.rotation[6] << " " << calib.rotation[7] << " " << calib.rotation[8] << "|\n";
    std::cout << "Translation:" <<std::endl;
    std::cout << "|" << calib.translation[0] << " " << calib.translation[1] << " " << calib.translation[2] << "|\n";

    calib = m_calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_ACCEL];
    std::cout << "\n===== Extrinsic: Colour Sensor to Accelerometer  =====\n";
    std::cout << "Rotation: " << std::endl;
    std::cout << "|" << calib.rotation[0] << " " << calib.rotation[1] << " " << calib.rotation[2] << "|\n";
    std::cout << "|" << calib.rotation[3] << " " << calib.rotation[4] << " " << calib.rotation[5] << "|\n";
    std::cout << "|" << calib.rotation[6] << " " << calib.rotation[7] << " " << calib.rotation[8] << "|\n";
    std::cout << "Translation:" <<std::endl;
    std::cout << "|" << calib.translation[0] << " " << calib.translation[1] << " " << calib.translation[2] << "|\n";

    return;
}


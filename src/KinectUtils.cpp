#include "KinectUtils.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

img_buf_t ExtractColourImg(k4a_capture_t capture) {
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

img_buf_t ExtractRawDepthImg(k4a_capture_t capture) {
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

img_buf_t ExtractIrImg(k4a_capture_t capture) {
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

std::pair<img_buf_t, img_buf_t> ExtractColourWithDepthGT(k4a_capture_t capture,
                                                         k4a_calibration_t cal) {
    k4a_image_t img = k4a_capture_get_color_image(capture);
    std::pair<img_buf_t, img_buf_t> ret;
    ret.first.bits_per_pixel = 0;
    ret.first.img_height = 0;
    ret.first.img_width = 0;
    ret.first.timestamp = 0;
    ret.first.stride = 0;
    ret.second = ret.first;
    if (img == NULL) {
        return ret;
    } else {
        uint8_t* tmp_buf = k4a_image_get_buffer(img);
        size_t im_size = k4a_image_get_size(img);
        ret.first.bits_per_pixel = 32;
        ret.first.data.assign(tmp_buf, tmp_buf + im_size);
        ret.first.timestamp = k4a_image_get_device_timestamp_usec(img);
        ret.first.img_width = k4a_image_get_width_pixels(img);
        ret.first.img_height = k4a_image_get_height_pixels(img);
        ret.first.stride = k4a_image_get_stride_bytes(img);
    }
    int width = k4a_image_get_width_pixels(img);
    int height = k4a_image_get_height_pixels(img);
    // done with the first image
    k4a_image_release(img);
    // Get raw depth image
    k4a_image_t depth_img = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                width,
                                                height,
                                                width * (int)sizeof(uint16_t),
                                                &depth_img)) {
        std::cout << "DepthTransform: Image Container Creation Failed" << std::endl;
        return ret;
    }
    k4a_image_t d_img_raw = k4a_capture_get_depth_image(capture);
    k4a_transformation_t transformation(k4a_transformation_create(&cal));
    if (d_img_raw == NULL) {
        std::cout << "Depth image read failed" << std::endl;
        return ret;
    } else if (transformation == NULL) {
        k4a_image_release(d_img_raw);
        std::cout << "Transformation read failed" << std::endl;
        return ret;
    } else if (K4A_RESULT_SUCCEEDED !=
               k4a_transformation_depth_image_to_color_camera(transformation, d_img_raw, depth_img)) {
        k4a_transformation_destroy(transformation);
        k4a_image_release(d_img_raw);
        std::cout << "Transformation Failed" << std::endl;
        return ret;
    } else { // Success
        uint8_t* tmp_buf = k4a_image_get_buffer(depth_img);
        size_t im_size = k4a_image_get_size(depth_img);
        ret.second.bits_per_pixel = 16;
        ret.second.data.assign(tmp_buf, tmp_buf + im_size);
        ret.second.timestamp = k4a_image_get_device_timestamp_usec(d_img_raw);
        ret.second.img_width = k4a_image_get_width_pixels(depth_img);
        ret.second.img_height = k4a_image_get_height_pixels(depth_img);
        ret.second.stride = k4a_image_get_stride_bytes(depth_img);
    }
    k4a_transformation_destroy(transformation);
    k4a_image_release(d_img_raw);
    k4a_image_release(depth_img);
    return ret;
}

void SaveImageBuf(img_buf_t img, std::string filename) {
    cv::Mat image(cv::Size(img.img_width, img.img_height), CV_16UC1, img.data.data(), cv::Mat::AUTO_STEP);
    cv::imwrite(filename, image);
}
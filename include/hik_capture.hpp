#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

struct HikCameraConfig
{
    bool has_exposure = false;
    bool has_gain = false;
    bool has_width = false;
    bool has_height = false;
    float exposure_us = 0.0F;
    float gain = 0.0F;
    unsigned int width = 0;
    unsigned int height = 0;
};

struct HikDeviceInfo
{
    unsigned int index = 0;
    std::string transport;
    std::string model;
    std::string serial;
    std::string name;
    std::string ip;
    bool accessible = false;
};

struct HikFrame
{
    cv::Mat image;
    unsigned int frame_number = 0;
    std::uint64_t hardware_timestamp = 0;
    int pixel_type = 0;
};

class HikCapture
{
public:
    HikCapture();
    ~HikCapture();

    HikCapture(const HikCapture&) = delete;
    HikCapture& operator=(const HikCapture&) = delete;

    void refreshDevices();
    const std::vector<HikDeviceInfo>& devices() const;

    void open(unsigned int index, const HikCameraConfig& config);
    bool grab(HikFrame& frame, int timeout_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace auto_aim

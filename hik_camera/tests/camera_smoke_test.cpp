#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include "hik_camera.hpp"

namespace
{

constexpr std::size_t kMinimumFrames = 30;
constexpr std::uint64_t kMinimumSpanNs = 3'000'000'000ULL;
constexpr auto kTestDeadline = std::chrono::seconds(10);
constexpr unsigned int kCaptureTimeoutMs = 500;
constexpr double kMinimumRSquared = 0.999;

struct TimestampSample
{
    std::uint64_t deviceTick = 0;
    std::uint64_t hostReceiveNs = 0;
};

struct TimestampFit
{
    double tickToNanoseconds = 0.0;
    double tickFrequencyHz = 0.0;
    double rSquared = 0.0;
};

TimestampFit fitTimestampScale(const std::vector<TimestampSample>& samples)
{
    if (samples.size() < 2)
    {
        throw std::runtime_error("timestamp calibration needs at least two samples");
    }

    double meanX = 0.0;
    double meanY = 0.0;
    for (const TimestampSample& sample : samples)
    {
        meanX += static_cast<double>(sample.deviceTick - samples.front().deviceTick);
        meanY += static_cast<double>(sample.hostReceiveNs - samples.front().hostReceiveNs);
    }
    meanX /= static_cast<double>(samples.size());
    meanY /= static_cast<double>(samples.size());

    double covariance = 0.0;
    double varianceX = 0.0;
    double varianceY = 0.0;
    for (const TimestampSample& sample : samples)
    {
        const double x = static_cast<double>(sample.deviceTick - samples.front().deviceTick);
        const double y = static_cast<double>(sample.hostReceiveNs - samples.front().hostReceiveNs);
        covariance += (x - meanX) * (y - meanY);
        varianceX += (x - meanX) * (x - meanX);
        varianceY += (y - meanY) * (y - meanY);
    }
    if (varianceX <= 0.0 || varianceY <= 0.0)
    {
        throw std::runtime_error("timestamp samples have zero span");
    }

    TimestampFit fit;
    fit.tickToNanoseconds = covariance / varianceX;
    fit.rSquared = (covariance * covariance) / (varianceX * varianceY);
    fit.tickFrequencyHz = 1e9 / fit.tickToNanoseconds;
    if (!std::isfinite(fit.tickToNanoseconds) || fit.tickToNanoseconds <= 0.0 ||
        !std::isfinite(fit.tickFrequencyHz) || fit.tickFrequencyHz <= 0.0 ||
        !std::isfinite(fit.rSquared))
    {
        throw std::runtime_error("timestamp fit produced a non-finite or non-positive scale");
    }
    return fit;
}

bool saveCalibration(
    const TimestampFit& fit,
    std::size_t sampleCount,
    double sampleSpanSeconds,
    const std::string& outputPath)
{
    const std::filesystem::path output(outputPath);
    const std::filesystem::path temporary = output.string() + ".tmp";
    try
    {
        std::filesystem::create_directories(output.parent_path());
        cv::FileStorage storage(
            temporary.string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
        if (!storage.isOpened())
        {
            throw std::runtime_error("cannot open temporary calibration file");
        }
        storage << "tick_to_nanoseconds" << fit.tickToNanoseconds;
        storage << "tick_frequency_hz" << fit.tickFrequencyHz;
        storage << "sample_count" << static_cast<int>(sampleCount);
        storage << "sample_span_seconds" << sampleSpanSeconds;
        storage << "fit_r_squared" << fit.rSquared;
        storage.release();
        std::filesystem::rename(temporary, output);
        return true;
    }
    catch (const std::exception& ex)
    {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        std::cerr << "warning: timestamp calibration was not saved: " << ex.what() << '\n';
        return false;
    }
}

int run()
{
    auto_aim::hik_camera::HikCameraOptions options;
    options.timestampMode = auto_aim::hik_camera::HikTimestampMode::RawOnly;
    auto_aim::hik_camera::HikCamera camera(options);

    std::vector<TimestampSample> samples;
    cv::Size imageSize;
    unsigned int firstFrameNumber = 0;
    unsigned int previousFrameNumber = 0;
    std::uint64_t previousDeviceTick = 0;
    std::uint64_t previousHostReceiveNs = 0;
    const auto deadline = std::chrono::steady_clock::now() + kTestDeadline;

    while (std::chrono::steady_clock::now() < deadline)
    {
        auto_aim::hik_camera::HikCameraFrame frame;
        const int result = camera.capture(frame, kCaptureTimeoutMs);
        if (result == static_cast<int>(MV_E_NODATA))
        {
            continue;
        }
        if (result != MV_OK)
        {
            throw std::runtime_error(
                "camera capture failed with MV code 0x" +
                cv::format("%08x", static_cast<unsigned int>(result)));
        }
        if (frame.image.empty() || frame.image.type() != CV_8UC3)
        {
            throw std::runtime_error("camera returned an empty or non-CV_8UC3 image");
        }
        if (samples.empty())
        {
            imageSize = frame.image.size();
            firstFrameNumber = frame.frameNumber;
        }
        else
        {
            if (frame.image.size() != imageSize)
            {
                throw std::runtime_error("camera image dimensions changed during the test");
            }
            if (static_cast<std::int32_t>(frame.frameNumber - previousFrameNumber) <= 0)
            {
                throw std::runtime_error("camera frame number did not advance");
            }
            if (frame.hardwareTimestamp <= previousDeviceTick)
            {
                throw std::runtime_error("camera device timestamp did not advance");
            }
            if (frame.hostReceiveTimestampNs <= previousHostReceiveNs)
            {
                throw std::runtime_error("host receive timestamp did not advance");
            }
        }

        samples.push_back({frame.hardwareTimestamp, frame.hostReceiveTimestampNs});
        previousFrameNumber = frame.frameNumber;
        previousDeviceTick = frame.hardwareTimestamp;
        previousHostReceiveNs = frame.hostReceiveTimestampNs;

        const std::uint64_t spanNs = samples.back().hostReceiveNs - samples.front().hostReceiveNs;
        if (samples.size() >= kMinimumFrames && spanNs >= kMinimumSpanNs)
        {
            break;
        }
    }

    if (samples.size() < kMinimumFrames)
    {
        throw std::runtime_error(
            "camera smoke test timed out after receiving only " + std::to_string(samples.size()) + " frames");
    }
    const std::uint64_t spanNs = samples.back().hostReceiveNs - samples.front().hostReceiveNs;
    if (spanNs < kMinimumSpanNs)
    {
        throw std::runtime_error("camera smoke test did not reach the required 3-second sample span");
    }

    const double spanSeconds = static_cast<double>(spanNs) * 1e-9;
    const double frameRate = static_cast<double>(samples.size() - 1) / spanSeconds;
    std::cout << "smoke PASS: frames=" << samples.size()
              << " size=" << imageSize.width << 'x' << imageSize.height
              << " first_frame=" << firstFrameNumber
              << " last_frame=" << previousFrameNumber
              << " span_s=" << std::fixed << std::setprecision(6) << spanSeconds
              << " effective_fps=" << std::setprecision(2) << frameRate << '\n';

    try
    {
        const TimestampFit fit = fitTimestampScale(samples);
        std::cout << std::setprecision(12)
                  << "timestamp fit: tick_to_nanoseconds=" << fit.tickToNanoseconds
                  << " tick_frequency_hz=" << fit.tickFrequencyHz
                  << " r_squared=" << fit.rSquared << '\n';
        if (fit.rSquared < kMinimumRSquared)
        {
            std::cerr << "warning: timestamp calibration rejected: r_squared=" << fit.rSquared
                      << " < " << kMinimumRSquared << "; existing config was kept\n";
            return 0;
        }
        if (saveCalibration(
                fit,
                samples.size(),
                spanSeconds,
                options.timestampCalibrationPath))
        {
            std::cout << "timestamp calibration saved: "
                      << options.timestampCalibrationPath << '\n';
        }
    }
    catch (const std::exception& ex)
    {
        std::cerr << "warning: timestamp calibration failed: " << ex.what()
                  << "; existing config was kept\n";
    }
    return 0;
}

}

int main()
try
{
    return run();
}
catch (const std::exception& ex)
{
    std::cerr << "smoke FAIL: " << ex.what() << '\n';
    return 1;
}

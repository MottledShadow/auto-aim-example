#include "app_options.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace auto_aim
{
namespace
{

bool startsWith(const std::string& text, const std::string& prefix)
{
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

std::string takeValue(int& i, int argc, char** argv, const std::string& arg, const std::string& key)
{
    const std::string with_equals = key + "=";
    if (startsWith(arg, with_equals))
    {
        return arg.substr(with_equals.size());
    }
    if (arg == key && i + 1 < argc)
    {
        return argv[++i];
    }
    throw std::runtime_error("missing value for " + key);
}

unsigned int parseUInt(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    unsigned long parsed = std::stoul(value, &pos, 10);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid integer for " + key + ": " + value);
    }
    return static_cast<unsigned int>(parsed);
}

int parseByte(const std::string& value, const std::string& key)
{
    const unsigned int parsed = parseUInt(value, key);
    if (parsed > 255)
    {
        throw std::runtime_error("invalid threshold for " + key + ": " + value);
    }
    return static_cast<int>(parsed);
}

float parseFloat(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    float parsed = std::stof(value, &pos);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid number for " + key + ": " + value);
    }
    return parsed;
}

double parseDouble(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    double parsed = std::stod(value, &pos);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid number for " + key + ": " + value);
    }
    return parsed;
}

} // namespace

void printUsage(const char* exe)
{
    std::cout
        << "Usage: " << exe << " [options]\n\n"
        << "Options:\n"
        << "  --list                    list cameras and exit\n"
        << "  --index N                 camera index, default 0\n"
        << "  --frames N                number of frames to grab, 0 means until Ctrl-C\n"
        << "  --timeout-ms N            frame timeout, default 1000\n"
        << "  --output DIR              save PNG frames to DIR, default captures\n"
        << "  --no-save                 grab/preview without writing images\n"
        << "  --show                    show live OpenCV preview, press q or Esc to quit\n"
        << "  --show-binary             show preprocessed binary image instead of raw preview\n"
        << "  --binary-threshold N      threshold for bright region extraction, default 180\n"
        << "  --open-kernel N           opening kernel size, 0 disables opening, default 3\n"
        << "  --close-kernel N          closing kernel size, 0 disables closing, default 3\n"
        << "  --morph-iterations N      opening/closing iterations, default 1\n"
        << "  --min-light-area VALUE    minimum light bar foreground area, default 5\n"
        << "  --max-light-area VALUE    maximum light bar foreground area, default 1000000\n"
        << "  --min-light-aspect VALUE  minimum long-side/short-side ratio, default 1.2\n"
        << "  --max-light-aspect VALUE  maximum long-side/short-side ratio, default 50\n"
        << "  --min-light-angle VALUE   minimum fitLine tilt from vertical, default 0\n"
        << "  --max-light-angle VALUE   maximum fitLine tilt from vertical, default 45\n"
        << "  --min-light-fill VALUE    minimum area/minAreaRect-area ratio, default 0.25\n"
        << "  --max-light-fill VALUE    maximum area/minAreaRect-area ratio, default 1\n"
        << "  --max-armor-length-ratio VALUE  maximum paired light length ratio, default 2\n"
        << "  --max-armor-angle-diff VALUE    maximum paired light angle difference, default 10\n"
        << "  --max-armor-y-diff VALUE        maximum paired light center y difference, default 40\n"
        << "  --min-armor-distance-ratio VALUE minimum center distance / average height, default 0.5\n"
        << "  --max-armor-distance-ratio VALUE maximum center distance / average height, default 8\n"
        << "  --exposure-us VALUE       set manual exposure time in microseconds\n"
        << "  --gain VALUE              set manual gain\n"
        << "  --width N                 set camera Width before grabbing\n"
        << "  --height N                set camera Height before grabbing\n"
        << "  --help                    show this help\n\n"
        << "Examples:\n"
        << "  " << exe << " --list\n"
        << "  " << exe << " --index 0 --frames 10 --output captures\n"
        << "  " << exe << " --index 0 --frames 0 --show --no-save --exposure-us 3000\n";
}

AppOptions parseArgs(int argc, char** argv)
{
    AppOptions options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--list")
        {
            options.list_only = true;
        }
        else if (arg == "--no-save")
        {
            options.save = false;
        }
        else if (arg == "--show")
        {
            options.show = true;
        }
        else if (arg == "--show-binary")
        {
            options.show_binary = true;
            options.show = true;
        }
        else if (arg == "--index" || startsWith(arg, "--index="))
        {
            options.index = parseUInt(takeValue(i, argc, argv, arg, "--index"), "--index");
        }
        else if (arg == "--frames" || startsWith(arg, "--frames="))
        {
            options.frames = parseUInt(takeValue(i, argc, argv, arg, "--frames"), "--frames");
        }
        else if (arg == "--timeout-ms" || startsWith(arg, "--timeout-ms="))
        {
            options.timeout_ms = static_cast<int>(parseUInt(takeValue(i, argc, argv, arg, "--timeout-ms"), "--timeout-ms"));
        }
        else if (arg == "--output" || startsWith(arg, "--output="))
        {
            options.output_dir = takeValue(i, argc, argv, arg, "--output");
        }
        else if (arg == "--binary-threshold" || startsWith(arg, "--binary-threshold="))
        {
            options.preprocess.binary_threshold = parseByte(
                takeValue(i, argc, argv, arg, "--binary-threshold"),
                "--binary-threshold");
        }
        else if (arg == "--open-kernel" || startsWith(arg, "--open-kernel="))
        {
            options.preprocess.open_kernel_size = static_cast<int>(
                parseUInt(takeValue(i, argc, argv, arg, "--open-kernel"), "--open-kernel"));
        }
        else if (arg == "--close-kernel" || startsWith(arg, "--close-kernel="))
        {
            options.preprocess.close_kernel_size = static_cast<int>(
                parseUInt(takeValue(i, argc, argv, arg, "--close-kernel"), "--close-kernel"));
        }
        else if (arg == "--morph-iterations" || startsWith(arg, "--morph-iterations="))
        {
            options.preprocess.morph_iterations = static_cast<int>(
                parseUInt(takeValue(i, argc, argv, arg, "--morph-iterations"), "--morph-iterations"));
        }
        else if (arg == "--min-light-area" || startsWith(arg, "--min-light-area="))
        {
            options.light_filter.min_area = parseDouble(takeValue(i, argc, argv, arg, "--min-light-area"), "--min-light-area");
        }
        else if (arg == "--max-light-area" || startsWith(arg, "--max-light-area="))
        {
            options.light_filter.max_area = parseDouble(takeValue(i, argc, argv, arg, "--max-light-area"), "--max-light-area");
        }
        else if (arg == "--min-light-aspect" || startsWith(arg, "--min-light-aspect="))
        {
            options.light_filter.min_aspect_ratio = parseDouble(takeValue(i, argc, argv, arg, "--min-light-aspect"), "--min-light-aspect");
        }
        else if (arg == "--max-light-aspect" || startsWith(arg, "--max-light-aspect="))
        {
            options.light_filter.max_aspect_ratio = parseDouble(takeValue(i, argc, argv, arg, "--max-light-aspect"), "--max-light-aspect");
        }
        else if (arg == "--min-light-angle" || startsWith(arg, "--min-light-angle="))
        {
            options.light_filter.min_line_angle_deg = parseDouble(takeValue(i, argc, argv, arg, "--min-light-angle"), "--min-light-angle");
        }
        else if (arg == "--max-light-angle" || startsWith(arg, "--max-light-angle="))
        {
            options.light_filter.max_line_angle_deg = parseDouble(takeValue(i, argc, argv, arg, "--max-light-angle"), "--max-light-angle");
        }
        else if (arg == "--min-light-fill" || startsWith(arg, "--min-light-fill="))
        {
            options.light_filter.min_fill_ratio = parseDouble(takeValue(i, argc, argv, arg, "--min-light-fill"), "--min-light-fill");
        }
        else if (arg == "--max-light-fill" || startsWith(arg, "--max-light-fill="))
        {
            options.light_filter.max_fill_ratio = parseDouble(takeValue(i, argc, argv, arg, "--max-light-fill"), "--max-light-fill");
        }
        else if (arg == "--max-armor-length-ratio" || startsWith(arg, "--max-armor-length-ratio="))
        {
            options.armor_matcher.max_light_length_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-length-ratio"),
                "--max-armor-length-ratio");
        }
        else if (arg == "--max-armor-angle-diff" || startsWith(arg, "--max-armor-angle-diff="))
        {
            options.armor_matcher.max_light_angle_diff_deg = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-angle-diff"),
                "--max-armor-angle-diff");
        }
        else if (arg == "--max-armor-y-diff" || startsWith(arg, "--max-armor-y-diff="))
        {
            options.armor_matcher.max_light_center_y_diff = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-y-diff"),
                "--max-armor-y-diff");
        }
        else if (arg == "--min-armor-distance-ratio" || startsWith(arg, "--min-armor-distance-ratio="))
        {
            options.armor_matcher.min_center_distance_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--min-armor-distance-ratio"),
                "--min-armor-distance-ratio");
        }
        else if (arg == "--max-armor-distance-ratio" || startsWith(arg, "--max-armor-distance-ratio="))
        {
            options.armor_matcher.max_center_distance_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-distance-ratio"),
                "--max-armor-distance-ratio");
        }
        else if (arg == "--exposure-us" || startsWith(arg, "--exposure-us="))
        {
            options.camera.has_exposure = true;
            options.camera.exposure_us = parseFloat(takeValue(i, argc, argv, arg, "--exposure-us"), "--exposure-us");
        }
        else if (arg == "--gain" || startsWith(arg, "--gain="))
        {
            options.camera.has_gain = true;
            options.camera.gain = parseFloat(takeValue(i, argc, argv, arg, "--gain"), "--gain");
        }
        else if (arg == "--width" || startsWith(arg, "--width="))
        {
            options.camera.has_width = true;
            options.camera.width = parseUInt(takeValue(i, argc, argv, arg, "--width"), "--width");
        }
        else if (arg == "--height" || startsWith(arg, "--height="))
        {
            options.camera.has_height = true;
            options.camera.height = parseUInt(takeValue(i, argc, argv, arg, "--height"), "--height");
        }
        else
        {
            throw std::runtime_error("unknown option: " + arg);
        }
    }
    return options;
}

} // namespace auto_aim

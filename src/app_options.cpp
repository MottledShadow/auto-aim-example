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
        << "  --help                    show this help\n\n"
        << "Examples:\n"
        << "  " << exe << " --list\n"
        << "  " << exe << " --index 0 --frames 10 --output captures\n"
        << "  " << exe << " --index 0 --frames 0 --show --no-save\n";
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
        else
        {
            throw std::runtime_error("unknown option: " + arg);
        }
    }
    return options;
}

} // namespace auto_aim

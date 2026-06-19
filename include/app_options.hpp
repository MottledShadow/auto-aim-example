#pragma once

#include <string>

namespace auto_aim
{

struct AppOptions
{
    unsigned int index = 0;
    unsigned int frames = 1;
    int timeout_ms = 1000;
    std::string output_dir = "captures";
    bool list_only = false;
    bool save = true;
    bool show = false;
    bool show_binary = false;
};

AppOptions parseArgs(int argc, char** argv);
void printUsage(const char* exe);

} // namespace auto_aim

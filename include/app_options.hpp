#pragma once

#include <string>

#include "armor_matcher.hpp"
#include "armor_preprocessor.hpp"
#include "hik_capture.hpp"
#include "light_bar_filter.hpp"
#include "pnp_solver.hpp"

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
    HikCameraConfig camera;
    ArmorPreprocessParams preprocess;
    LightBarFilterParams light_filter;
    ArmorMatcherParams armor_matcher;
    PnpSolverParams pnp;
};

AppOptions parseArgs(int argc, char** argv);
void printUsage(const char* exe);

} // namespace auto_aim

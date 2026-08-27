#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "lightbar_detector.hpp"

namespace auto_aim::detector::tuning
{

struct Annotation
{
    std::string filename;
    LightColor targetColor = LightColor::Unknown;
    cv::Rect2f targetBox;
};

struct ParameterSet
{
    int binaryThreshold = 90;
    LightBarFilterParams filter;
    LightBarMatcherParams matcher;
};

struct SearchRanges
{
    int binaryThresholdMin = 50;
    int binaryThresholdMax = 150;
    double minAreaMin = 5.0;
    double minAreaMax = 40.0;
    double maxAreaMin = 3000.0;
    double maxAreaMax = 10000.0;
    double minAspectRatioMin = 1.5;
    double minAspectRatioMax = 3.5;
    double maxAspectRatioMin = 8.0;
    double maxAspectRatioMax = 20.0;
    double maxLineAngleDegMin = 8.0;
    double maxLineAngleDegMax = 30.0;
    double minFillRatioMin = 0.25;
    double minFillRatioMax = 0.75;
    double maxLightLengthRatioMin = 1.2;
    double maxLightLengthRatioMax = 2.0;
    double maxLightAngleDiffDegMin = 4.0;
    double maxLightAngleDiffDegMax = 20.0;
    double maxLightCenterYDiffMin = 10.0;
    double maxLightCenterYDiffMax = 80.0;
    double minCenterDistanceRatioMin = 1.2;
    double minCenterDistanceRatioMax = 3.0;
    double maxCenterDistanceRatioMin = 4.5;
    double maxCenterDistanceRatioMax = 8.0;
};

struct ImageOutcome
{
    int truePositives = 0;
    int falsePositives = 0;
    int falseNegatives = 0;
    int matchedIndex = -1;
    std::size_t candidateCount = 0;
    std::size_t lightBarCount = 0;
    std::size_t armorCount = 0;
};

struct Metrics
{
    int truePositives = 0;
    int falsePositives = 0;
    int falseNegatives = 0;
    std::size_t imageCount = 0;
    std::size_t totalCandidateCount = 0;
    std::size_t totalLightBarCount = 0;
    std::size_t totalArmorCount = 0;
    double precision = 0.0;
    double recall = 0.0;
    double f1 = 0.0;
    double meanCandidateCount = 0.0;
    double meanLightBarCount = 0.0;
    double meanArmorCount = 0.0;
};

struct StabilityMetrics
{
    double worstF1 = 0.0;
};

bool naturalLess(const std::string& lhs, const std::string& rhs);

std::string colorName(LightColor color);
LightColor parseColor(const std::string& value);

std::vector<Annotation> loadAnnotations(const std::string& path);
void saveAnnotations(const std::string& path, const std::vector<Annotation>& annotations);
void validateAnnotation(const Annotation& annotation, const cv::Size& imageSize = {});
std::vector<std::string> annotationWorklist(
    const std::vector<std::string>& filenames,
    const std::vector<Annotation>& annotations,
    bool force);
void upsertAnnotation(std::vector<Annotation>& annotations, const Annotation& annotation);

ImageOutcome evaluateImage(
    const Annotation& annotation,
    const std::vector<Armor>& armors,
    std::size_t candidateCount = 0,
    std::size_t lightBarCount = 0);

void addOutcome(Metrics& metrics, const ImageOutcome& outcome);
void finalizeMetrics(Metrics& metrics);
bool betterMetrics(const Metrics& lhs, const Metrics& rhs);

ParameterSet defaultParameters();
std::string parameterKey(const ParameterSet& parameters);
double distanceFromDefaults(const ParameterSet& parameters);

std::vector<ParameterSet> sampleParameterSets(
    std::size_t randomCount,
    std::uint32_t seed,
    const SearchRanges& ranges = {});

using Evaluator = std::function<Metrics(const ParameterSet&)>;

std::vector<ParameterSet> locallyRefine(
    const std::vector<ParameterSet>& seeds,
    const Evaluator& evaluator,
    const SearchRanges& ranges = {},
    const std::vector<double>& relativeSteps = {0.10, 0.05});

StabilityMetrics evaluateStability(
    const ParameterSet& parameters,
    const Evaluator& evaluator,
    const SearchRanges& ranges = {},
    double relativeStep = 0.02);

} // namespace auto_aim::detector::tuning

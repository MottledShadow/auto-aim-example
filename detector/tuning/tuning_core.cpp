#include "tuning_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>

namespace auto_aim::detector::tuning
{
namespace
{

constexpr double kComparisonEpsilon = 1e-12;
constexpr std::size_t kParameterCount = 12;

double parseDouble(const std::string& value, const std::string& field, std::size_t line)
{
    std::size_t consumed = 0;
    double result = 0.0;
    try
    {
        result = std::stod(value, &consumed);
    }
    catch (const std::exception&)
    {
        throw std::runtime_error(
            "annotation line " + std::to_string(line) + ": invalid " + field);
    }
    if (consumed != value.size() || !std::isfinite(result))
    {
        throw std::runtime_error(
            "annotation line " + std::to_string(line) + ": invalid " + field);
    }
    return result;
}

std::vector<std::string> splitCsvLine(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ','))
    {
        fields.push_back(field);
    }
    if (!line.empty() && line.back() == ',')
    {
        fields.emplace_back();
    }
    return fields;
}

double uniform(std::mt19937& generator, double minimum, double maximum)
{
    return std::uniform_real_distribution<double>(minimum, maximum)(generator);
}

double clampValue(double value, double minimum, double maximum)
{
    return std::clamp(value, minimum, maximum);
}

ParameterSet adjustedParameter(
    const ParameterSet& source,
    std::size_t index,
    double signedRelativeStep,
    const SearchRanges& ranges)
{
    ParameterSet result = source;
    const auto adjust = [signedRelativeStep](double value, double minimum, double maximum) {
        return clampValue(value + signedRelativeStep * (maximum - minimum), minimum, maximum);
    };

    switch (index)
    {
    case 0:
        result.binaryThreshold = std::clamp(
            static_cast<int>(std::lround(
                static_cast<double>(source.binaryThreshold) +
                signedRelativeStep * (ranges.binaryThresholdMax - ranges.binaryThresholdMin))),
            ranges.binaryThresholdMin,
            ranges.binaryThresholdMax);
        break;
    case 1:
        result.filter.minArea = adjust(source.filter.minArea, ranges.minAreaMin, ranges.minAreaMax);
        break;
    case 2:
        result.filter.maxArea = adjust(source.filter.maxArea, ranges.maxAreaMin, ranges.maxAreaMax);
        break;
    case 3:
        result.filter.minAspectRatio = adjust(
            source.filter.minAspectRatio,
            ranges.minAspectRatioMin,
            ranges.minAspectRatioMax);
        break;
    case 4:
        result.filter.maxAspectRatio = adjust(
            source.filter.maxAspectRatio,
            ranges.maxAspectRatioMin,
            ranges.maxAspectRatioMax);
        break;
    case 5:
        result.filter.maxLineAngleDeg = adjust(
            source.filter.maxLineAngleDeg,
            ranges.maxLineAngleDegMin,
            ranges.maxLineAngleDegMax);
        break;
    case 6:
        result.filter.minFillRatio = adjust(
            source.filter.minFillRatio,
            ranges.minFillRatioMin,
            ranges.minFillRatioMax);
        break;
    case 7:
        result.matcher.maxLightLengthRatio = adjust(
            source.matcher.maxLightLengthRatio,
            ranges.maxLightLengthRatioMin,
            ranges.maxLightLengthRatioMax);
        break;
    case 8:
        result.matcher.maxLightAngleDiffDeg = adjust(
            source.matcher.maxLightAngleDiffDeg,
            ranges.maxLightAngleDiffDegMin,
            ranges.maxLightAngleDiffDegMax);
        break;
    case 9:
        result.matcher.maxLightCenterYDiff = adjust(
            source.matcher.maxLightCenterYDiff,
            ranges.maxLightCenterYDiffMin,
            ranges.maxLightCenterYDiffMax);
        break;
    case 10:
        result.matcher.minCenterDistanceRatio = adjust(
            source.matcher.minCenterDistanceRatio,
            ranges.minCenterDistanceRatioMin,
            ranges.minCenterDistanceRatioMax);
        break;
    case 11:
        result.matcher.maxCenterDistanceRatio = adjust(
            source.matcher.maxCenterDistanceRatio,
            ranges.maxCenterDistanceRatioMin,
            ranges.maxCenterDistanceRatioMax);
        break;
    default:
        throw std::out_of_range("parameter index out of range");
    }
    return result;
}

double normalizedDistance(double value, double reference, double scale)
{
    return std::abs(value - reference) / scale;
}

} // namespace

bool naturalLess(const std::string& lhs, const std::string& rhs)
{
    std::size_t left = 0;
    std::size_t right = 0;
    while (left < lhs.size() && right < rhs.size())
    {
        const bool leftDigit = std::isdigit(static_cast<unsigned char>(lhs[left])) != 0;
        const bool rightDigit = std::isdigit(static_cast<unsigned char>(rhs[right])) != 0;
        if (leftDigit && rightDigit)
        {
            std::size_t leftEnd = left;
            std::size_t rightEnd = right;
            while (leftEnd < lhs.size() && std::isdigit(static_cast<unsigned char>(lhs[leftEnd])))
            {
                ++leftEnd;
            }
            while (rightEnd < rhs.size() && std::isdigit(static_cast<unsigned char>(rhs[rightEnd])))
            {
                ++rightEnd;
            }
            const std::string leftDigits = lhs.substr(left, leftEnd - left);
            const std::string rightDigits = rhs.substr(right, rightEnd - right);
            const auto trimZeros = [](const std::string& value) {
                const std::size_t first = value.find_first_not_of('0');
                return first == std::string::npos ? std::string("0") : value.substr(first);
            };
            const std::string leftNumber = trimZeros(leftDigits);
            const std::string rightNumber = trimZeros(rightDigits);
            if (leftNumber.size() != rightNumber.size())
            {
                return leftNumber.size() < rightNumber.size();
            }
            if (leftNumber != rightNumber)
            {
                return leftNumber < rightNumber;
            }
            if (leftDigits.size() != rightDigits.size())
            {
                return leftDigits.size() < rightDigits.size();
            }
            left = leftEnd;
            right = rightEnd;
            continue;
        }
        if (lhs[left] != rhs[right])
        {
            return lhs[left] < rhs[right];
        }
        ++left;
        ++right;
    }
    return lhs.size() < rhs.size();
}

std::string colorName(LightColor color)
{
    switch (color)
    {
    case LightColor::Red:
        return "red";
    case LightColor::Blue:
        return "blue";
    default:
        return "unknown";
    }
}

LightColor parseColor(const std::string& value)
{
    if (value == "red")
    {
        return LightColor::Red;
    }
    if (value == "blue")
    {
        return LightColor::Blue;
    }
    throw std::runtime_error("invalid target_color '" + value + "' (expected red or blue)");
}

std::vector<Annotation> loadAnnotations(const std::string& path)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        return {};
    }

    std::string line;
    if (!std::getline(input, line))
    {
        return {};
    }
    constexpr const char* kHeader = "filename,target_color,x,y,width,height";
    if (!line.empty() && line.back() == '\r')
    {
        line.pop_back();
    }
    if (line != kHeader)
    {
        throw std::runtime_error(
            "annotation CSV has an unexpected header: " + path +
            "; expected target box: " + kHeader);
    }

    std::vector<Annotation> result;
    std::size_t lineNumber = 1;
    while (std::getline(input, line))
    {
        ++lineNumber;
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        if (line.empty())
        {
            continue;
        }
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() != 6 || fields[0].empty())
        {
            throw std::runtime_error(
                "annotation line " + std::to_string(lineNumber) +
                ": expected 6 fields in target-box format");
        }
        Annotation annotation;
        annotation.filename = fields[0];
        try
        {
            annotation.targetColor = parseColor(fields[1]);
        }
        catch (const std::exception& ex)
        {
            throw std::runtime_error(
                "annotation line " + std::to_string(lineNumber) + ": " + ex.what());
        }
        annotation.targetBox = cv::Rect2f(
            static_cast<float>(parseDouble(fields[2], "x", lineNumber)),
            static_cast<float>(parseDouble(fields[3], "y", lineNumber)),
            static_cast<float>(parseDouble(fields[4], "width", lineNumber)),
            static_cast<float>(parseDouble(fields[5], "height", lineNumber)));
        try
        {
            validateAnnotation(annotation);
        }
        catch (const std::exception& ex)
        {
            throw std::runtime_error(
                "annotation line " + std::to_string(lineNumber) + ": " + ex.what());
        }
        if (std::any_of(result.begin(), result.end(), [&](const Annotation& existing) {
                return existing.filename == annotation.filename;
            }))
        {
            throw std::runtime_error(
                "annotation line " + std::to_string(lineNumber) + ": duplicate filename " +
                annotation.filename);
        }
        result.push_back(annotation);
    }
    std::sort(result.begin(), result.end(), [](const Annotation& lhs, const Annotation& rhs) {
        return naturalLess(lhs.filename, rhs.filename);
    });
    return result;
}

void saveAnnotations(const std::string& path, const std::vector<Annotation>& annotations)
{
    const std::filesystem::path output(path);
    if (!output.parent_path().empty())
    {
        std::filesystem::create_directories(output.parent_path());
    }
    const std::filesystem::path temporary = output.string() + ".tmp";
    {
        std::ofstream stream(temporary);
        if (!stream.is_open())
        {
            throw std::runtime_error("cannot write annotation CSV: " + temporary.string());
        }
        stream << "filename,target_color,x,y,width,height\n";
        stream << std::fixed << std::setprecision(3);
        std::vector<Annotation> sorted = annotations;
        std::sort(sorted.begin(), sorted.end(), [](const Annotation& lhs, const Annotation& rhs) {
            return naturalLess(lhs.filename, rhs.filename);
        });
        for (const Annotation& annotation : sorted)
        {
            validateAnnotation(annotation);
            stream << annotation.filename << ',' << colorName(annotation.targetColor) << ','
                   << annotation.targetBox.x << ',' << annotation.targetBox.y << ','
                   << annotation.targetBox.width << ',' << annotation.targetBox.height << '\n';
        }
        if (!stream.good())
        {
            throw std::runtime_error("failed while writing annotation CSV: " + temporary.string());
        }
    }
    std::filesystem::rename(temporary, output);
}

void validateAnnotation(const Annotation& annotation, const cv::Size& imageSize)
{
    if (annotation.filename.empty())
    {
        throw std::runtime_error("filename is empty");
    }
    if (annotation.targetColor != LightColor::Red && annotation.targetColor != LightColor::Blue)
    {
        throw std::runtime_error("target color must be red or blue");
    }
    const cv::Rect2f& box = annotation.targetBox;
    if (!std::isfinite(box.x) || !std::isfinite(box.y) ||
        !std::isfinite(box.width) || !std::isfinite(box.height))
    {
        throw std::runtime_error("target box coordinate is not finite");
    }
    if (box.x < 0.0F || box.y < 0.0F || box.width <= 0.0F || box.height <= 0.0F)
    {
        throw std::runtime_error("target box must have a non-negative origin and positive size");
    }
    if (imageSize.width > 0 && imageSize.height > 0 &&
        (box.x + box.width > static_cast<float>(imageSize.width) ||
         box.y + box.height > static_cast<float>(imageSize.height)))
    {
        throw std::runtime_error("target box lies outside the image");
    }
}

ImageOutcome evaluateImage(
    const Annotation& annotation,
    const std::vector<Armor>& armors,
    std::size_t candidateCount,
    std::size_t lightBarCount)
{
    validateAnnotation(annotation);
    ImageOutcome outcome;
    outcome.candidateCount = candidateCount;
    outcome.lightBarCount = lightBarCount;
    outcome.armorCount = armors.size();
    const cv::Point2f boxCenter(
        annotation.targetBox.x + annotation.targetBox.width * 0.5F,
        annotation.targetBox.y + annotation.targetBox.height * 0.5F);
    double bestSquaredDistance = std::numeric_limits<double>::infinity();

    for (std::size_t index = 0; index < armors.size(); ++index)
    {
        const cv::Point2f center = armors[index].center;
        const float right = annotation.targetBox.x + annotation.targetBox.width;
        const float bottom = annotation.targetBox.y + annotation.targetBox.height;
        if (center.x < annotation.targetBox.x || center.x > right ||
            center.y < annotation.targetBox.y || center.y > bottom)
        {
            continue;
        }
        const double dx = static_cast<double>(center.x - boxCenter.x);
        const double dy = static_cast<double>(center.y - boxCenter.y);
        const double squaredDistance = dx * dx + dy * dy;
        if (squaredDistance < bestSquaredDistance)
        {
            bestSquaredDistance = squaredDistance;
            outcome.matchedIndex = static_cast<int>(index);
        }
    }

    if (outcome.matchedIndex >= 0)
    {
        outcome.truePositives = 1;
        outcome.falsePositives = static_cast<int>(armors.size()) - 1;
    }
    else
    {
        outcome.falsePositives = static_cast<int>(armors.size());
        outcome.falseNegatives = 1;
    }
    return outcome;
}

void addOutcome(Metrics& metrics, const ImageOutcome& outcome)
{
    metrics.truePositives += outcome.truePositives;
    metrics.falsePositives += outcome.falsePositives;
    metrics.falseNegatives += outcome.falseNegatives;
    ++metrics.imageCount;
    metrics.totalCandidateCount += outcome.candidateCount;
    metrics.totalLightBarCount += outcome.lightBarCount;
    metrics.totalArmorCount += outcome.armorCount;
}

void finalizeMetrics(Metrics& metrics)
{
    const int predicted = metrics.truePositives + metrics.falsePositives;
    const int actual = metrics.truePositives + metrics.falseNegatives;
    metrics.precision = predicted > 0
        ? static_cast<double>(metrics.truePositives) / predicted
        : 0.0;
    metrics.recall = actual > 0
        ? static_cast<double>(metrics.truePositives) / actual
        : 0.0;
    metrics.f1 = metrics.precision + metrics.recall > 0.0
        ? 2.0 * metrics.precision * metrics.recall / (metrics.precision + metrics.recall)
        : 0.0;
    if (metrics.imageCount > 0)
    {
        const double count = static_cast<double>(metrics.imageCount);
        metrics.meanCandidateCount = metrics.totalCandidateCount / count;
        metrics.meanLightBarCount = metrics.totalLightBarCount / count;
        metrics.meanArmorCount = metrics.totalArmorCount / count;
    }
}

bool betterMetrics(const Metrics& lhs, const Metrics& rhs)
{
    if (std::abs(lhs.f1 - rhs.f1) > kComparisonEpsilon)
    {
        return lhs.f1 > rhs.f1;
    }
    if (std::abs(lhs.recall - rhs.recall) > kComparisonEpsilon)
    {
        return lhs.recall > rhs.recall;
    }
    if (std::abs(lhs.precision - rhs.precision) > kComparisonEpsilon)
    {
        return lhs.precision > rhs.precision;
    }
    if (std::abs(lhs.meanLightBarCount - rhs.meanLightBarCount) > kComparisonEpsilon)
    {
        return lhs.meanLightBarCount < rhs.meanLightBarCount;
    }
    if (std::abs(lhs.meanCandidateCount - rhs.meanCandidateCount) > kComparisonEpsilon)
    {
        return lhs.meanCandidateCount < rhs.meanCandidateCount;
    }
    return lhs.meanArmorCount < rhs.meanArmorCount;
}

ParameterSet defaultParameters()
{
    return {};
}

std::string parameterKey(const ParameterSet& parameters)
{
    std::ostringstream stream;
    stream << parameters.binaryThreshold << '|'
           << std::setprecision(17)
           << parameters.filter.minArea << '|'
           << parameters.filter.maxArea << '|'
           << parameters.filter.minAspectRatio << '|'
           << parameters.filter.maxAspectRatio << '|'
           << parameters.filter.maxLineAngleDeg << '|'
           << parameters.filter.minFillRatio << '|'
           << parameters.matcher.maxLightLengthRatio << '|'
           << parameters.matcher.maxLightAngleDiffDeg << '|'
           << parameters.matcher.maxLightCenterYDiff << '|'
           << parameters.matcher.minCenterDistanceRatio << '|'
           << parameters.matcher.maxCenterDistanceRatio;
    return stream.str();
}

double distanceFromDefaults(const ParameterSet& parameters)
{
    const ParameterSet defaults;
    return normalizedDistance(parameters.binaryThreshold, defaults.binaryThreshold, 20.0) +
        normalizedDistance(parameters.filter.minArea, defaults.filter.minArea, 5.0) +
        normalizedDistance(parameters.filter.maxArea, defaults.filter.maxArea, 3000.0) +
        normalizedDistance(parameters.filter.minAspectRatio, defaults.filter.minAspectRatio, 0.5) +
        normalizedDistance(parameters.filter.maxAspectRatio, defaults.filter.maxAspectRatio, 5.0) +
        normalizedDistance(parameters.filter.maxLineAngleDeg, defaults.filter.maxLineAngleDeg, 5.0) +
        normalizedDistance(parameters.filter.minFillRatio, defaults.filter.minFillRatio, 0.2) +
        normalizedDistance(parameters.matcher.maxLightLengthRatio, defaults.matcher.maxLightLengthRatio, 0.2) +
        normalizedDistance(parameters.matcher.maxLightAngleDiffDeg, defaults.matcher.maxLightAngleDiffDeg, 5.0) +
        normalizedDistance(parameters.matcher.maxLightCenterYDiff, defaults.matcher.maxLightCenterYDiff, 15.0) +
        normalizedDistance(parameters.matcher.minCenterDistanceRatio, defaults.matcher.minCenterDistanceRatio, 0.5) +
        normalizedDistance(parameters.matcher.maxCenterDistanceRatio, defaults.matcher.maxCenterDistanceRatio, 1.0);
}

std::vector<ParameterSet> sampleParameterSets(
    std::size_t randomCount,
    std::uint32_t seed,
    const SearchRanges& ranges)
{
    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> threshold(
        ranges.binaryThresholdMin,
        ranges.binaryThresholdMax);

    std::vector<ParameterSet> result;
    result.reserve(randomCount + 1);
    result.push_back(defaultParameters());
    for (std::size_t index = 0; index < randomCount; ++index)
    {
        ParameterSet parameters;
        parameters.binaryThreshold = threshold(generator);
        parameters.filter.minArea = uniform(generator, ranges.minAreaMin, ranges.minAreaMax);
        parameters.filter.maxArea = uniform(generator, ranges.maxAreaMin, ranges.maxAreaMax);
        parameters.filter.minAspectRatio = uniform(
            generator, ranges.minAspectRatioMin, ranges.minAspectRatioMax);
        parameters.filter.maxAspectRatio = uniform(
            generator, ranges.maxAspectRatioMin, ranges.maxAspectRatioMax);
        parameters.filter.maxLineAngleDeg = uniform(
            generator, ranges.maxLineAngleDegMin, ranges.maxLineAngleDegMax);
        parameters.filter.minFillRatio = uniform(
            generator, ranges.minFillRatioMin, ranges.minFillRatioMax);
        parameters.matcher.maxLightLengthRatio = uniform(
            generator, ranges.maxLightLengthRatioMin, ranges.maxLightLengthRatioMax);
        parameters.matcher.maxLightAngleDiffDeg = uniform(
            generator, ranges.maxLightAngleDiffDegMin, ranges.maxLightAngleDiffDegMax);
        parameters.matcher.maxLightCenterYDiff = uniform(
            generator, ranges.maxLightCenterYDiffMin, ranges.maxLightCenterYDiffMax);
        parameters.matcher.minCenterDistanceRatio = uniform(
            generator, ranges.minCenterDistanceRatioMin, ranges.minCenterDistanceRatioMax);
        parameters.matcher.maxCenterDistanceRatio = uniform(
            generator, ranges.maxCenterDistanceRatioMin, ranges.maxCenterDistanceRatioMax);
        result.push_back(parameters);
    }
    return result;
}

std::vector<ParameterSet> locallyRefine(
    const std::vector<ParameterSet>& seeds,
    const Evaluator& evaluator,
    const SearchRanges& ranges,
    const std::vector<double>& relativeSteps)
{
    std::vector<ParameterSet> refined;
    refined.reserve(seeds.size());
    for (const ParameterSet& seed : seeds)
    {
        ParameterSet current = seed;
        Metrics currentMetrics = evaluator(current);
        for (double step : relativeSteps)
        {
            for (std::size_t parameter = 0; parameter < kParameterCount; ++parameter)
            {
                ParameterSet bestParameters = current;
                Metrics bestMetrics = currentMetrics;
                for (double direction : {-1.0, 1.0})
                {
                    const ParameterSet candidate = adjustedParameter(
                        current, parameter, direction * step, ranges);
                    const Metrics candidateMetrics = evaluator(candidate);
                    if (betterMetrics(candidateMetrics, bestMetrics) ||
                        (!betterMetrics(bestMetrics, candidateMetrics) &&
                         distanceFromDefaults(candidate) < distanceFromDefaults(bestParameters)))
                    {
                        bestParameters = candidate;
                        bestMetrics = candidateMetrics;
                    }
                }
                current = bestParameters;
                currentMetrics = bestMetrics;
            }
        }
        refined.push_back(current);
    }
    return refined;
}

StabilityMetrics evaluateStability(
    const ParameterSet& parameters,
    const Evaluator& evaluator,
    const SearchRanges& ranges,
    double relativeStep)
{
    const Metrics baseline = evaluator(parameters);
    StabilityMetrics stability{baseline.f1};
    for (std::size_t parameter = 0; parameter < kParameterCount; ++parameter)
    {
        for (double direction : {-1.0, 1.0})
        {
            const ParameterSet candidate = adjustedParameter(
                parameters, parameter, direction * relativeStep, ranges);
            const Metrics metrics = evaluator(candidate);
            stability.worstF1 = std::min(stability.worstF1, metrics.f1);
        }
    }
    return stability;
}

} // namespace auto_aim::detector::tuning

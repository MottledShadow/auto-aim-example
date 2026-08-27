#include "tuning_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "debug_draw.hpp"

namespace fs = std::filesystem;

namespace
{

using auto_aim::detector::Armor;
using auto_aim::detector::LightBar;
using auto_aim::detector::LightColor;
using auto_aim::detector::LightbarDetector;
using auto_aim::detector::PreprocessResult;
using auto_aim::detector::tuning::Annotation;
using auto_aim::detector::tuning::ImageOutcome;
using auto_aim::detector::tuning::Metrics;
using auto_aim::detector::tuning::ParameterSet;
using auto_aim::detector::tuning::SearchRanges;
using auto_aim::detector::tuning::StabilityMetrics;

constexpr std::size_t kRandomCandidateCount = 2000;
constexpr std::size_t kRefinementSeedCount = 20;
constexpr std::size_t kReportCandidateCount = 100;
constexpr std::uint32_t kRandomSeed = 20260827U;
constexpr double kNearBestF1Tolerance = 0.01;

struct DatasetImage
{
    fs::path path;
    cv::Mat image;
    Annotation annotation;
};

struct DetectionStages
{
    PreprocessResult preprocess;
    std::vector<LightBar> lightBars;
    std::vector<Armor> armors;
    ImageOutcome outcome;
};

struct RankedResult
{
    ParameterSet parameters;
    Metrics metrics;
    StabilityMetrics stability;
};

struct ImageReport
{
    std::string filename;
    ImageOutcome outcome;
};

std::vector<fs::path> snapshotPaths(const fs::path& captureDirectory)
{
    if (!fs::is_directory(captureDirectory))
    {
        throw std::runtime_error("capture directory does not exist: " + captureDirectory.string());
    }
    std::vector<fs::path> result;
    for (const fs::directory_entry& entry : fs::directory_iterator(captureDirectory))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }
        const std::string filename = entry.path().filename().string();
        if (filename.rfind("snapshot_", 0) == 0 && entry.path().extension() == ".png")
        {
            result.push_back(entry.path());
        }
    }
    std::sort(result.begin(), result.end(), [](const fs::path& lhs, const fs::path& rhs) {
        return auto_aim::detector::tuning::naturalLess(
            lhs.filename().string(), rhs.filename().string());
    });
    if (result.empty())
    {
        throw std::runtime_error("no snapshot_*.png files found in " + captureDirectory.string());
    }
    return result;
}

cv::Mat loadBgrImage(const fs::path& path)
{
    cv::Mat image = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (image.empty())
    {
        throw std::runtime_error("cannot read image: " + path.string());
    }
    if (image.type() == CV_8UC4)
    {
        cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
    }
    if (image.type() != CV_8UC3)
    {
        throw std::runtime_error("image must be 8-bit BGR/RGBA: " + path.string());
    }
    return image;
}

LightColor promptColor(LightColor current)
{
    while (true)
    {
        std::cout << "target color [r/b";
        if (current == LightColor::Red || current == LightColor::Blue)
        {
            std::cout << ", Enter keeps "
                      << auto_aim::detector::tuning::colorName(current);
        }
        std::cout << "]: " << std::flush;

        std::string input;
        if (!std::getline(std::cin, input))
        {
            throw std::runtime_error("stdin closed while reading target color");
        }
        if (input.empty() && (current == LightColor::Red || current == LightColor::Blue))
        {
            return current;
        }
        if (input == "r" || input == "red")
        {
            return LightColor::Red;
        }
        if (input == "b" || input == "blue")
        {
            return LightColor::Blue;
        }
        std::cout << "enter r for red or b for blue\n";
    }
}

int annotateDataset(
    const fs::path& captureDirectory,
    const fs::path& annotationPath,
    bool force)
{
    const std::vector<fs::path> paths = snapshotPaths(captureDirectory);
    std::vector<Annotation> annotations =
        auto_aim::detector::tuning::loadAnnotations(annotationPath.string());

    std::map<std::string, fs::path> pathByFilename;
    std::vector<std::string> filenames;
    filenames.reserve(paths.size());
    for (const fs::path& path : paths)
    {
        const std::string filename = path.filename().string();
        pathByFilename.emplace(filename, path);
        filenames.push_back(filename);
    }
    for (const Annotation& annotation : annotations)
    {
        const auto found = pathByFilename.find(annotation.filename);
        if (found == pathByFilename.end())
        {
            throw std::runtime_error(
                "annotation references missing capture: " + annotation.filename);
        }
        const cv::Mat image = loadBgrImage(found->second);
        auto_aim::detector::tuning::validateAnnotation(annotation, image.size());
    }

    const std::vector<std::string> worklist =
        auto_aim::detector::tuning::annotationWorklist(filenames, annotations, force);
    if (worklist.empty())
    {
        std::cout << "all " << paths.size() << " snapshots are already annotated\n";
        return 0;
    }

    std::cout << "annotating " << worklist.size() << " of " << paths.size()
              << " snapshots; Enter/Space accepts ROI, c cancels ROI\n";
    for (std::size_t index = 0; index < worklist.size(); ++index)
    {
        const std::string& filename = worklist[index];
        const cv::Mat image = loadBgrImage(pathByFilename.at(filename));
        LightColor currentColor = LightColor::Unknown;
        const auto existing = std::find_if(
            annotations.begin(), annotations.end(), [&](const Annotation& annotation) {
                return annotation.filename == filename;
            });
        if (existing != annotations.end())
        {
            currentColor = existing->targetColor;
        }

        cv::Rect selected;
        while (selected.width <= 0 || selected.height <= 0)
        {
            const std::string windowName =
                "annotate " + std::to_string(index + 1) + "/" +
                std::to_string(worklist.size()) + " " + filename;
            selected = cv::selectROI(windowName, image, true, false, true);
            cv::destroyWindow(windowName);
            if (selected.width > 0 && selected.height > 0)
            {
                break;
            }

            std::cout << "selection canceled for " << filename
                      << "; [r]etry or [q]uit: " << std::flush;
            std::string action;
            if (!std::getline(std::cin, action) || action == "q" || action == "quit")
            {
                std::cout << "annotation stopped; completed rows remain saved\n";
                return 0;
            }
        }

        Annotation annotation;
        annotation.filename = filename;
        annotation.targetColor = promptColor(currentColor);
        annotation.targetBox = cv::Rect2f(selected);
        auto_aim::detector::tuning::validateAnnotation(annotation, image.size());
        auto_aim::detector::tuning::upsertAnnotation(annotations, annotation);
        auto_aim::detector::tuning::saveAnnotations(annotationPath.string(), annotations);
        std::cout << "saved " << filename << " "
                  << auto_aim::detector::tuning::colorName(annotation.targetColor)
                  << " box=" << selected.x << ',' << selected.y << ','
                  << selected.width << ',' << selected.height << '\n';
    }

    cv::destroyAllWindows();
    std::cout << "annotation complete: " << annotations.size() << " rows saved to "
              << annotationPath << '\n';
    return 0;
}

std::vector<DatasetImage> loadDataset(
    const fs::path& captureDirectory,
    const fs::path& annotationPath)
{
    const std::vector<fs::path> paths = snapshotPaths(captureDirectory);
    const std::vector<Annotation> annotations =
        auto_aim::detector::tuning::loadAnnotations(annotationPath.string());
    std::map<std::string, Annotation> byFilename;
    for (const Annotation& annotation : annotations)
    {
        byFilename.emplace(annotation.filename, annotation);
    }

    std::vector<DatasetImage> result;
    result.reserve(paths.size());
    for (const fs::path& path : paths)
    {
        const std::string filename = path.filename().string();
        const auto found = byFilename.find(filename);
        if (found == byFilename.end())
        {
            throw std::runtime_error(
                "missing annotation for " + filename +
                "; complete annotations.csv and run tuning-run.sh validate");
        }
        cv::Mat image = loadBgrImage(path);
        auto_aim::detector::tuning::validateAnnotation(found->second, image.size());
        result.push_back({path, std::move(image), found->second});
        byFilename.erase(found);
    }
    if (!byFilename.empty())
    {
        throw std::runtime_error(
            "annotation references missing capture: " + byFilename.begin()->first);
    }
    return result;
}

int validateDataset(const fs::path& captureDirectory, const fs::path& annotationPath)
{
    const std::vector<DatasetImage> images = loadDataset(captureDirectory, annotationPath);
    std::cout << "annotation validation PASS: " << images.size() << " snapshots\n";
    for (const DatasetImage& image : images)
    {
        const Annotation& annotation = image.annotation;
        const cv::Rect2f& box = annotation.targetBox;
        std::cout << "  " << annotation.filename << " "
                  << auto_aim::detector::tuning::colorName(annotation.targetColor)
                  << " box=" << std::fixed << std::setprecision(2)
                  << box.x << ',' << box.y << ',' << box.width << ',' << box.height << '\n';
    }
    return 0;
}

class DatasetEvaluator
{
public:
    explicit DatasetEvaluator(std::vector<DatasetImage> images)
        : images_(std::move(images))
    {
    }

    const std::vector<DatasetImage>& images() const { return images_; }

    Metrics evaluate(const ParameterSet& parameters)
    {
        Metrics metrics;
        const std::vector<PreprocessResult>& preprocess = cachedPreprocess(parameters.binaryThreshold);
        for (std::size_t index = 0; index < images_.size(); ++index)
        {
            LightbarDetector detector;
            detector.filterParams = parameters.filter;
            detector.filterParams.targetColor = images_[index].annotation.targetColor;
            detector.matcherParams = parameters.matcher;
            const std::vector<LightBar> bars = detector.filterLightBars(
                images_[index].image, preprocess[index]);
            const std::vector<Armor> armors = detector.matchArmors(bars);
            auto_aim::detector::tuning::addOutcome(
                metrics,
                auto_aim::detector::tuning::evaluateImage(
                    images_[index].annotation,
                    armors,
                    preprocess[index].candidates.size(),
                    bars.size()));
        }
        auto_aim::detector::tuning::finalizeMetrics(metrics);
        return metrics;
    }

    DetectionStages detect(std::size_t imageIndex, const ParameterSet& parameters) const
    {
        LightbarDetector detector;
        detector.binaryThreshold = parameters.binaryThreshold;
        detector.filterParams = parameters.filter;
        detector.filterParams.targetColor = images_.at(imageIndex).annotation.targetColor;
        detector.matcherParams = parameters.matcher;

        DetectionStages stages;
        stages.preprocess = detector.preprocess(images_[imageIndex].image);
        stages.lightBars = detector.filterLightBars(images_[imageIndex].image, stages.preprocess);
        stages.armors = detector.matchArmors(stages.lightBars);
        stages.outcome = auto_aim::detector::tuning::evaluateImage(
            images_[imageIndex].annotation,
            stages.armors,
            stages.preprocess.candidates.size(),
            stages.lightBars.size());
        return stages;
    }

private:
    const std::vector<PreprocessResult>& cachedPreprocess(int threshold)
    {
        const auto found = preprocessCache_.find(threshold);
        if (found != preprocessCache_.end())
        {
            return found->second;
        }

        LightbarDetector detector;
        detector.binaryThreshold = threshold;
        std::vector<PreprocessResult> entries;
        entries.reserve(images_.size());
        for (const DatasetImage& image : images_)
        {
            PreprocessResult preprocess = detector.preprocess(image.image);
            preprocess.binary.release();
            entries.push_back(std::move(preprocess));
        }
        return preprocessCache_.emplace(threshold, std::move(entries)).first->second;
    }

    std::vector<DatasetImage> images_;
    std::unordered_map<int, std::vector<PreprocessResult>> preprocessCache_;
};

bool betterResult(const RankedResult& lhs, const RankedResult& rhs)
{
    if (auto_aim::detector::tuning::betterMetrics(lhs.metrics, rhs.metrics))
    {
        return true;
    }
    if (auto_aim::detector::tuning::betterMetrics(rhs.metrics, lhs.metrics))
    {
        return false;
    }
    const double leftDistance = auto_aim::detector::tuning::distanceFromDefaults(lhs.parameters);
    const double rightDistance = auto_aim::detector::tuning::distanceFromDefaults(rhs.parameters);
    if (leftDistance != rightDistance)
    {
        return leftDistance < rightDistance;
    }
    return auto_aim::detector::tuning::parameterKey(lhs.parameters) <
        auto_aim::detector::tuning::parameterKey(rhs.parameters);
}

void writeParameterHeader(std::ostream& stream)
{
    stream << "binary_threshold,min_area,max_area,min_aspect_ratio,max_aspect_ratio,"
              "max_line_angle_deg,min_fill_ratio,max_light_length_ratio,"
              "max_light_angle_diff_deg,max_light_center_y_diff,"
              "min_center_distance_ratio,max_center_distance_ratio";
}

void writeParameterValues(std::ostream& stream, const ParameterSet& parameters)
{
    stream << parameters.binaryThreshold << ','
           << parameters.filter.minArea << ','
           << parameters.filter.maxArea << ','
           << parameters.filter.minAspectRatio << ','
           << parameters.filter.maxAspectRatio << ','
           << parameters.filter.maxLineAngleDeg << ','
           << parameters.filter.minFillRatio << ','
           << parameters.matcher.maxLightLengthRatio << ','
           << parameters.matcher.maxLightAngleDiffDeg << ','
           << parameters.matcher.maxLightCenterYDiff << ','
           << parameters.matcher.minCenterDistanceRatio << ','
           << parameters.matcher.maxCenterDistanceRatio;
}

void writeBestParameters(const fs::path& path, const RankedResult& result)
{
    const ParameterSet defaults;
    std::ofstream output(path);
    if (!output.is_open())
    {
        throw std::runtime_error("cannot write " + path.string());
    }
    output << std::fixed << std::setprecision(6)
           << "score: F1=" << result.metrics.f1
           << " precision=" << result.metrics.precision
           << " recall=" << result.metrics.recall
           << " worst_perturbed_f1=" << result.stability.worstF1
           << " mean_candidates=" << result.metrics.meanCandidateCount
           << " mean_light_bars=" << result.metrics.meanLightBarCount
           << " mean_armors=" << result.metrics.meanArmorCount << "\n\n"
           << "LightbarDetector detector;\n"
           << "detector.binaryThreshold = " << result.parameters.binaryThreshold << ";\n"
           << "detector.filterParams.minArea = " << result.parameters.filter.minArea << ";\n"
           << "detector.filterParams.maxArea = " << result.parameters.filter.maxArea << ";\n"
           << "detector.filterParams.minAspectRatio = " << result.parameters.filter.minAspectRatio << ";\n"
           << "detector.filterParams.maxAspectRatio = " << result.parameters.filter.maxAspectRatio << ";\n"
           << "detector.filterParams.maxLineAngleDeg = " << result.parameters.filter.maxLineAngleDeg << ";\n"
           << "detector.filterParams.minFillRatio = " << result.parameters.filter.minFillRatio << ";\n"
           << "detector.matcherParams.maxLightLengthRatio = " << result.parameters.matcher.maxLightLengthRatio << ";\n"
           << "detector.matcherParams.maxLightAngleDiffDeg = " << result.parameters.matcher.maxLightAngleDiffDeg << ";\n"
           << "detector.matcherParams.maxLightCenterYDiff = " << result.parameters.matcher.maxLightCenterYDiff << ";\n"
           << "detector.matcherParams.minCenterDistanceRatio = " << result.parameters.matcher.minCenterDistanceRatio << ";\n"
           << "detector.matcherParams.maxCenterDistanceRatio = " << result.parameters.matcher.maxCenterDistanceRatio << ";\n\n"
           << "Current defaults distance: "
           << auto_aim::detector::tuning::distanceFromDefaults(result.parameters) << "\n"
           << "Default binaryThreshold: " << defaults.binaryThreshold << '\n';
}

cv::Mat resizedPanel(const cv::Mat& image, const std::string& title)
{
    cv::Mat panel;
    cv::resize(image, panel, cv::Size(720, 540), 0.0, 0.0, cv::INTER_AREA);
    cv::rectangle(panel, cv::Rect(0, 0, panel.cols, 34), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
        panel, title, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
        0.65, cv::Scalar(0, 255, 0), 2);
    return panel;
}

void drawArmorPolygon(cv::Mat& image, const Armor& armor, const cv::Scalar& color, int thickness)
{
    const std::vector<cv::Point> points = {
        armor.leftLight.top,
        armor.rightLight.top,
        armor.rightLight.bottom,
        armor.leftLight.bottom,
    };
    cv::polylines(image, points, true, color, thickness, cv::LINE_AA);
}

void drawTargetBox(cv::Mat& image, const Annotation& annotation)
{
    cv::rectangle(image, annotation.targetBox, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
}

std::vector<ImageReport> writeDiagnostics(
    const DatasetEvaluator& evaluator,
    const ParameterSet& parameters,
    const fs::path& outputDirectory,
    const std::string& label)
{
    fs::create_directories(outputDirectory);
    std::vector<ImageReport> reports;
    reports.reserve(evaluator.images().size());

    for (std::size_t index = 0; index < evaluator.images().size(); ++index)
    {
        const DatasetImage& item = evaluator.images()[index];
        const DetectionStages stages = evaluator.detect(index, parameters);

        cv::Mat original = item.image.clone();
        drawTargetBox(original, item.annotation);

        cv::Mat binary;
        cv::cvtColor(stages.preprocess.binary, binary, cv::COLOR_GRAY2BGR);

        cv::Mat filter = item.image.clone();
        auto_aim::debug_draw::drawLightBarMetrics(
            filter, stages.preprocess.candidates, parameters.filter);
        for (const LightBar& light : stages.lightBars)
        {
            cv::line(filter, light.top, light.bottom, cv::Scalar(255, 255, 0), 3, cv::LINE_AA);
        }

        cv::Mat matcher = item.image.clone();
        auto_aim::debug_draw::drawArmorMetrics(
            matcher, stages.lightBars, parameters.matcher);
        drawTargetBox(matcher, item.annotation);
        for (std::size_t armorIndex = 0; armorIndex < stages.armors.size(); ++armorIndex)
        {
            const bool matched = static_cast<int>(armorIndex) == stages.outcome.matchedIndex;
            drawArmorPolygon(
                matcher,
                stages.armors[armorIndex],
                matched ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                matched ? 4 : 2);
        }
        const std::string outcomeText =
            "TP=" + std::to_string(stages.outcome.truePositives) +
            " FP=" + std::to_string(stages.outcome.falsePositives) +
            " FN=" + std::to_string(stages.outcome.falseNegatives) +
            " C/B/A=" + std::to_string(stages.outcome.candidateCount) + "/" +
            std::to_string(stages.outcome.lightBarCount) + "/" +
            std::to_string(stages.outcome.armorCount);

        cv::Mat top;
        cv::hconcat(
            resizedPanel(original, label + " original + target box"),
            resizedPanel(binary, "binary threshold=" + std::to_string(parameters.binaryThreshold)),
            top);
        cv::Mat bottom;
        cv::hconcat(
            resizedPanel(filter, "filter candidates + accepted bars"),
            resizedPanel(matcher, "matching " + outcomeText),
            bottom);
        cv::Mat mosaic;
        cv::vconcat(top, bottom, mosaic);

        const fs::path outputPath = outputDirectory / item.path.filename();
        if (!cv::imwrite(outputPath.string(), mosaic))
        {
            throw std::runtime_error("cannot write diagnostic image: " + outputPath.string());
        }
        reports.push_back({item.path.filename().string(), stages.outcome});
    }
    return reports;
}

int scan(
    const fs::path& captureDirectory,
    const fs::path& annotationPath,
    const fs::path& outputDirectory)
{
    const auto started = std::chrono::steady_clock::now();
    fs::create_directories(outputDirectory);
    DatasetEvaluator evaluator(loadDataset(captureDirectory, annotationPath));
    std::cout << "loaded " << evaluator.images().size() << " annotated snapshots\n";

    std::unordered_map<std::string, Metrics> scoreCache;
    const auto evaluateCached = [&](const ParameterSet& parameters) {
        const std::string key = auto_aim::detector::tuning::parameterKey(parameters);
        const auto found = scoreCache.find(key);
        if (found != scoreCache.end())
        {
            return found->second;
        }
        const Metrics metrics = evaluator.evaluate(parameters);
        scoreCache.emplace(key, metrics);
        return metrics;
    };

    const SearchRanges ranges;
    const std::vector<ParameterSet> sampled =
        auto_aim::detector::tuning::sampleParameterSets(
            kRandomCandidateCount, kRandomSeed, ranges);
    std::vector<RankedResult> ranked;
    ranked.reserve(sampled.size() + kRefinementSeedCount);
    for (std::size_t index = 0; index < sampled.size(); ++index)
    {
        ranked.push_back({sampled[index], evaluateCached(sampled[index]), {}});
        if ((index + 1) % 200 == 0)
        {
            std::cout << "random search " << (index + 1) << '/' << sampled.size() << '\n';
        }
    }
    std::sort(ranked.begin(), ranked.end(), betterResult);

    std::vector<ParameterSet> seeds;
    for (std::size_t index = 0; index < std::min(kRefinementSeedCount, ranked.size()); ++index)
    {
        seeds.push_back(ranked[index].parameters);
    }
    const std::vector<ParameterSet> refined = auto_aim::detector::tuning::locallyRefine(
        seeds, evaluateCached, ranges, {0.10, 0.05});
    for (const ParameterSet& parameters : refined)
    {
        const std::string key = auto_aim::detector::tuning::parameterKey(parameters);
        if (std::none_of(ranked.begin(), ranked.end(), [&](const RankedResult& result) {
                return auto_aim::detector::tuning::parameterKey(result.parameters) == key;
            }))
        {
            ranked.push_back({parameters, evaluateCached(parameters), {}});
        }
    }
    std::sort(ranked.begin(), ranked.end(), betterResult);
    if (ranked.size() > kReportCandidateCount)
    {
        ranked.resize(kReportCandidateCount);
    }

    std::cout << "evaluating stability for top " << ranked.size() << " candidates\n";
    for (RankedResult& result : ranked)
    {
        result.stability = auto_aim::detector::tuning::evaluateStability(
            result.parameters, evaluateCached, ranges, 0.02);
    }

    const double bestF1 = ranked.front().metrics.f1;
    std::size_t recommendedIndex = 0;
    for (std::size_t index = 1; index < ranked.size(); ++index)
    {
        if (ranked[index].metrics.f1 + kNearBestF1Tolerance < bestF1)
        {
            continue;
        }
        const RankedResult& candidate = ranked[index];
        const RankedResult& current = ranked[recommendedIndex];
        if (candidate.stability.worstF1 > current.stability.worstF1 + 1e-12 ||
            (std::abs(candidate.stability.worstF1 - current.stability.worstF1) <= 1e-12 &&
             betterResult(candidate, current)))
        {
            recommendedIndex = index;
        }
    }
    const RankedResult recommended = ranked[recommendedIndex];

    {
        std::ofstream output(outputDirectory / "ranking.csv");
        output << "rank,recommended,tp,fp,fn,precision,recall,f1,"
                  "mean_candidates,mean_light_bars,mean_armors,"
                  "worst_perturbed_f1,default_distance,";
        writeParameterHeader(output);
        output << '\n' << std::fixed << std::setprecision(8);
        for (std::size_t index = 0; index < ranked.size(); ++index)
        {
            const RankedResult& result = ranked[index];
            output << (index + 1) << ',' << (index == recommendedIndex ? "yes" : "no") << ','
                   << result.metrics.truePositives << ',' << result.metrics.falsePositives << ','
                   << result.metrics.falseNegatives << ',' << result.metrics.precision << ','
                   << result.metrics.recall << ',' << result.metrics.f1 << ','
                   << result.metrics.meanCandidateCount << ','
                   << result.metrics.meanLightBarCount << ','
                   << result.metrics.meanArmorCount << ','
                   << result.stability.worstF1 << ','
                   << auto_aim::detector::tuning::distanceFromDefaults(result.parameters) << ',';
            writeParameterValues(output, result.parameters);
            output << '\n';
        }
    }
    writeBestParameters(outputDirectory / "best_params.txt", recommended);

    const ParameterSet defaults;
    const Metrics defaultMetrics = evaluateCached(defaults);
    const std::vector<ImageReport> baselineReports = writeDiagnostics(
        evaluator, defaults, outputDirectory / "baseline", "baseline");
    const std::vector<ImageReport> bestReports = writeDiagnostics(
        evaluator, recommended.parameters, outputDirectory / "best", "recommended");

    {
        std::ofstream output(outputDirectory / "per_image.csv");
        output << "filename,target_color,x,y,width,height,";
        const auto writeOutcomeHeader = [&](const std::string& prefix) {
            output << prefix << "_tp," << prefix << "_fp," << prefix << "_fn,"
                   << prefix << "_matched_index," << prefix << "_candidates,"
                   << prefix << "_light_bars," << prefix << "_armors";
        };
        writeOutcomeHeader("baseline");
        output << ',';
        writeOutcomeHeader("best");
        output << '\n';
        output << std::fixed << std::setprecision(6);
        const auto writeOutcome = [&](const ImageOutcome& outcome) {
            output << outcome.truePositives << ',' << outcome.falsePositives << ','
                   << outcome.falseNegatives << ',' << outcome.matchedIndex << ','
                   << outcome.candidateCount << ',' << outcome.lightBarCount << ','
                   << outcome.armorCount;
        };
        for (std::size_t index = 0; index < evaluator.images().size(); ++index)
        {
            const Annotation& annotation = evaluator.images()[index].annotation;
            const ImageOutcome& baseline = baselineReports[index].outcome;
            const ImageOutcome& best = bestReports[index].outcome;
            output << annotation.filename << ','
                   << auto_aim::detector::tuning::colorName(annotation.targetColor) << ','
                   << annotation.targetBox.x << ',' << annotation.targetBox.y << ','
                   << annotation.targetBox.width << ',' << annotation.targetBox.height << ',';
            writeOutcome(baseline);
            output << ',';
            writeOutcome(best);
            output << '\n';
        }
    }

    const double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    {
        std::ofstream output(outputDirectory / "summary.txt");
        output << std::fixed << std::setprecision(6)
               << "images=" << evaluator.images().size() << '\n'
               << "random_seed=" << kRandomSeed << '\n'
               << "random_candidates=" << kRandomCandidateCount << '\n'
               << "unique_evaluations=" << scoreCache.size() << '\n'
               << "elapsed_seconds=" << seconds << '\n'
               << "baseline_f1=" << defaultMetrics.f1 << '\n'
               << "baseline_precision=" << defaultMetrics.precision << '\n'
               << "baseline_recall=" << defaultMetrics.recall << '\n'
               << "recommended_f1=" << recommended.metrics.f1 << '\n'
               << "recommended_precision=" << recommended.metrics.precision << '\n'
               << "recommended_recall=" << recommended.metrics.recall << '\n'
               << "baseline_mean_candidates=" << defaultMetrics.meanCandidateCount << '\n'
               << "baseline_mean_light_bars=" << defaultMetrics.meanLightBarCount << '\n'
               << "baseline_mean_armors=" << defaultMetrics.meanArmorCount << '\n'
               << "recommended_mean_candidates=" << recommended.metrics.meanCandidateCount << '\n'
               << "recommended_mean_light_bars=" << recommended.metrics.meanLightBarCount << '\n'
               << "recommended_mean_armors=" << recommended.metrics.meanArmorCount << '\n'
               << "recommended_worst_perturbed_f1=" << recommended.stability.worstF1 << '\n';
    }

    std::cout << std::fixed << std::setprecision(4)
              << "scan complete: evaluations=" << scoreCache.size()
              << " baseline_f1=" << defaultMetrics.f1
              << " recommended_f1=" << recommended.metrics.f1
              << " stable_f1=" << recommended.stability.worstF1
              << " mean_candidates=" << recommended.metrics.meanCandidateCount
              << " mean_light_bars=" << recommended.metrics.meanLightBarCount
              << " elapsed_s=" << seconds << '\n'
              << "results: " << outputDirectory << '\n';
    return 0;
}

void usage(const char* executable)
{
    std::cerr
        << "Usage:\n"
        << "  " << executable << " annotate <capture_dir> <annotations.csv> [--force]\n"
        << "  " << executable << " validate <capture_dir> <annotations.csv>\n"
        << "  " << executable << " scan <capture_dir> <annotations.csv> <output_dir>\n";
}

} // namespace

int main(int argc, char** argv)
try
{
    if (argc < 2)
    {
        usage(argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    if (mode == "annotate" && (argc == 4 || argc == 5))
    {
        const bool force = argc == 5;
        if (force && std::string(argv[4]) != "--force")
        {
            usage(argv[0]);
            return 2;
        }
        return annotateDataset(argv[2], argv[3], force);
    }
    if (mode == "validate" && argc == 4)
    {
        return validateDataset(argv[2], argv[3]);
    }
    if (mode == "scan" && argc == 5)
    {
        return scan(argv[2], argv[3], argv[4]);
    }
    usage(argv[0]);
    return 2;
}
catch (const std::exception& ex)
{
    std::cerr << "detector_tuner: " << ex.what() << '\n';
    return 1;
}

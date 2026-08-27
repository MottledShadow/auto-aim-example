#include "tuning_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{
namespace fs = std::filesystem;

using auto_aim::detector::Armor;
using auto_aim::detector::LightColor;
using auto_aim::detector::tuning::Annotation;
using auto_aim::detector::tuning::ImageOutcome;
using auto_aim::detector::tuning::Metrics;
using auto_aim::detector::tuning::ParameterSet;

class TemporaryDirectory
{
public:
    TemporaryDirectory()
    {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = fs::temp_directory_path() /
            ("detector-tuning-core-test-" + std::to_string(suffix));
        fs::create_directories(path_);
    }

    ~TemporaryDirectory()
    {
        std::error_code ignored;
        fs::remove_all(path_, ignored);
    }

    const fs::path& path() const { return path_; }

private:
    fs::path path_;
};

Armor makeArmor(float centerX, float centerY)
{
    Armor armor;
    armor.leftLight.top = {centerX - 10.0F, centerY - 10.0F};
    armor.leftLight.bottom = {centerX - 10.0F, centerY + 10.0F};
    armor.leftLight.center = {centerX - 10.0F, centerY};
    armor.rightLight.top = {centerX + 10.0F, centerY - 10.0F};
    armor.rightLight.bottom = {centerX + 10.0F, centerY + 10.0F};
    armor.rightLight.center = {centerX + 10.0F, centerY};
    armor.center = {centerX, centerY};
    return armor;
}

Annotation makeAnnotation(
    const std::string& filename = "snapshot_0.png",
    LightColor color = LightColor::Red,
    float x = 10.0F,
    float y = 20.0F,
    float width = 100.0F,
    float height = 60.0F)
{
    Annotation annotation;
    annotation.filename = filename;
    annotation.targetColor = color;
    annotation.targetBox = {x, y, width, height};
    return annotation;
}

TEST(TuningNaturalSort, OrdersNumericSnapshotSuffixesNaturally)
{
    std::vector<std::string> names = {
        "snapshot_10.png", "snapshot_2.png", "snapshot_1.png"};
    std::sort(names.begin(), names.end(), auto_aim::detector::tuning::naturalLess);
    EXPECT_EQ(
        names,
        (std::vector<std::string>{"snapshot_1.png", "snapshot_2.png", "snapshot_10.png"}));
}

TEST(TuningAnnotations, RoundTripsAndSortsBoxCsv)
{
    TemporaryDirectory temporary;
    const fs::path path = temporary.path() / "annotations.csv";
    const std::vector<Annotation> input = {
        makeAnnotation("snapshot_10.png", LightColor::Blue, 10, 20, 40, 60),
        makeAnnotation("snapshot_2.png", LightColor::Red, 1, 2, 11, 12),
    };

    auto_aim::detector::tuning::saveAnnotations(path.string(), input);
    const std::vector<Annotation> output =
        auto_aim::detector::tuning::loadAnnotations(path.string());

    ASSERT_EQ(output.size(), 2U);
    EXPECT_EQ(output[0].filename, "snapshot_2.png");
    EXPECT_EQ(output[0].targetColor, LightColor::Red);
    EXPECT_EQ(output[0].targetBox, cv::Rect2f(1, 2, 11, 12));
    EXPECT_EQ(output[1].filename, "snapshot_10.png");
    EXPECT_EQ(output[1].targetColor, LightColor::Blue);
}

TEST(TuningAnnotations, RejectsDuplicateInvalidColorAndWrongHeader)
{
    TemporaryDirectory temporary;
    const fs::path path = temporary.path() / "annotations.csv";
    {
        std::ofstream output(path);
        output << "filename,target_color,x,y,width,height\n"
               << "snapshot_1.png,red,1,2,11,12\n"
               << "snapshot_1.png,blue,1,2,11,12\n";
    }
    EXPECT_THROW(auto_aim::detector::tuning::loadAnnotations(path.string()), std::runtime_error);

    {
        std::ofstream output(path);
        output << "filename,target_color,x,y,width,height\n"
               << "snapshot_1.png,green,1,2,11,12\n";
    }
    EXPECT_THROW(auto_aim::detector::tuning::loadAnnotations(path.string()), std::runtime_error);

    {
        std::ofstream output(path);
        output << "filename,target_color,left_top_x,left_top_y\n";
    }
    EXPECT_THROW(auto_aim::detector::tuning::loadAnnotations(path.string()), std::runtime_error);
}

TEST(TuningAnnotations, RejectsInvalidAndOutOfBoundsBoxes)
{
    Annotation invalid = makeAnnotation();
    invalid.targetBox.width = 0.0F;
    EXPECT_THROW(
        auto_aim::detector::tuning::validateAnnotation(invalid, cv::Size(200, 100)),
        std::runtime_error);

    Annotation outside = makeAnnotation();
    outside.targetBox.x = 150.0F;
    EXPECT_THROW(
        auto_aim::detector::tuning::validateAnnotation(outside, cv::Size(200, 100)),
        std::runtime_error);
}

TEST(TuningScoring, CountsMatchAndAllAdditionalDetectionsAsFalsePositives)
{
    const auto outcome = auto_aim::detector::tuning::evaluateImage(
        makeAnnotation(), {makeArmor(160.0F, 50.0F), makeArmor(60.0F, 50.0F)}, 12, 4);

    EXPECT_EQ(outcome.truePositives, 1);
    EXPECT_EQ(outcome.falsePositives, 1);
    EXPECT_EQ(outcome.falseNegatives, 0);
    EXPECT_EQ(outcome.matchedIndex, 1);
    EXPECT_EQ(outcome.candidateCount, 12U);
    EXPECT_EQ(outcome.lightBarCount, 4U);
    EXPECT_EQ(outcome.armorCount, 2U);
}

TEST(TuningScoring, SelectsInsideCandidateClosestToBoxCenter)
{
    const auto outcome = auto_aim::detector::tuning::evaluateImage(
        makeAnnotation(), {makeArmor(20.0F, 25.0F), makeArmor(62.0F, 49.0F)});

    EXPECT_EQ(outcome.matchedIndex, 1);
    EXPECT_EQ(outcome.truePositives, 1);
    EXPECT_EQ(outcome.falsePositives, 1);
}

TEST(TuningScoring, AcceptsCentersOnEveryBoxBoundary)
{
    const Annotation annotation = makeAnnotation();
    const std::vector<cv::Point2f> boundaries = {
        {10.0F, 50.0F}, {110.0F, 50.0F}, {60.0F, 20.0F}, {60.0F, 80.0F}};
    for (const cv::Point2f& center : boundaries)
    {
        const auto outcome = auto_aim::detector::tuning::evaluateImage(
            annotation, {makeArmor(center.x, center.y)});
        EXPECT_EQ(outcome.truePositives, 1);
        EXPECT_EQ(outcome.falseNegatives, 0);
    }
}

TEST(TuningScoring, OutsideCandidateIsFalsePositiveAndMiss)
{
    const auto outcome = auto_aim::detector::tuning::evaluateImage(
        makeAnnotation(), {makeArmor(150.0F, 50.0F)});

    EXPECT_EQ(outcome.truePositives, 0);
    EXPECT_EQ(outcome.falsePositives, 1);
    EXPECT_EQ(outcome.falseNegatives, 1);
    EXPECT_EQ(outcome.matchedIndex, -1);
}

TEST(TuningScoring, ComputesDetectionAndStageMeans)
{
    Metrics metrics;
    auto_aim::detector::tuning::addOutcome(metrics, ImageOutcome{1, 1, 0, 0, 10, 4, 2});
    auto_aim::detector::tuning::addOutcome(metrics, ImageOutcome{0, 1, 1, -1, 20, 2, 1});
    auto_aim::detector::tuning::finalizeMetrics(metrics);

    EXPECT_DOUBLE_EQ(metrics.precision, 1.0 / 3.0);
    EXPECT_DOUBLE_EQ(metrics.recall, 0.5);
    EXPECT_DOUBLE_EQ(metrics.f1, 0.4);
    EXPECT_DOUBLE_EQ(metrics.meanCandidateCount, 15.0);
    EXPECT_DOUBLE_EQ(metrics.meanLightBarCount, 3.0);
    EXPECT_DOUBLE_EQ(metrics.meanArmorCount, 1.5);
}

TEST(TuningScoring, EqualDetectionMetricsPreferLessStageNoise)
{
    Metrics quiet;
    quiet.f1 = quiet.recall = quiet.precision = 1.0;
    quiet.meanLightBarCount = 2.0;
    quiet.meanCandidateCount = 5.0;
    quiet.meanArmorCount = 1.0;
    Metrics noisy = quiet;
    noisy.meanLightBarCount = 3.0;

    EXPECT_TRUE(auto_aim::detector::tuning::betterMetrics(quiet, noisy));
    EXPECT_FALSE(auto_aim::detector::tuning::betterMetrics(noisy, quiet));
}

TEST(TuningSearch, SamplingIsReproducibleAndIncludesDefaults)
{
    const auto first = auto_aim::detector::tuning::sampleParameterSets(5, 20260827U);
    const auto second = auto_aim::detector::tuning::sampleParameterSets(5, 20260827U);

    ASSERT_EQ(first.size(), 6U);
    ASSERT_EQ(second.size(), first.size());
    for (std::size_t index = 0; index < first.size(); ++index)
    {
        EXPECT_EQ(
            auto_aim::detector::tuning::parameterKey(first[index]),
            auto_aim::detector::tuning::parameterKey(second[index]));
    }
    EXPECT_EQ(first.front().binaryThreshold, 90);
    EXPECT_GE(first.back().binaryThreshold, 50);
    EXPECT_LE(first.back().binaryThreshold, 150);
}

TEST(TuningSearch, LocalRefinementMovesTowardBetterValueWithinBounds)
{
    ParameterSet seed;
    seed.binaryThreshold = 50;
    const auto evaluator = [](const ParameterSet& parameters) {
        Metrics metrics;
        metrics.f1 = 1.0 - std::abs(parameters.binaryThreshold - 90) / 100.0;
        metrics.recall = metrics.f1;
        metrics.precision = metrics.f1;
        return metrics;
    };

    const std::vector<ParameterSet> refined =
        auto_aim::detector::tuning::locallyRefine({seed}, evaluator, {}, {0.40});
    ASSERT_EQ(refined.size(), 1U);
    EXPECT_EQ(refined[0].binaryThreshold, 90);
}

TEST(TuningSearch, StabilityUsesWorstSingleParameterPerturbation)
{
    ParameterSet parameters;
    const auto evaluator = [](const ParameterSet& candidate) {
        Metrics metrics;
        metrics.f1 = 1.0 - std::abs(candidate.binaryThreshold - 90) / 100.0;
        return metrics;
    };

    const auto stability = auto_aim::detector::tuning::evaluateStability(
        parameters, evaluator, {}, 0.02);
    EXPECT_NEAR(stability.worstF1, 0.98, 1e-12);
}

} // namespace

#include "lightbar_detector.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

namespace
{

using auto_aim::detector::Armor;
using auto_aim::detector::ArmorType;
using auto_aim::detector::ContourCandidate;
using auto_aim::detector::LightBar;
using auto_aim::detector::LightColor;
using auto_aim::detector::LightbarDetector;
using auto_aim::detector::PreprocessResult;

constexpr int kFrameSize = 320;

cv::Mat makeFrame(const cv::Scalar& color = cv::Scalar(80, 80, 255))
{
    return cv::Mat(kFrameSize, kFrameSize, CV_8UC3, color).clone();
}

ContourCandidate makeCandidate(
    float centerX,
    float centerY,
    float width,
    float height,
    double area,
    float lineAngleDeg = 0.0F,
    bool zeroDirection = false)
{
    const cv::RotatedRect rect(
        cv::Point2f(centerX, centerY),
        cv::Size2f(width, height),
        0.0F);
    cv::Point2f vertices[4];
    rect.points(vertices);

    std::vector<cv::Point> contour;
    contour.reserve(4);
    for (const cv::Point2f& vertex : vertices)
    {
        contour.emplace_back(cvRound(vertex.x), cvRound(vertex.y));
    }

    cv::Vec4f centerLine;
    if (zeroDirection)
    {
        centerLine = cv::Vec4f(0.0F, 0.0F, centerX, centerY);
    }
    else
    {
        const float radians = lineAngleDeg * static_cast<float>(CV_PI / 180.0);
        centerLine = cv::Vec4f(
            std::sin(radians),
            std::cos(radians),
            centerX,
            centerY);
    }

    return ContourCandidate{contour, rect, centerLine, area};
}

bool hasCandidateNear(const PreprocessResult& result, const cv::Point2f& expectedCenter)
{
    return std::any_of(
        result.candidates.begin(),
        result.candidates.end(),
        [&](const ContourCandidate& candidate) {
            return cv::norm(candidate.rect.center - expectedCenter) < 0.1;
        });
}

LightBar makeVerticalLight(
    float centerX,
    float centerY,
    float length = 20.0F,
    float angle = 0.0F)
{
    return LightBar{
        LightColor::Red,
        {centerX, centerY - length * 0.5F},
        {centerX, centerY + length * 0.5F},
        {centerX, centerY},
        length,
        angle,
    };
}

bool connects(const Armor& armor, float leftX, float rightX)
{
    return armor.leftLight.center.x == leftX && armor.rightLight.center.x == rightX;
}

TEST(LightbarPreprocess, ThresholdsPixelsAndBuildsOnlyValidContours)
{
    cv::Mat frame = cv::Mat::zeros(160, 160, CV_8UC3);
    cv::rectangle(frame, cv::Rect(20, 20, 8, 32), cv::Scalar(80, 80, 255), cv::FILLED);
    cv::rectangle(frame, cv::Rect(60, 20, 8, 32), cv::Scalar(255, 80, 80), cv::FILLED);
    cv::rectangle(frame, cv::Rect(100, 20, 8, 32), cv::Scalar(0, 0, 100), cv::FILLED);
    frame.at<cv::Vec3b>(140, 140) = cv::Vec3b(255, 255, 255);

    LightbarDetector detector;
    detector.binaryThreshold = 90;
    const PreprocessResult result = detector.preprocess(frame);

    EXPECT_EQ(result.binary.type(), CV_8UC1);
    EXPECT_EQ(result.binary.size(), frame.size());
    EXPECT_EQ(result.binary.at<std::uint8_t>(35, 23), 255);
    EXPECT_EQ(result.binary.at<std::uint8_t>(35, 63), 255);
    EXPECT_EQ(result.binary.at<std::uint8_t>(35, 103), 0);
    EXPECT_EQ(result.binary.at<std::uint8_t>(0, 0), 0);
    EXPECT_EQ(result.binary.at<std::uint8_t>(140, 140), 255);

    ASSERT_EQ(result.candidates.size(), 2U);
    EXPECT_TRUE(hasCandidateNear(result, cv::Point2f(23.5F, 35.5F)));
    EXPECT_TRUE(hasCandidateNear(result, cv::Point2f(63.5F, 35.5F)));
    for (const ContourCandidate& candidate : result.candidates)
    {
        EXPECT_GT(candidate.area, 0.0);
        EXPECT_GT(std::hypot(candidate.centerLine[0], candidate.centerLine[1]), 0.0);
    }
}

TEST(LightbarPreprocess, UsesStrictGreaterThanBinaryThreshold)
{
    const cv::Mat frame(3, 3, CV_8UC3, cv::Scalar(100, 100, 100));
    LightbarDetector detector;

    detector.binaryThreshold = 99;
    EXPECT_EQ(detector.preprocess(frame).binary.at<std::uint8_t>(1, 1), 255);

    detector.binaryThreshold = 100;
    EXPECT_EQ(detector.preprocess(frame).binary.at<std::uint8_t>(1, 1), 0);
}

TEST(LightbarFilter, KeepsValidRedCandidateAndBuildsOrderedEndpoints)
{
    const cv::Mat frame = makeFrame();
    PreprocessResult pre;
    pre.candidates.push_back(makeCandidate(80.0F, 80.0F, 4.0F, 20.0F, 80.0));
    const LightbarDetector detector;

    const std::vector<LightBar> lights = detector.filterLightBars(frame, pre);

    ASSERT_EQ(lights.size(), 1U);
    EXPECT_EQ(lights[0].color, LightColor::Red);
    EXPECT_FLOAT_EQ(lights[0].center.x, 80.0F);
    EXPECT_FLOAT_EQ(lights[0].center.y, 80.0F);
    EXPECT_FLOAT_EQ(lights[0].length, 20.0F);
    EXPECT_NEAR(lights[0].angle, 0.0F, 1e-5F);
    EXPECT_NEAR(lights[0].top.x, 80.0F, 1e-5F);
    EXPECT_NEAR(lights[0].top.y, 70.0F, 1e-5F);
    EXPECT_NEAR(lights[0].bottom.x, 80.0F, 1e-5F);
    EXPECT_NEAR(lights[0].bottom.y, 90.0F, 1e-5F);
    EXPECT_LT(lights[0].top.y, lights[0].bottom.y);
}

TEST(LightbarFilter, SelectsOnlyTheConfiguredColorAfterPreprocess)
{
    cv::Mat frame = cv::Mat::zeros(120, 140, CV_8UC3);
    cv::rectangle(frame, cv::Rect(20, 30, 8, 32), cv::Scalar(80, 80, 255), cv::FILLED);
    cv::rectangle(frame, cv::Rect(90, 30, 8, 32), cv::Scalar(255, 80, 80), cv::FILLED);

    LightbarDetector detector;
    detector.binaryThreshold = 90;
    const PreprocessResult pre = detector.preprocess(frame);
    ASSERT_EQ(pre.candidates.size(), 2U);

    const std::vector<LightBar> redLights = detector.filterLightBars(frame, pre);
    ASSERT_EQ(redLights.size(), 1U);
    EXPECT_EQ(redLights[0].color, LightColor::Red);
    EXPECT_NEAR(redLights[0].center.x, 23.5F, 0.1F);

    detector.filterParams.targetColor = LightColor::Blue;
    const std::vector<LightBar> blueLights = detector.filterLightBars(frame, pre);
    ASSERT_EQ(blueLights.size(), 1U);
    EXPECT_EQ(blueLights[0].color, LightColor::Blue);
    EXPECT_NEAR(blueLights[0].center.x, 93.5F, 0.1F);
}

TEST(LightbarFilter, RejectsCandidateWithEqualRedAndBlueMeans)
{
    const cv::Mat frame = makeFrame(cv::Scalar(180, 80, 180));
    PreprocessResult pre;
    pre.candidates.push_back(makeCandidate(80.0F, 80.0F, 4.0F, 20.0F, 80.0));
    const LightbarDetector detector;

    EXPECT_TRUE(detector.filterLightBars(frame, pre).empty());
}

TEST(LightbarFilter, IncludesCandidateAtExactComputedAngleLimit)
{
    const cv::Mat frame = makeFrame();
    PreprocessResult pre;
    pre.candidates.push_back(
        makeCandidate(80.0F, 80.0F, 4.0F, 20.0F, 80.0, 15.0F));

    const double vx = pre.candidates[0].centerLine[0];
    const double vy = pre.candidates[0].centerLine[1];
    const double angle = std::acos(std::abs(vy) / std::hypot(vx, vy)) * 180.0 / CV_PI;

    LightbarDetector detector;
    detector.filterParams.maxLineAngleDeg = angle;
    EXPECT_EQ(detector.filterLightBars(frame, pre).size(), 1U);

    detector.filterParams.maxLineAngleDeg = std::nextafter(angle, 0.0);
    EXPECT_TRUE(detector.filterLightBars(frame, pre).empty());
}

struct FilterBoundaryCase
{
    std::string name;
    ContourCandidate candidate;
    bool shouldPass = false;
};

class LightbarFilterBoundaryTest : public testing::TestWithParam<FilterBoundaryCase>
{
};

TEST_P(LightbarFilterBoundaryTest, AppliesGeometryLimits)
{
    const FilterBoundaryCase& testCase = GetParam();
    const cv::Mat frame = makeFrame();
    PreprocessResult pre;
    pre.candidates.push_back(testCase.candidate);
    LightbarDetector detector;
    detector.filterParams.minArea = 10.0;
    detector.filterParams.maxArea = 6000.0;
    detector.filterParams.minAspectRatio = 2.0;
    detector.filterParams.maxAspectRatio = 15.0;
    detector.filterParams.minLineAngleDeg = 0.0;
    detector.filterParams.maxLineAngleDeg = 15.0;
    detector.filterParams.minFillRatio = 0.5;
    detector.filterParams.maxFillRatio = 1.0;

    const std::vector<LightBar> lights = detector.filterLightBars(frame, pre);

    EXPECT_EQ(!lights.empty(), testCase.shouldPass);
    EXPECT_LE(lights.size(), 1U);
}

INSTANTIATE_TEST_SUITE_P(
    FilterLimits,
    LightbarFilterBoundaryTest,
    testing::Values(
        FilterBoundaryCase{
            "AreaAtMinimum", makeCandidate(80.0F, 80.0F, 2.0F, 10.0F, 10.0), true},
        FilterBoundaryCase{
            "AreaBelowMinimum", makeCandidate(80.0F, 80.0F, 2.0F, 9.0F, 9.0), false},
        FilterBoundaryCase{
            "AreaAtMaximum", makeCandidate(160.0F, 160.0F, 50.0F, 120.0F, 6000.0), true},
        FilterBoundaryCase{
            "AreaAboveMaximum", makeCandidate(160.0F, 160.0F, 50.0F, 120.0F, 6001.0), false},
        FilterBoundaryCase{
            "AspectRatioAtMinimum", makeCandidate(80.0F, 80.0F, 10.0F, 20.0F, 200.0), true},
        FilterBoundaryCase{
            "AspectRatioBelowMinimum", makeCandidate(80.0F, 80.0F, 10.0F, 19.9F, 199.0), false},
        FilterBoundaryCase{
            "AspectRatioAtMaximum", makeCandidate(80.0F, 80.0F, 4.0F, 60.0F, 240.0), true},
        FilterBoundaryCase{
            "AspectRatioAboveMaximum", makeCandidate(80.0F, 80.0F, 4.0F, 60.4F, 241.6), false},
        FilterBoundaryCase{
            "AngleAtMinimum", makeCandidate(80.0F, 80.0F, 4.0F, 20.0F, 80.0, 0.0F), true},
        FilterBoundaryCase{
            "AngleAboveMaximum", makeCandidate(80.0F, 80.0F, 4.0F, 20.0F, 80.0, 15.1F), false},
        FilterBoundaryCase{
            "FillRatioAtMinimum", makeCandidate(80.0F, 80.0F, 10.0F, 20.0F, 100.0), true},
        FilterBoundaryCase{
            "FillRatioBelowMinimum", makeCandidate(80.0F, 80.0F, 10.0F, 20.0F, 99.8), false},
        FilterBoundaryCase{
            "FillRatioAtMaximum", makeCandidate(80.0F, 80.0F, 4.0F, 20.0F, 80.0), true},
        FilterBoundaryCase{
            "ZeroWidth", makeCandidate(80.0F, 80.0F, 0.0F, 20.0F, 10.0), false},
        FilterBoundaryCase{
            "ZeroDirection", makeCandidate(80.0F, 80.0F, 4.0F, 20.0F, 80.0, 0.0F, true), false}),
    [](const testing::TestParamInfo<FilterBoundaryCase>& info) {
        return info.param.name;
    });

TEST(LightbarMatcher, NormalizesInputOrderAndBuildsSmallArmor)
{
    const LightbarDetector detector;
    const LightBar left = makeVerticalLight(100.0F, 80.0F);
    const LightBar right = makeVerticalLight(160.0F, 80.0F);

    const std::vector<Armor> armors = detector.matchArmors({right, left});

    ASSERT_EQ(armors.size(), 1U);
    EXPECT_FLOAT_EQ(armors[0].leftLight.center.x, 100.0F);
    EXPECT_FLOAT_EQ(armors[0].rightLight.center.x, 160.0F);
    EXPECT_FLOAT_EQ(armors[0].center.x, 130.0F);
    EXPECT_FLOAT_EQ(armors[0].center.y, 80.0F);
    EXPECT_EQ(armors[0].type, ArmorType::Small);
}

TEST(LightbarMatcher, ClassifiesExactLargeArmorBoundaryAsLarge)
{
    const LightbarDetector detector;
    const LightBar left = makeVerticalLight(100.0F, 80.0F);
    const LightBar right = makeVerticalLight(180.0F, 80.0F);

    const std::vector<Armor> armors = detector.matchArmors({left, right});

    ASSERT_EQ(armors.size(), 1U);
    EXPECT_EQ(armors[0].type, ArmorType::Large);
}

struct GeometryBoundaryCase
{
    std::string name;
    LightBar left;
    LightBar right;
    bool shouldMatch = false;
};

class LightbarMatcherGeometryBoundaryTest
    : public testing::TestWithParam<GeometryBoundaryCase>
{
};

TEST_P(LightbarMatcherGeometryBoundaryTest, AppliesInclusiveLimits)
{
    const GeometryBoundaryCase& testCase = GetParam();
    const LightbarDetector detector;

    const std::vector<Armor> armors = detector.matchArmors({testCase.left, testCase.right});

    EXPECT_EQ(!armors.empty(), testCase.shouldMatch);
    EXPECT_LE(armors.size(), 1U);
}

INSTANTIATE_TEST_SUITE_P(
    GeometryLimits,
    LightbarMatcherGeometryBoundaryTest,
    testing::Values(
        GeometryBoundaryCase{
            "LengthRatioAtMaximum",
            makeVerticalLight(100.0F, 100.0F, 20.0F),
            makeVerticalLight(175.0F, 100.0F, 30.0F),
            true},
        GeometryBoundaryCase{
            "LengthRatioAboveMaximum",
            makeVerticalLight(100.0F, 100.0F, 20.0F),
            makeVerticalLight(175.0F, 100.0F, 30.2F),
            false},
        GeometryBoundaryCase{
            "AngleDifferenceAtMaximum",
            makeVerticalLight(100.0F, 100.0F, 20.0F, 0.0F),
            makeVerticalLight(160.0F, 100.0F, 20.0F, 10.0F),
            true},
        GeometryBoundaryCase{
            "AngleDifferenceAboveMaximum",
            makeVerticalLight(100.0F, 100.0F, 20.0F, 0.0F),
            makeVerticalLight(160.0F, 100.0F, 20.0F, 10.1F),
            false},
        GeometryBoundaryCase{
            "CenterYDifferenceAtMaximum",
            makeVerticalLight(100.0F, 100.0F),
            makeVerticalLight(150.0F, 130.0F),
            true},
        GeometryBoundaryCase{
            "CenterYDifferenceAboveMaximum",
            makeVerticalLight(100.0F, 100.0F),
            makeVerticalLight(150.0F, 130.1F),
            false},
        GeometryBoundaryCase{
            "CenterDistanceAtMinimum",
            makeVerticalLight(100.0F, 100.0F),
            makeVerticalLight(140.0F, 100.0F),
            true},
        GeometryBoundaryCase{
            "CenterDistanceBelowMinimum",
            makeVerticalLight(100.0F, 100.0F),
            makeVerticalLight(139.9F, 100.0F),
            false},
        GeometryBoundaryCase{
            "CenterDistanceAtMaximum",
            makeVerticalLight(100.0F, 100.0F),
            makeVerticalLight(220.0F, 100.0F),
            true},
        GeometryBoundaryCase{
            "CenterDistanceAboveMaximum",
            makeVerticalLight(100.0F, 100.0F),
            makeVerticalLight(220.1F, 100.0F),
            false}),
    [](const testing::TestParamInfo<GeometryBoundaryCase>& info) {
        return info.param.name;
    });

TEST(LightbarMatcher, RejectsZeroLengthLight)
{
    const LightbarDetector detector;
    const LightBar zeroLength = makeVerticalLight(100.0F, 100.0F, 0.0F);
    const LightBar normal = makeVerticalLight(160.0F, 100.0F);

    EXPECT_TRUE(detector.matchArmors({zeroLength, normal}).empty());
}

TEST(LightbarMatcher, MiddleLightBlocksOnlyTheOuterPair)
{
    const LightbarDetector detector;
    const LightBar left = makeVerticalLight(100.0F, 100.0F);
    const LightBar middle = makeVerticalLight(140.0F, 100.0F);
    const LightBar right = makeVerticalLight(180.0F, 100.0F);

    const std::vector<Armor> armors = detector.matchArmors({left, middle, right});

    ASSERT_EQ(armors.size(), 2U);
    EXPECT_TRUE(std::any_of(armors.begin(), armors.end(), [](const Armor& armor) {
        return connects(armor, 100.0F, 140.0F);
    }));
    EXPECT_TRUE(std::any_of(armors.begin(), armors.end(), [](const Armor& armor) {
        return connects(armor, 140.0F, 180.0F);
    }));
    EXPECT_FALSE(std::any_of(armors.begin(), armors.end(), [](const Armor& armor) {
        return connects(armor, 100.0F, 180.0F);
    }));
}

} // namespace

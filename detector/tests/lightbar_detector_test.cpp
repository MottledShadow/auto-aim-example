#include "lightbar_detector.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{

using auto_aim::detector::Armor;
using auto_aim::detector::ArmorType;
using auto_aim::detector::LightBar;
using auto_aim::detector::LightColor;
using auto_aim::detector::LightbarDetector;

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

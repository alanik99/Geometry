#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "utils3d.hpp"

TEST(DistanceTestsSuite, SkewSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0),  utils3D::Point(1.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(0.0, 2.4, -1.0), utils3D::Point(0.0, 2.4, 3.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(2.4));
}

TEST(DistanceTestsSuite, IntersectingSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0), utils3D::Point(1.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(1.0, 1.0, 1.0), utils3D::Point(-1.0, -1.0, -1.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(0.0));
}

TEST(DistanceTestsSuite, FullyOverlappingSegments) {
    const utils3D::Segment seg1(utils3D::Point(1.0, 1.0, 1.0), utils3D::Point(3.0, 3.0, 3.0));
    const utils3D::Segment seg2(utils3D::Point(1.0, 1.0, 1.0), utils3D::Point(3.0, 3.0, 3.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(0.0));
}

TEST(DistanceTestsSuite, PartlyOverlappingSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0), utils3D::Point(3.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(2.0, 0.0, 0.0), utils3D::Point(4.0, 0.0, 0.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(0.0));
}

TEST(DistanceTestsSuite, NonOverlappingSameLineSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0),  utils3D::Point(1.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(3.17, 0.0, 0.0), utils3D::Point(4.0, 0.0, 0.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(2.17));
}

TEST(DistanceTestsSuite, OverlappingParallelSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0), utils3D::Point(1.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(0.0, 1.1, 0.0), utils3D::Point(5.0, 1.1, 0.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(1.1));
}

TEST(DistanceTestsSuite, NonOverlappingParallelSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0), utils3D::Point(1.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(2.0, 1.0, 0.0), utils3D::Point(4.0, 1.0, 0.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(std::sqrt(2)));
}

TEST(DistanceTestsSuite, NonOverlappingNonParallelSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0), utils3D::Point(1.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(2.0, 1.0, 0.0), utils3D::Point(4.0, 1.5, 3.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_TRUE(d.has_value());
    EXPECT_THAT(d.value(), testing::DoubleEq(std::sqrt(2)));
}

TEST(DistanceTestsSuite, PointSegments) {
    const utils3D::Segment seg1(utils3D::Point(0.0, 0.0, 0.0), utils3D::Point(0.0, 0.0, 0.0));
    const utils3D::Segment seg2(utils3D::Point(1.0, 1.0, 1.0), utils3D::Point(1.0, 1.0, 1.0));

    const auto d = utils3D::distance(seg1, seg2);
    ASSERT_FALSE(d.has_value());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/**
 * @author Lucas Souchet - lsouchet@aldebaran.com
 * Aldebaran Robotics (c) 2016 All Rights Reserved
 *
 */

#include <gtest/gtest.h>
#include <almath/types/occupancymapparams.h>
#include <almath/tools/altrigonometry.h>

using namespace AL;
using namespace Math;
class OccupancyMapParamsTest: public ::testing::Test {
  public:
    OccupancyMapParamsTest() {}
};

TEST_F(OccupancyMapParamsTest, ctor) {
  OccupancyMapParams params(100, 0.1f, Position2D(1.0f, -1.0f));
  EXPECT_EQ(params.size, 100);
  EXPECT_NEAR(params.metersPerPixel, 0.1f, 1e-5f);
  EXPECT_TRUE(params.originOffset.isNear(Position2D(-4.0f, 4.0f)));
}

TEST_F(OccupancyMapParamsTest, getPixelFromPose) {
  OccupancyMapParams params(100, 0.1f, Position2D(1.0f, -1.0f));
  Point2Di px = params.getPixelFromPose(Pose2D(1.0f, -1.0f, 0.0f));
  EXPECT_EQ(px.x, 50);
  EXPECT_EQ(px.y, 50);
  px = params.getPixelFromPose(Pose2D(1.0f, -1.0f, PI_2));
  EXPECT_EQ(px.x, 50);
  EXPECT_EQ(px.y, 50);
  px = params.getPixelFromPose(Pose2D(2.0f, -1.0f, 0.0f));
  EXPECT_EQ(px.x, 60);
  EXPECT_EQ(px.y, 50);
  px = params.getPixelFromPose(Pose2D(2.0f, -2.0f, 0.0f));
  EXPECT_EQ(px.x, 60);
  EXPECT_EQ(px.y, 60);
  px = params.getPixelFromPose(Pose2D(0.0f, 0.0f, 0.0f));
  EXPECT_EQ(px.x, 40);
  EXPECT_EQ(px.y, 40);
}

TEST_F(OccupancyMapParamsTest, getPixelFromPosition) {
  OccupancyMapParams params(100, 0.1f, Position2D(1.0f, -1.0f));
  Point2Di px = params.getPixelFromPosition(Position2D(1.0f, -1.0f));
  EXPECT_EQ(px.x, 50);
  EXPECT_EQ(px.y, 50);
  px = params.getPixelFromPosition(Position2D(1.0f, -1.0f));
  EXPECT_EQ(px.x, 50);
  EXPECT_EQ(px.y, 50);
  px = params.getPixelFromPosition(Position2D(2.0f, -1.0f));
  EXPECT_EQ(px.x, 60);
  EXPECT_EQ(px.y, 50);
  px = params.getPixelFromPosition(Position2D(2.0f, -2.0f));
  EXPECT_EQ(px.x, 60);
  EXPECT_EQ(px.y, 60);
  px = params.getPixelFromPosition(Position2D(0.0f, 0.0f));
  EXPECT_EQ(px.x, 40);
  EXPECT_EQ(px.y, 40);
}

TEST_F(OccupancyMapParamsTest, getPoseFromPixel) {
   OccupancyMapParams params(100, 0.1f, Position2D(1.0f, -1.0f));
   Pose2D p = params.getPoseFromPixel(Pose2Di(50, 50, PI_2));
   EXPECT_NEAR(p.x, 1.0f, 1e-3f);
   EXPECT_NEAR(p.y, -1.0f, 1e-3f);
   EXPECT_NEAR(p.theta, PI_2, 1e-3f);
   p = params.getPoseFromPixel(Pose2Di(0, 0, PI_2));
   EXPECT_NEAR(p.x, -4.0f, 1e-3f);
   EXPECT_NEAR(p.y, 4.0f, 1e-3f);
   EXPECT_NEAR(p.theta, PI_2, 1e-3f);
   p = params.getPoseFromPixel(Pose2Di(100, 0, PI_2));
   EXPECT_NEAR(p.x, 6.0f, 1e-3f);
   EXPECT_NEAR(p.y, 4.0f, 1e-3f);
   EXPECT_NEAR(p.theta, PI_2, 1e-3f);
   p = params.getPoseFromPixel(Pose2Di(100, 100, PI_2));
   EXPECT_NEAR(p.x, 6.0f, 1e-3f);
   EXPECT_NEAR(p.y, -6.0f, 1e-3f);
   EXPECT_NEAR(p.theta, PI_2, 1e-3f);
   p = params.getPoseFromPixel(Pose2Di(0, 100, PI_2));
   EXPECT_NEAR(p.x, -4.0f, 1e-3f);
   EXPECT_NEAR(p.y, -6.0f, 1e-3f);
   EXPECT_NEAR(p.theta, PI_2, 1e-3f);
   p = params.getPoseFromPixel(Pose2Di(200, 50, PI_2));
   EXPECT_NEAR(p.x, 16.0f, 1e-3f);
   EXPECT_NEAR(p.y, -1.0f, 1e-3f);
   EXPECT_NEAR(p.theta, PI_2, 1e-3f);
}

TEST_F(OccupancyMapParamsTest, getPositionFromPixel) {
   OccupancyMapParams params(100, 0.1f, Position2D(1.0f, -1.0f));
   Position2D p = params.getPositionFromPixel(Point2Di(50, 50));
   EXPECT_NEAR(p.x, 1.0f, 1e-3f);
   EXPECT_NEAR(p.y, -1.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(0, 0));
   EXPECT_NEAR(p.x, -4.0f, 1e-3f);
   EXPECT_NEAR(p.y, 4.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(100, 0));
   EXPECT_NEAR(p.x, 6.0f, 1e-3f);
   EXPECT_NEAR(p.y, 4.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(100, 100));
   EXPECT_NEAR(p.x, 6.0f, 1e-3f);
   EXPECT_NEAR(p.y, -6.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(0, 100));
   EXPECT_NEAR(p.x, -4.0f, 1e-3f);
   EXPECT_NEAR(p.y, -6.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(200, 50));
   EXPECT_NEAR(p.x, 16.0f, 1e-3f);
   EXPECT_NEAR(p.y, -1.0f, 1e-3f);
}

TEST_F(OccupancyMapParamsTest, getPositionFromScreenPixel) {
   OccupancyMapParams params(100, 0.1f, Position2D(1.0f, -1.0f));
   Position2D p = params.getPositionFromPixel(Point2Di(50, 50));
   EXPECT_NEAR(p.x, 1.0f, 1e-3f);
   EXPECT_NEAR(p.y, -1.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(0, 0));
   EXPECT_NEAR(p.x, -4.0f, 1e-3f);
   EXPECT_NEAR(p.y, 4.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(100, 0));
   EXPECT_NEAR(p.x, 6.0f, 1e-3f);
   EXPECT_NEAR(p.y, 4.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(100, 100));
   EXPECT_NEAR(p.x, 6.0f, 1e-3f);
   EXPECT_NEAR(p.y, -6.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(0, 100));
   EXPECT_NEAR(p.x, -4.0f, 1e-3f);
   EXPECT_NEAR(p.y, -6.0f, 1e-3f);
   p = params.getPositionFromPixel(Point2Di(200, 50));
   EXPECT_NEAR(p.x, 16.0f, 1e-3f);
   EXPECT_NEAR(p.y, -1.0f, 1e-3f);
}

TEST_F(OccupancyMapParamsTest, inverse) {
  OccupancyMapParams params(100, 0.1f, Position2D(1.0f, -1.0f));
  Position2D start(2.65f, 3.14f);
  Position2D final = params.getPositionFromPixel(
        params.getPixelFromPosition(start));
  EXPECT_NEAR(start.x, final.x, 6e-2f);
  EXPECT_NEAR(start.y, final.y, 6e-2f);

  Point2Di startI(44, 32);
  Point2Di finalI = params.getPixelFromPosition(
        params.getPositionFromPixel(startI));
  EXPECT_EQ(startI.x, finalI.x);
  EXPECT_EQ(startI.y, finalI.y);
}

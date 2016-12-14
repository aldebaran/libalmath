/**
 * @author Lucas Souchet - lsouchet@aldebaran.com
 * Aldebaran Robotics (c) 2016 All Rights Reserved
 *
 */

#include <almath/types/occupancymapparams.h>

namespace AL {
namespace Math {

OccupancyMapParams::OccupancyMapParams(int size,
    float metersPerPixel, const Math::Position2D& mapCenter,
    float obstacleThreshold)
  : size(size),
    metersPerPixel(metersPerPixel),
    obstacleProbabilityThreshold(obstacleThreshold) {
  initOriginOffset(mapCenter);
}

void OccupancyMapParams::initOriginOffset(const Math::Position2D& mapCenter) {
  originOffset.x = mapCenter.x - (size / 2.0f * metersPerPixel);
  originOffset.y = mapCenter.y + (size / 2.0f * metersPerPixel);
  // Transformation between the map center and origin offset is a translation:
  // X = size/2*mpp
  // Y = size/2*mpp, and
  // X = x
  // Y = -y
  // Because the frame robot and the image have an inverted y, see below:
  // robot:                      image:
  //     y                           --> X
  //     ^                          |
  //     |                         \/
  //     --> x                       Y
}

Point2Di OccupancyMapParams::getPixelFromPose(
    const Pose2D& pose) const {
  return getPixelFromOffsetAndScale(pose, metersPerPixel, originOffset);
}


// TODO: improve performance of this.
Point2Di OccupancyMapParams::getPixelFromPosition(
    const Position2D &position) const {
  return getPixelFromOffsetAndScale(pose2DFromPosition2D(position),
                                    metersPerPixel, originOffset);
}

Pose2D OccupancyMapParams::getPoseFromPixel(const Pose2Di& pixel) const {
  return Pose2D(pixel.x * metersPerPixel + originOffset.x,
                      -pixel.y * metersPerPixel + originOffset.y, pixel.theta);
}

Position2D OccupancyMapParams::getPositionFromPixel(
    const Point2Di &pixel) const {
  return Position2D(pixel.x * metersPerPixel + originOffset.x,
                    -pixel.y * metersPerPixel + originOffset.y);
}

Point2Di OccupancyMapParams::getDeltaPixelFromDeltaPosition(
    const Position2D &position) const {
  // Give an offset of (0.0, 0.0) to get the delta.
  return getPixelFromOffsetAndScale(AL::Math::pose2DFromPosition2D(position),
                                    metersPerPixel, Position2D());
}

float OccupancyMapParams::getObstacleProbabilityThreshold() const {
  return obstacleProbabilityThreshold;
}

}
}

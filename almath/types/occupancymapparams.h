#ifndef OCCUPANCYMAPPARAMS_H
#define OCCUPANCYMAPPARAMS_H

#include <almath/tools/almath.h>
#include <almath/api.h>

namespace AL {
namespace Math {

  /// <summary>
  /// Create a Point2Di.
  /// Point2Di are used on objects that deal with pixels only, and should
  /// not care about metric distances.
  ///
  /// A Point2Di is just defined by integers x and y.
  /// </summary>
  /// \ingroup Types
  struct ALMATH_API Point2Di {
    explicit Point2Di(int _x=0, int _y=0): x(_x), y(_y) {}
    int x;
    int y;
    bool operator == (const Point2Di& other) const {
      return (x == other.x && y == other.y);
    }
  };

  /// <summary>
  /// Create a Pose2Di.
  /// Pose2Di are used on objects that deal with pixels only, and should
  /// not care about metric distances.
  ///
  /// A Pose2Di is defined by integers x and y, and a theta for orientation.
  /// </summary>
  /// \ingroup Types
  struct ALMATH_API Pose2Di {
    explicit Pose2Di(int _x=0, int _y=0, float _angle=0.0f)
        : x(_x), y(_y), theta(_angle) {}
    explicit Pose2Di(const Point2Di& pt, float _angle=0.0f)
        : x(pt.x), y(pt.y), theta(_angle) {}

    int x;
    int y;
    float theta;
    bool operator == (const Pose2Di& other) const {
      return (x == other.x && y == other.y && std::abs(theta - other.theta) < 1e-6f);
    }
  };

  typedef std::vector<Pose2D> Pose2DVect;
  typedef std::vector<Pose2Di> Pose2DiVect;

  static inline int roundToInt(float x) {
    if (x >= 0.0f) {
      return static_cast<int>(x + 0.5f);
    }
    return static_cast<int>(x - 0.5f);
  }

  /// <summary>
  /// Create a OccupancyMapParams.
  /// OccupancyMapParams are used used to define occupancy maps and to provide
  /// helpers to change from metrical to pixel frames.
  /// Maps are squared.
  ///
  /// </summary>
  /// \ingroup Types
struct ALMATH_API OccupancyMapParams {
  explicit OccupancyMapParams(int size, float metersPerPixel,
      const Position2D &mapCenter, float obstacleThreshold = 0.25f);

  /// <summary>
  /// Set the origin offset accordingly to the given desired metrical
  /// position of the central pixel.
  /// </summary>
  /// <param name="mapCenter"> desired metrical position of the central pixel. </param>
  void initOriginOffset(const Position2D &mapCenter);

  static inline Point2Di getPixelFromOffsetAndScale(
      const Math::Pose2D& pose,
      float metersPerPixel,
      const Math::Position2D& originOffset) {
    const float kPixelPerMeter = 1.0f / metersPerPixel;

    const float xx = (pose.x - originOffset.x) * kPixelPerMeter;
    const int x = roundToInt(xx);

    const float yy = -(pose.y - originOffset.y) * kPixelPerMeter;
    const int y = roundToInt(yy);
    return Point2Di(x, y);
  }

  /// <summary>
  /// Get the Point2Di in the map corresponding to the given metrical pose.
  /// </summary>
  /// <param name="pose"> desired metrical Pose2D. </param>
  /// <returns> Point2Di of the associated pixel. </returns>
  Point2Di getPixelFromPose(const Pose2D& pose) const;

  /// <summary>
  /// Get the Point2Di in the map corresponding to the given metrical position.
  /// </summary>
  /// <param name="position"> desired metrical Position2D. </param>
  /// <returns> Point2Di of the associated pixel. </returns>
  Point2Di getPixelFromPosition(const Position2D& position) const;

  /// <summary>
  /// Get the Pose2D corresponding to the given pixel pose in the map.
  /// </summary>
  /// <param name="pixel"> desired pixel pose. </param>
  /// <returns> Pose2D of the associated pose. </returns>
  Pose2D getPoseFromPixel(const Pose2Di &pixel) const;

  /// <summary>
  /// Get the Pose2D corresponding to the given pixel pose in the map.
  /// </summary>
  /// <param name="pixel"> desired pixel pose. </param>
  /// <returns> Position2D of the associated position. </returns>
  Position2D getPositionFromPixel(const Point2Di &pixel) const;

  /// <summary>
  /// Get the Point2Di representing the pixel difference corresponding to the
  /// given metrical difference.
  /// </summary>
  /// <param name="position"> Metrical delta. </param>
  /// <returns> Point2Di of the corresponding pixel difference. </returns>
  Point2Di getDeltaPixelFromDeltaPosition(const Position2D& position) const;

  /// <summary> Get the maximum pixel value of an obstacle in the map.
  /// </summary>
  float getObstacleProbabilityThreshold() const;

  /// <summary> Pixel size of the associated squared occupancy map. </summary>
  int size;
  /// <summary> Factor representing the metrical size of each pixel.</summary>
  float metersPerPixel;
  /// <summary> Metrical coordinate of the (0, 0) pixel.</summary>
  Position2D originOffset;
  /// <summary> Maximum pixel value of an obstacle in the map </summary>
  float obstacleProbabilityThreshold;
};
} // Math
} // AL

#endif // OCCUPANCYMAPPARAMS_H


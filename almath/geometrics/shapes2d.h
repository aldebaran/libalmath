#ifndef LIB_ALMATH_GEOMETRICS_SHAPES2D_H
#define LIB_ALMATH_GEOMETRICS_SHAPES2D_H

#include <almath/api.h>
#include <vector>
#include <boost/variant.hpp>
#include <Eigen/Dense>

namespace AL {
namespace Math {

class ALMATH_API Plane;

bool operator==(const Plane &, const Plane &);
bool operator!=(const Plane &, const Plane &);

class ALMATH_API ConvexPolygon
{
// Note: this class only deals with simple, convex polygns.
// For complex polygons, this library is worth considering
// https://doc.cgal.org/latest/Polygon/index.html#Chapter_2D_Polygon
public:
  using Vector2 = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
  // construct a convex polygon from its outline.
  // The outline is specified by a list of vertices in clockwise or
  // counterclockwise order.
  // It is not necessary for the start and end points to coincide;
  // if they do not, the polygon will be automatically closed.
  ConvexPolygon(std::vector<Vector2> outline);
  ConvexPolygon() = default;
  // return the outline of the polygon, with degenerate sides cleaned.
  const std::vector<Vector2> &outline() const { return _outline; }
private:
  std::vector<Vector2> _outline;
};

// beware: identical polygons with different (but equivalent) outlines will
// be reported as different (eg. [A, B, C] vs [B, C, A]).
inline bool operator==(const ConvexPolygon &lhs, const ConvexPolygon &rhs)
{
  return lhs.outline() == rhs.outline();
}

inline bool operator!=(const ConvexPolygon &lhs, const ConvexPolygon &rhs)
{
  return !(lhs == rhs);
}

inline ConvexPolygon operator*(const Eigen::Isometry2d &lhs,
                               const ConvexPolygon &rhs)
{
  return ConvexPolygon{lhs * rhs.outline()};
}

ALMATH_API
bool isInside(const ConvexPolygon &shape, const Eigen::Vector2d &point);


class ALMATH_API ShapesUnion : public boost::static_visitor<>
{
  std::vector<boost::variant<Plane, ConvexPolygon>> _shapes;
  // note: we could keep _shapes sorted in a flat_set to make ShapesUnion
  // instances (more) meaningfully comparable.
  bool isWholePlane() const;
public:
  const std::vector<boost::variant<Plane, ConvexPolygon>> &shapes() const
  {
    return _shapes;
  }

  void operator()(const Plane &);

  void operator()(const ConvexPolygon &s);

  void operator()(const ShapesUnion &s);
};

ALMATH_API
bool operator==(const ShapesUnion &lhs, const ShapesUnion &rhs);

ALMATH_API boost::variant<Plane, ConvexPolygon, ShapesUnion>
shapeFromJSON(std::istream &);


ALMATH_API void writeAsJSON(std::ostream &os, const Plane &shape);

ALMATH_API void writeAsJSON(std::ostream &os, const ConvexPolygon &shape);

ALMATH_API void writeAsJSON(std::ostream &os, const ShapesUnion &shape);

ALMATH_API void writeAsJSON(
    std::ostream &os,
    const boost::variant<Plane, ConvexPolygon, ShapesUnion> &shape);

}  // End namespace Math.
}  // End namespace AL.

#endif  // LIB_ALMATH_GEOMETRICS_SHAPES2D_H

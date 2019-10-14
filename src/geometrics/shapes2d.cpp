#include <cassert>
#include <iostream>
#include <stdexcept>
#include <almath/geometrics/shapes2d.h>
#include <almath/geometrics/shapes3d.h> // for the Plane type
#include <almath/geometrics/shape.h>
#include <nlohmann/json.hpp>

namespace {

using ShapeVariant = boost::variant<AL::Math::Plane,
                                    AL::Math::ConvexPolygon,
                                    AL::Math::ShapesUnion>;

enum struct OutlineOrientation
{
  Unknown,
  Direct,
  Indirect
};

// return the outline, where:
// * consecutive points whose distance is less than epsDistance and
// * consecutive segments which are aligned (the sinus of their angle is less
//   than epsSinus)
// have been merged.
//
// Throw std::invalid_argument if the outline does not define a convex polygon.
//
// Examples: given the points    A B C
//                               *-*-*
//                                \ /
//                                 * D
// * cleanConvexPolygonOutlineDegenerateSides([A, C, D])    ->  [A, C, D]
// * cleanConvexPolygonOutlineDegenerateSides([A, B, C, D]) ->  [A, C, D]
// * cleanConvexPolygonOutlineDegenerateSides([B, C, D, A]) ->  [C, D, A]
// * cleanConvexPolygonOutlineDegenerateSides([C, D, A, B]) ->  [C, D, A]
// * cleanConvexPolygonOutlineDegenerateSides([A, C])       ->  [A, C]
// * cleanConvexPolygonOutlineDegenerateSides([A, B, C])    ->  [A, C]
// * cleanConvexPolygonOutlineDegenerateSides([B, C, A])    ->  [C, A]
// * cleanConvexPolygonOutlineDegenerateSides([A])          ->  [A]
// * cleanConvexPolygonOutlineDegenerateSides([A, A])       ->  [A]
// * cleanConvexPolygonOutlineDegenerateSides([])           ->  []
// * cleanConvexPolygonOutlineDegenerateSides([A, B, D, C]) ->  throws
//
// Note: as can be seen from the examples, the same polygon can be described
// in several equivalent ways, even after clean-up
// (eg. [A, C, D] vs [C, D, A]).
std::vector<AL::Math::ConvexPolygon::Vector2>
cleanConvexPolygonOutlineDegenerateSides(
    std::vector<AL::Math::ConvexPolygon::Vector2> outline,
    double epsSinus,
    double epsDistance)
{
  assert(epsSinus >= 0);
  assert(epsDistance >= 0);
  if (outline.empty())
  {
    // Note: because our JSON format does not support whose outline is empty,
    // we could have chosen to throw here.
    // But having C++ class supports degenerate polygons with 0, 1, 2 edges
    // seems nice, let not throw.
    return outline;
  }
  OutlineOrientation prevOrientation = OutlineOrientation::Unknown;
  auto a = outline.begin();
  auto b = std::next(a);
  while (b != outline.end())
  {
    const Eigen::Vector2d ab = *b - *a;
    const double ab_norm = ab.norm();
    if (ab_norm < epsDistance)
    {
      // erase the ab segment: keep a, remove b.
      // Notes:
      // * we could have kept ab's midpoint instead of a
      // * we check the distance before the alignment,
      //   to avoid throwing
      b = outline.erase(b);
      continue;
    }
    const Eigen::Vector2d ab_normalized = ab/ab_norm;

    auto c = std::next(b);
    if (c == outline.end())
    {
      if (outline.size() <= 2u)
        break;
      c = outline.begin();
      assert(c != a);
    }

    const Eigen::Vector2d bc = *c - *b;
    const double bc_norm = bc.norm();
    if (bc_norm < epsDistance)
    {
      // erase the bc segment: keep c, remove b.
      b = outline.erase(b);
      continue;
    }
    const Eigen::Vector2d bc_normalized = bc/bc_norm;
    const auto sinus = (ab_normalized[0] * bc_normalized[1] -
                        ab_normalized[1] * bc_normalized[0]);
    OutlineOrientation curOrientation = OutlineOrientation::Unknown;
    if (sinus < -epsSinus)
    {
      if (prevOrientation == OutlineOrientation::Direct)
        throw std::invalid_argument("non-convex polygon");
      curOrientation = OutlineOrientation::Indirect;
    }
    else if (sinus > epsSinus)
    {
      if (prevOrientation == OutlineOrientation::Indirect)
        throw std::invalid_argument("non-convex polygon");
      curOrientation = OutlineOrientation::Direct;
    }
    if (curOrientation == OutlineOrientation::Unknown)
    {
      // a, b and c are aligned. Let remove the inner point.
      const double b_curv = ab.norm();
      const double c_curv = ab_normalized.dot(*c - *a);
      if (c_curv >= b_curv)
      {
        // bc goes forward, let remove b
        //     a     b     c
        //     *     *     *
        // ab:  -->--
        // bc:        -->--
        b = outline.erase(b);
      }
      else if (c_curv <= 0.)
      {
        // bc goes backward, c is further than a, let remove a
        //     c     a     b
        //     *     *     *
        // ab:        -->--
        // bc:  -----<-----
        a = outline.erase(a);
        b = std::next(a);
      }
      else
      {
        // bc goes backward, c is between a and b, let remove c
        //     a     c     b
        //     *     *     *
        // ab:  ----->-----
        // bc:        --<--
        c = outline.erase(c);
        if (c == outline.begin())
        {
          // erasing c invalidated a and b.
          // Anyway, we're done.
          break;
        }
      }
    }
    else
    {
      prevOrientation = curOrientation;
      a = b;
      ++b;
    }
  }
  return outline;
}

bool isInsideConvexPolygon(
    const std::vector<AL::Math::ConvexPolygon::Vector2> &outline,
    const Eigen::Vector2d &point,
    double epsSinus,
    double epsDistance)
{
  assert(cleanConvexPolygonOutlineDegenerateSides(outline, epsSinus, epsDistance) == outline);
  // Handle degenerate cases
  const auto n = outline.size();
  if (n == 0u)
    return false;
  if (n == 1u)
    return (point - outline.front()).norm() < epsDistance;
  if (n == 2u) {
    const Eigen::Vector2d &a = outline.front();
    const Eigen::Vector2d &b = outline.back();
    const Eigen::Vector2d &p = point;
    const Eigen::Vector2d ab = b - a;
    const Eigen::Vector2d ap = p - a;
    const double dotprod = ab.dot(ap);
    if (dotprod <= 0)
    {
      // the closest point from p in ab is a
      return ap.norm() < epsDistance;
    }
    else if (dotprod >= ab.squaredNorm())
    {
      // the closest point from p in ab is b
      return (p - b).norm() < epsDistance;
    }
    // TODO: this is suboptimal
    return (a + dotprod/ab.norm() * ab.normalized() - p).norm() < epsDistance;
  }

  // testPoint is inside polygon iff all angles defined by
  // (testPoint, polygon[i]) (testPoint, polygon[i + 1]) have the same sign
  OutlineOrientation orientation = OutlineOrientation::Unknown;
  auto a = outline.begin();
  Eigen::Vector2d pa = (*a - point).normalized();
  while (a != outline.end())
  {
    auto b = std::next(a);
    if (b == outline.end())
      b = outline.begin();
    ;
    const Eigen::Vector2d pb = (*b - point).normalized();
    const float angleDet = pb[0] * pa[1] - pa[0] * pb[1];
    if (angleDet > epsSinus) {
      if (orientation == OutlineOrientation::Indirect)
        return false;
      orientation = OutlineOrientation::Direct;
    } else if (angleDet < -epsSinus) {
      if (orientation == OutlineOrientation::Direct)
        return false;
      orientation = OutlineOrientation::Indirect;
    }
    pa = pb;
    ++a;
  }
  assert(orientation != OutlineOrientation::Unknown);
  return true;
}


std::vector<AL::Math::ConvexPolygon::Vector2> pointsFromJSON(
    const nlohmann::json &j)
{
  assert(j.is_array());
  std::vector<AL::Math::ConvexPolygon::Vector2> ret;
  ret.reserve(j.size());
  for (size_t i=0; i < j.size(); ++i)
  {
    const auto el = j[i];
    if (!el.is_array() || el.size() != 2u)
      throw std::invalid_argument("parsing error");
    ret.push_back(AL::Math::ConvexPolygon::Vector2(j[i].at(0), j[i].at(1)));
  }
  return ret;
}

::ShapeVariant shapeFromJSON(const nlohmann::json &j)
{
  if (j.is_string() && j == "Plane")
  {
    return AL::Math::Plane{};
  }
  else if (j.is_object())
  {
    auto it = j.find("Polygon");
    if (it != j.end() && it->is_array())
      return AL::Math::ConvexPolygon{pointsFromJSON(*it)};

    it = j.find("Union");
    if (it != j.end() && it->is_array())
    {
      auto iit = it->begin();
      AL::Math::ShapesUnion ret;
      while (iit != it->end())
      {
        auto tmp = shapeFromJSON(*(iit++));
        boost::apply_visitor(ret, tmp); // recursion
      }
      return ret;
    }
  }
  throw std::invalid_argument("parsing error");
}

struct IsPointInsideVisitor : public boost::static_visitor<bool>
{
  double x;
  double y;

  IsPointInsideVisitor(double x, double y) : x(x), y(y) {}

  bool operator()(const AL::Math::ConvexPolygon &shape) const
  {
    return AL::Math::isInside(shape, Eigen::Vector2d(x, y));
  }

  bool operator()(const AL::Math::Plane &) const
  {
    return true;
  }

  bool operator()(const AL::Math::ShapesUnion &shape) const
  {
    return std::any_of(shape.shapes().begin(),
                       shape.shapes().end(),
                       [this](const ::ShapeVariant &s) {
                         return boost::apply_visitor(*this, s);
                       });
  }
};

class are_strict_equals : public boost::static_visitor<bool>
{
public:

  template <typename T, typename U>
  bool operator()( const T &, const U & ) const
  {
    return false; // cannot compare different types
  }

  template <typename T>
  bool operator()( const T & lhs, const T & rhs ) const
  {
    return lhs == rhs;
  }
};

}

namespace std {

void to_json(nlohmann::json& j,
             const vector<AL::Math::ConvexPolygon::Vector2> &vectors)
{
  j = nlohmann::json::array();
  for (const auto &v: vectors)
    j.push_back({v[0], v[1]});
}
}

namespace AL {
namespace Math {

static const double epsSinus = 1e-4f;
static const double epsDistance = 1e-6f;

ConvexPolygon::ConvexPolygon(std::vector<Vector2> outline)
  : _outline(::cleanConvexPolygonOutlineDegenerateSides(std::move(outline),
                                                        epsSinus,
                                                        epsDistance)) {}

bool isInside(const ConvexPolygon &shape, const Eigen::Vector2d &point)
{
  return ::isInsideConvexPolygon(shape.outline(), point, epsSinus,
                                 epsDistance);
}


bool ShapesUnion::isWholePlane() const
{
  return (_shapes.size() == 1u && boost::get<const Plane>(_shapes.data()));
}

void ShapesUnion::operator()(const Plane &)
{
  _shapes.assign(1u, Plane{});
}

void ShapesUnion::operator()(const ConvexPolygon &s)
{
  if (isWholePlane())
    return;
  _shapes.push_back(s);
}

void ShapesUnion::operator()(const ShapesUnion &s)
{
  if (isWholePlane())
    return;
  if (s.isWholePlane())
  {
    _shapes.assign(1u, Plane{});
  }
  else
  {
    std::copy(s._shapes.begin(), s._shapes.end(),
              std::back_inserter(_shapes));
  }
}

bool operator==(const ShapesUnion &lhs, const ShapesUnion &rhs)
{
  return lhs.shapes() == rhs.shapes();
}

void to_json(nlohmann::json& j, const Plane &)
{
    j = nlohmann::json("Plane");
}

void to_json(nlohmann::json& j, const ConvexPolygon &shape)
{
  j = nlohmann::json{{"Polygon", nlohmann::json(shape.outline())}};
}

void to_json(nlohmann::json& j, const ShapesUnion &shape);

class ShapesJSONWriter : public boost::static_visitor<>
{
  nlohmann::json& _j;
public:
  ShapesJSONWriter(nlohmann::json& j) : _j(j) {}

  void operator()(const Plane &s) { to_json(_j, s); }

  void operator()(const ConvexPolygon &s) { to_json(_j, s); }

  void operator()(const ShapesUnion &s) { to_json(_j, s); }
};

::ShapeVariant shapeFromJSON(std::istream &is)
{
  nlohmann::json j;
  is >> j;
  return ::shapeFromJSON(j);
}


void to_json(nlohmann::json& j, const ShapesUnion &shape) {
  auto shapes = nlohmann::json::array();
  nlohmann::json sj;
  for (const auto &s: shape.shapes()) {
    ShapesJSONWriter vis{sj};
    boost::apply_visitor(vis, s);
    shapes.push_back(sj);
  }
  j = nlohmann::json{{"Union", std::move(shapes)}};
}

void writeAsJSON(std::ostream &os, const Plane &shape)
{
  nlohmann::json j;
  to_json(j, shape);
  os << j;
}

void writeAsJSON(std::ostream &os, const ConvexPolygon &shape)
{
  nlohmann::json j;
  to_json(j, shape);
  os << j;
}

void writeAsJSON(std::ostream &os, const ShapesUnion &shape)
{
  nlohmann::json j;
  to_json(j, shape);
  os << j;
}

void writeAsJSON(std::ostream &os, const ::ShapeVariant &shape)
{
  nlohmann::json j;
  ShapesJSONWriter vis{j};
  boost::apply_visitor(vis, shape);
  os << j;
}

class ShapePrivate
{
public:
  ::ShapeVariant shape;
  ShapePrivate(::ShapeVariant &&shape) : shape(shape) {}
};

bool Shape::isInside(double x, double y) const
{
  if (!_p)
    return false;
  return boost::apply_visitor(::IsPointInsideVisitor(x, y), _p->shape);
}

Shape Shape::parseFromJSON(std::istream &is)
{
  Shape ret;
  ret._p = std::make_shared<ShapePrivate>(shapeFromJSON(is));
  return ret;
}

void writeAsJSON(std::ostream &os, const Shape &shape)
{
  if (!shape._p)
    return;
  nlohmann::json j;
  ShapesJSONWriter vis{j};
  boost::apply_visitor(vis, shape._p->shape);
  os << j;
}

bool operator==(const Shape &lhs, const Shape &rhs)
{
  if (!lhs._p)
    return !rhs._p;
  if (!rhs._p)
    return false;
  return boost::apply_visitor(are_strict_equals(),
                              lhs._p->shape,
                              rhs._p->shape);
}

}
}

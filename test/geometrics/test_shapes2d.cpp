#include <memory>
#include <gtest/gtest.h>
#include <almath/geometrics/shapes2d.h>
#include <almath/geometrics/shapes3d.h> // For Plane
#include <almath/geometrics/shape.h>
#include <sstream>
#include <nlohmann/json.hpp>

using namespace AL;
using namespace Math;

TEST(nlohmann_json, number_integer){
  nlohmann::json j;
  std::istringstream("5") >> j;
  EXPECT_TRUE(j.is_number());
  EXPECT_EQ(5, j);
  EXPECT_EQ(5., j);
  EXPECT_NE("5", j);
}

TEST(nlohmann_json, numeric_fp_integer){
  nlohmann::json j;
  std::istringstream("5.0") >> j;
  EXPECT_TRUE(j.is_number());
  EXPECT_EQ(5, j);
  EXPECT_EQ(5., j);
  EXPECT_NE("5.0", j);
}

TEST(nlohmann_json, numeric_fp_integer_ending_dot){
  nlohmann::json j;
  EXPECT_ANY_THROW(std::istringstream("5.") >> j);
}

TEST(nlohmann_json, numeric_fp){
  nlohmann::json j;
  std::istringstream("5.5") >> j;
  EXPECT_TRUE(j.is_number());
  EXPECT_NE(5, j);
  EXPECT_EQ(5.5, j);
  EXPECT_NE("5.5", j);
}

TEST(nlohmann_json, string){
  nlohmann::json j;
  std::istringstream("\"toto\"") >> j;
  EXPECT_TRUE(j.is_string());
  EXPECT_EQ("toto", j);
}

TEST(nlohmann_json, array){
  nlohmann::json j;
  std::istringstream("[-0.5, 2]") >> j;
  EXPECT_TRUE(j.is_array());
  EXPECT_EQ(2u, j.size());
  EXPECT_EQ(-0.5, j[0]);
  EXPECT_EQ(2, j[1]);
}

TEST(nlohmann_json, object){
  nlohmann::json j;
  std::istringstream("{\"polygon\": [[-0.5, 1], [-0.5, 0], [0.5, 0]]}") >> j;
  EXPECT_TRUE(j.is_object());
  EXPECT_NE(j.end(), j.find("polygon"));
  EXPECT_EQ(j.end(), j.find("circle"));
  EXPECT_TRUE(j["polygon"].is_array());
}

ConvexPolygon make_cp0() {
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  outline.emplace_back(0., 1.);
  return ConvexPolygon{outline};
}
const auto cp0 = make_cp0();
const auto cp0_json = "{\"Polygon\":[[0.0,0.0],[1.0,0.0],[0.0,1.0]]}";

ConvexPolygon make_cp1() {
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(-1., 0.);
  outline.emplace_back(0., -1.);
  return ConvexPolygon{outline};
}
const auto cp1 = make_cp1();
const auto cp1_json = "{\"Polygon\":[[0.0,0.0],[-1.0,0.0],[0.0,-1.0]]}";

ConvexPolygon make_cp2() {
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  outline.emplace_back(1., 1.);
  outline.emplace_back(0., 1.);
  return ConvexPolygon{outline};
}
const auto cp2 = make_cp2();


ShapesUnion make_ucp0() {
  ShapesUnion ret;
  ret(cp0);
  return ret;
}

const auto ucp0 = make_ucp0();
const auto ucp0_json =
    "{\"Union\":[{\"Polygon\":[[0.0,0.0],[1.0,0.0],[0.0,1.0]]}]}";

ShapesUnion make_ucp0cp1() {
  ShapesUnion ret = make_ucp0();
  ret(cp1);
  return ret;
}
const auto ucp0cp1 = make_ucp0cp1();
const auto ucp0cp1_json =
    "{\"Union\":[{\"Polygon\":[[0.0,0.0],[1.0,0.0],[0.0,1.0]]},"
                "{\"Polygon\":[[0.0,0.0],[-1.0,0.0],[0.0,-1.0]]}]}";

TEST(equality, basic)
{
  EXPECT_EQ(Plane{}, Plane{});

  EXPECT_TRUE(cp0 == cp0);
  EXPECT_TRUE(cp0 == make_cp0());
  EXPECT_FALSE(cp0 == cp1);

  ShapesUnion u;
  EXPECT_TRUE(u == u);
  EXPECT_TRUE(u == ShapesUnion{});
  EXPECT_FALSE(u == ucp0cp1);
  EXPECT_TRUE(ucp0cp1 == ucp0cp1);
  EXPECT_TRUE(ucp0cp1 == make_ucp0cp1());
}


TEST(equality, equivalent_polygons)
{
  std::vector<ConvexPolygon::Vector2> outline0;
  outline0.emplace_back(0., 0.);
  outline0.emplace_back(1., 0.);
  outline0.emplace_back(0., 1.);
  ConvexPolygon cp0bis{outline0};
  std::vector<ConvexPolygon::Vector2> outline0b;
  outline0.emplace_back(1., 0.);
  outline0.emplace_back(0., 1.);
  outline0.emplace_back(0., 0.);
  ConvexPolygon cp0b{outline0b};
  // those two shapes are equivalent but reported as not equal
  EXPECT_FALSE(cp0 == cp0b);
}

TEST(equality, equivalent_unions)
{
  ShapesUnion ucp1cp0;
  ucp1cp0(cp1);
  ucp1cp0(cp0);
  // those two shapes are equivalent but reported as not equal
  EXPECT_FALSE(ucp0cp1 == ucp1cp0);
}

namespace {
using ShapeVariant = boost::variant<AL::Math::Plane,
                                    AL::Math::ConvexPolygon,
                                    AL::Math::ShapesUnion>;
}
struct UnionOfTwoShapesVisitor
    : public boost::static_visitor<::ShapeVariant>
{
  template <typename S0, typename S1>
  ::ShapeVariant operator()(S0 lhs, S1 rhs) const
  {
    ShapesUnion ret;
    ret(std::move(lhs));
    ret(std::move(rhs));
    return ret;
  }
};

::ShapeVariant unionOf(::ShapeVariant lhs, ::ShapeVariant rhs) {
  return boost::apply_visitor(UnionOfTwoShapesVisitor{}, lhs, rhs);
}

TEST(unionOf, plane)
{
  {
    auto res = boost::get<ShapesUnion>(unionOf(Plane{}, Plane{}));
    ASSERT_EQ(1u, res.shapes().size());
    EXPECT_EQ(Plane{}, boost::get<Plane>(res.shapes()[0]));
  }
  {
    auto res = boost::get<ShapesUnion>(unionOf(Plane{}, cp0));
    ASSERT_EQ(1u, res.shapes().size());
    EXPECT_EQ(Plane{}, boost::get<Plane>(res.shapes()[0]));
  }
  {
    auto res = boost::get<ShapesUnion>(unionOf(cp0, Plane{}));
    ASSERT_EQ(1u, res.shapes().size());
    EXPECT_EQ(Plane{}, boost::get<Plane>(res.shapes()[0]));
  }
  {
    ShapesUnion u0;
    u0(cp0);
    auto res = boost::get<ShapesUnion>(unionOf(u0, Plane{}));
    ASSERT_EQ(1u, res.shapes().size());
    EXPECT_EQ(Plane{}, boost::get<Plane>(res.shapes()[0]));
  }
}

TEST(unionOf, simple)
{
  {
    auto res = boost::get<ShapesUnion>(unionOf(cp0, cp2));
    ASSERT_EQ(2u, res.shapes().size());
    EXPECT_EQ(cp0, boost::get<ConvexPolygon>(res.shapes()[0]));
    EXPECT_EQ(cp2, boost::get<ConvexPolygon>(res.shapes()[1]));
  }
  {
    ShapesUnion u0;
    u0(cp0);
    auto res = boost::get<ShapesUnion>(unionOf(u0, cp2));
    ASSERT_EQ(2u, res.shapes().size());
    EXPECT_EQ(cp0, boost::get<ConvexPolygon>(res.shapes()[0]));
    EXPECT_EQ(cp2, boost::get<ConvexPolygon>(res.shapes()[1]));
  }
}


TEST(ConvexPolygon, basic)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  outline.emplace_back(0., 1.);
  ConvexPolygon shape{outline};

  EXPECT_EQ(outline, shape.outline());

  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0.2, 0.2)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(1., 1.)));
  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0., 0.)));
}

TEST(ConvexPolygon, basic_midpoint)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  outline.emplace_back(.5, .5);
  outline.emplace_back(0., 1.);

  std::vector<ConvexPolygon::Vector2> expected;
  expected.emplace_back(0., 0.);
  expected.emplace_back(1., 0.);
  expected.emplace_back(0., 1.);
  EXPECT_EQ(expected, ConvexPolygon{outline}.outline());
}

TEST(ConvexPolygon, basic_loop)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  outline.emplace_back(0., 1.);
  outline.emplace_back(0., 0.);

  std::vector<ConvexPolygon::Vector2> expected;
  expected.emplace_back(0., 0.);
  expected.emplace_back(1., 0.);
  expected.emplace_back(0., 1.);
  EXPECT_EQ(expected, ConvexPolygon{outline}.outline());
}


TEST(ConvexPolygon, empty)
{
  std::vector<ConvexPolygon::Vector2> outline;
  ConvexPolygon shape{outline};

  EXPECT_TRUE(shape.outline().empty());

  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(0., 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(1., 1.)));
}

TEST(ConvexPolygon, all_zero)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1e-12, 0.);
  outline.emplace_back(0., 1e-12);
  ConvexPolygon shape{outline};

  std::vector<ConvexPolygon::Vector2> expected;
  expected.emplace_back(0., 0.);
  EXPECT_EQ(expected, shape.outline());

  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0., 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(1., 1.)));
}

TEST(ConvexPolygon, all_aligned)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  outline.emplace_back(2., 0.);
  ConvexPolygon shape{outline};

  std::vector<ConvexPolygon::Vector2> expected;
  expected.emplace_back(0., 0.);
  expected.emplace_back(2., 0.);
  EXPECT_EQ(expected, shape.outline());

  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0.5, 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(0.5, 1.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(3., 1.)));
}

TEST(ConvexPolygon, all_aligned_out_of_order)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(2., 0.);
  outline.emplace_back(1., 0.);
  ConvexPolygon shape{outline};

  std::vector<ConvexPolygon::Vector2> expected;
  expected.emplace_back(0., 0.);
  expected.emplace_back(2., 0.);
  EXPECT_EQ(expected, shape.outline());

  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0.5, 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(0.5, 1.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(3., 1.)));
}

TEST(ConvexPolygon, all_aligned_out_of_order_again)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(1., 0.);
  outline.emplace_back(0., 0.);
  outline.emplace_back(2., 0.);
  ConvexPolygon shape{outline};

  std::vector<ConvexPolygon::Vector2> expected;
  expected.emplace_back(0., 0.);
  expected.emplace_back(2., 0.);
  EXPECT_EQ(expected, shape.outline());

  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0.5, 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(0.5, 1.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(3., 1.)));
}

TEST(ConvexPolygon, non_convex)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  outline.emplace_back(0., 1.);
  outline.emplace_back(1., 1.);
  EXPECT_ANY_THROW(ConvexPolygon{outline}.outline());
}

TEST(ConvexPolygon, no_point)
{
  std::vector<ConvexPolygon::Vector2> outline;
  ConvexPolygon shape{outline};

  EXPECT_EQ(outline, shape.outline());

  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(0., 0.)));
}

TEST(ConvexPolygon, single_point)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  ConvexPolygon shape{outline};

  EXPECT_EQ(outline, shape.outline());

  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0., 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(1., 1.)));
}

TEST(ConvexPolygon, two_points)
{
  std::vector<ConvexPolygon::Vector2> outline;
  outline.emplace_back(0., 0.);
  outline.emplace_back(1., 0.);
  ConvexPolygon shape{outline};

  EXPECT_EQ(outline, shape.outline());

  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0., 0.)));
  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(0.5, 0.)));
  EXPECT_TRUE(isInside(shape, Eigen::Vector2d(1., 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(-1., 0.)));
  EXPECT_FALSE(isInside(shape, Eigen::Vector2d(0.5, 0.5)));
}

TEST(shapeFromJSON, plane)
{
  std::stringstream ss("\"Plane\"");
  EXPECT_EQ(Plane{}, boost::get<Plane>(shapeFromJSON(ss)));
}

TEST(shapeFromJSON, convex_polygon)
{
  std::stringstream ss("{\"Polygon\":[[0, 0.0],[1,0] , [0,1]]}");
  EXPECT_EQ(cp0, boost::get<ConvexPolygon>(shapeFromJSON(ss)));
}

TEST(shapeFromJSON, convex_polygon_wrong_outline)
{
  std::stringstream ss("{\"Polygon\":[[0, 0.0],[1,0,2] , [0,1]]}");
  EXPECT_ANY_THROW(shapeFromJSON(ss));
}

TEST(shapeFromJSON, convex_polygon_empty_outline)
{
  std::stringstream ss("{\"Polygon\":[]}");
  EXPECT_EQ(ConvexPolygon{}, boost::get<ConvexPolygon>(shapeFromJSON(ss)));
}

TEST(shapeFromJSON, nonconvex_polygon)
{
  std::stringstream ss("{\"Polygon\":[[0, 0],[1,0],[0,1],[1,1]]}");
  EXPECT_ANY_THROW(shapeFromJSON(ss));

}

TEST(shapeFromJSON, shapes_union)
{
  std::stringstream ss(ucp0_json);
  auto res = boost::get<ShapesUnion>(shapeFromJSON(ss));
  EXPECT_EQ(1u, res.shapes().size());
  EXPECT_EQ(cp0, boost::get<ConvexPolygon>(res.shapes().at(0)));
}

TEST(shapeFromJSON, shapes_union_single)
{
  std::stringstream ss(ucp0_json);
  auto res = boost::get<ShapesUnion>(shapeFromJSON(ss));
  EXPECT_EQ(1u, res.shapes().size());
  EXPECT_EQ(cp0, boost::get<ConvexPolygon>(res.shapes().at(0)));
}

TEST(shapeFromJSON, shapes_union_empty)
{
  std::stringstream ss("{\"Union\":[]}");
  EXPECT_EQ(ShapesUnion{}, boost::get<ShapesUnion>(shapeFromJSON(ss)));
}

TEST(writeAsJSON, plane)
{
  std::stringstream ss;
  writeAsJSON(ss, Plane{});
  EXPECT_EQ("\"Plane\"", ss.str());
}

TEST(writeAsJSON, polygon)
{
  std::stringstream ss;
  writeAsJSON(ss, cp0);
  EXPECT_EQ(cp0_json, ss.str());
}


TEST(writeAsJSON, polygon_empty)
{
  std::stringstream ss;
  writeAsJSON(ss, ConvexPolygon{});
  EXPECT_EQ("{\"Polygon\":[]}", ss.str());
}

TEST(writeAsJSON, shapes_union)
{
  std::stringstream ss;
  writeAsJSON(ss, ucp0cp1);
  EXPECT_EQ(ucp0cp1_json, ss.str());
}

TEST(writeAsJSON, shapes_union_single)
{
  std::stringstream ss;
  writeAsJSON(ss, ucp0);
  EXPECT_EQ(ucp0_json, ss.str());
}

TEST(writeAsJSON, shapes_union_empty)
{
  std::stringstream ss;
  writeAsJSON(ss, ShapesUnion{});
  EXPECT_EQ("{\"Union\":[]}", ss.str());
}

TEST(shape, from_to_json_ucp0cp1)
{
  std::stringstream is(ucp0cp1_json);
  auto shape = Shape::parseFromJSON(is);

  std::stringstream os;
  writeAsJSON(os, shape);
  EXPECT_EQ(ucp0cp1_json, os.str());
}

TEST(shape, from_to_json_ucp0)
{
  std::stringstream is(ucp0_json);
  auto shape = Shape::parseFromJSON(is);

  std::stringstream os;
  writeAsJSON(os, shape);
  EXPECT_EQ(ucp0_json, os.str());
}

TEST(shape, from_to_json_plane)
{
  std::stringstream is("\"Plane\"");
  auto shape = Shape::parseFromJSON(is);

  std::stringstream os;
  writeAsJSON(os, shape);
  EXPECT_EQ("\"Plane\"", os.str());
}

TEST(shape, isInside)
{
  EXPECT_FALSE(Shape{}.isInside(0., 0.));

  std::stringstream is(ucp0cp1_json);
  auto shape = Shape::parseFromJSON(is);

  EXPECT_TRUE(shape.isInside(0., 0.));
  EXPECT_TRUE(shape.isInside(0.2, 0.2));
  EXPECT_TRUE(shape.isInside(-0.2, -0.2));
  EXPECT_FALSE(shape.isInside(0.2, -0.2));
  EXPECT_FALSE(shape.isInside(-0.2, 0.2));
}

TEST(shape, equality)
{
  std::stringstream is0("\"Plane\"");
  auto s0 = Shape::parseFromJSON(is0);
  is0.seekg(0);
  auto s0b = Shape::parseFromJSON(is0);
  EXPECT_TRUE(s0 == s0);
  EXPECT_TRUE(s0 == s0b);
  EXPECT_TRUE(s0b == s0);

  auto s1 = Shape{};
  EXPECT_FALSE(s0 == s1);
  EXPECT_FALSE(s1 == s0);
  EXPECT_TRUE(s1 == s1);

  s0b = s1;
  EXPECT_TRUE(s0b == s1);
  EXPECT_TRUE(s1 == s0b);
  EXPECT_FALSE(s0b == s0);
  EXPECT_FALSE(s0 == s0b);
}

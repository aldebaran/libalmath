/**
* Copyright (c) Aldebaran 2016 All Rights Reserved
*/

#include <gtest/gtest.h>
#include <almath/scenegraph/qianim.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/range/adaptor/map.hpp>
#include <iterator>

using namespace AL::qianim;

void print(const ptree &pt) {
  using namespace boost::property_tree;
  xml_parser::write_xml(std::cout, pt,
                        xml_writer_settings<std::string>(' ', 2));
  std::cout << std::endl;
}

TEST(ALMathQiAnimTest, Tangent_abscissa)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::Tangent::get_abscissa<double>(pt));

  EXPECT_NO_THROW(V2::Tangent::put_abscissa<double>(pt, -1.1));
  EXPECT_EQ(-1.1, V2::Tangent::get_abscissa<double>(pt));

  // just test each method once with float to ensure it compiles
  EXPECT_NO_THROW(V2::Tangent::put_abscissa<float>(pt, -1.1f));
  EXPECT_EQ(-1.1f, V2::Tangent::get_abscissa<float>(pt));
}

TEST(ALMathQiAnimTest, Tangent_ordinate)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::Tangent::get_ordinate<double>(pt));

  EXPECT_NO_THROW(V2::Tangent::put_ordinate<double>(pt, -1.1));
  EXPECT_EQ(-1.1, V2::Tangent::get_ordinate<double>(pt));
}

TEST(ALMathQiAnimTest, Tangent_side) {
  ptree pt;
  EXPECT_ANY_THROW(V2::Tangent::get_side(pt));

  EXPECT_NO_THROW(V2::Tangent::put_side(pt, Side::left));
  EXPECT_EQ(Side::left, V2::Tangent::get_side(pt));

  EXPECT_NO_THROW(V2::Tangent::put_side(pt, Side::right));
  EXPECT_EQ(Side::right, V2::Tangent::get_side(pt));

  pt.put("<xmlattr>.side", "left ");
  EXPECT_ANY_THROW(V2::Tangent::get_side(pt));
}

TEST(ALMathQiAnimTest, Key_frame)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::Key::get_frame(pt));

  EXPECT_NO_THROW(V2::Key::put_frame(pt, 0));
  EXPECT_EQ(0, V2::Key::get_frame(pt));

  EXPECT_NO_THROW(V2::Key::put_frame(pt, 1));
  EXPECT_EQ(1, V2::Key::get_frame(pt));

  EXPECT_ANY_THROW(V2::Key::put_frame(pt, -1));
  pt.put("<xmlattr>.frame", -1);
  EXPECT_ANY_THROW(V2::Key::get_frame(pt));
}

ptree &add_key(ptree &curve, int frame, float value) {
  ptree &ret = curve.add_child("Key", ptree());
  V2::Key::put_frame(ret, frame);
  V2::Key::put_value(ret, value);
  return ret;
}

ptree &add_tangent(ptree &key, const std::string &side) {
  ptree &ret = key.add_child("Tangent", ptree());
  ret.put("<xmlattr>.side", side);
  return ret;
}

ptree &add_tangent(ptree &key, const std::string &side,
                   float abscissa, float ordinate) {
  ptree &ret = add_tangent(key, side);
  V2::Tangent::put_abscissa(ret, abscissa);
  V2::Tangent::put_ordinate(ret, ordinate);
  return ret;
}

TEST(ALMathQiAnimTest, Key_tangent_none)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::Key::get_tangent(pt, Side::left));
  EXPECT_ANY_THROW(V2::Key::get_tangent(pt, Side::right));
}

TEST(ALMathQiAnimTest, Key_tangent_left)
{
  ptree pt;
  add_tangent(pt, "left");
  EXPECT_NO_THROW(V2::Key::get_tangent(pt, Side::left));
  EXPECT_ANY_THROW(V2::Key::get_tangent(pt, Side::right));
}

TEST(ALMathQiAnimTest, Key_tangent_right)
{
  ptree pt;
  add_tangent(pt, "right");
  EXPECT_ANY_THROW(V2::Key::get_tangent(pt, Side::left));
  EXPECT_NO_THROW(V2::Key::get_tangent(pt, Side::right));

  // same with _optional API
  EXPECT_FALSE(V2::Key::get_tangent_optional(pt, Side::left));
  EXPECT_TRUE(V2::Key::get_tangent_optional(pt, Side::right));
}

TEST(ALMathQiAnimTest, Key_tangent_both)
{
  ptree pt;
  add_tangent(pt, "left");
  add_tangent(pt, "right");
  EXPECT_NO_THROW(V2::Key::get_tangent(pt, Side::left));
  EXPECT_NO_THROW(V2::Key::get_tangent(pt, Side::right));
}

TEST(ALMathQiAnimTest, Key_tangent_require)
{
  ptree pt;
  ptree &left = add_tangent(pt, "left");
  // the existing node is returned
  EXPECT_EQ(&left, &V2::Key::require_tangent(pt, Side::left));
  EXPECT_NO_THROW(V2::Key::get_tangent(pt, Side::left));
  // a new node is added
  EXPECT_NE(&left, &V2::Key::require_tangent(pt, Side::right));
  EXPECT_NO_THROW(V2::Key::get_tangent(pt, Side::right));
}

TEST(ALMathQiAnimTest, Key_tangent_put)
{
  ptree pt;
  ptree &orig_left = add_tangent(pt, "left");

  ptree &left = V2::Key::put_tangent(pt, Side::left, -1.1, 1.);
  EXPECT_EQ(&orig_left, &left); // the existing node was returned
  EXPECT_EQ(Side::left, V2::Tangent::get_side(left));
  EXPECT_EQ(-1.1, V2::Tangent::get_abscissa<double>(left));
  EXPECT_EQ(1., V2::Tangent::get_ordinate<double>(left));
}

TEST(ALMathQiAnimTest, Key_tangent_erase) {
  ptree pt;

  EXPECT_EQ(0u, V2::Key::erase_tangent(pt, Side::left));
  EXPECT_EQ(0u, V2::Key::erase_tangent(pt, Side::right));

  add_tangent(pt, "left");
  add_tangent(pt, "left");
  add_tangent(pt, "right");

  EXPECT_EQ(2u, V2::Key::erase_tangent(pt, Side::left));

  EXPECT_FALSE(V2::Key::get_tangent_optional(pt, Side::left));
  EXPECT_TRUE(V2::Key::get_tangent_optional(pt, Side::right));

  EXPECT_EQ(1u, V2::Key::erase_tangent(pt, Side::right));

  EXPECT_FALSE(V2::Key::get_tangent_optional(pt, Side::right));
}

TEST(ALMathQiAnimTest, Key_value)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::Key::get_value<double>(pt));

  EXPECT_NO_THROW(V2::Key::put_value<double>(pt, 0.));
  EXPECT_EQ(0., V2::Key::get_value<double>(pt));

  EXPECT_NO_THROW(V2::Key::put_value<double>(pt, -1.1));
  EXPECT_EQ(-1.1, V2::Key::get_value<double>(pt));

  // just test each method once with float to ensure it compiles
  EXPECT_NO_THROW(V2::Key::put_value<float>(pt, -1.1f));
  EXPECT_EQ(-1.1f, V2::Key::get_value<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_actuator)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_actuator(pt));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_actuator(pt, "poney"));
  EXPECT_EQ("poney", V2::ActuatorCurve::get_actuator(pt));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_actuator(pt, ""));
  EXPECT_EQ("", V2::ActuatorCurve::get_actuator(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_fps)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_fps(pt));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_fps(pt, 1));
  EXPECT_EQ(1, V2::ActuatorCurve::get_fps(pt));

  EXPECT_ANY_THROW(V2::ActuatorCurve::put_fps(pt, -1));
  pt.put("<xmlattr>.fps", -1);
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_fps(pt));

  EXPECT_ANY_THROW(V2::ActuatorCurve::put_fps(pt, 0));
  pt.put("<xmlattr>.fps", 0);
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_fps(pt));

  // check it works for Labels elements too
  EXPECT_NO_THROW(V2::Labels::put_fps(pt, 2));
  EXPECT_EQ(2, V2::Labels::get_fps(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_mute)
{
  ptree pt;
  EXPECT_FALSE(V2::ActuatorCurve::get_mute(pt));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_mute(pt, false));
  EXPECT_FALSE(V2::ActuatorCurve::get_mute(pt));
  EXPECT_EQ("false", pt.get<std::string>("<xmlattr>.mute"));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_mute(pt, true));
  EXPECT_TRUE(V2::ActuatorCurve::get_mute(pt));
  EXPECT_EQ("true", pt.get<std::string>("<xmlattr>.mute"));
}

TEST(ALMathQiAnimTest, ActuatorCurve_unit)
{
  ptree pt;
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_unit(pt));

  pt.put("<xmlattr>.unit", "");
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_unit(pt));

  pt.put("<xmlattr>.unit", "invalid stuff");
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_unit(pt));

  pt.put("<xmlattr>.unit", "unknown");
  EXPECT_ANY_THROW(V2::ActuatorCurve::get_unit(pt));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_unit(pt, Unit::radian));
  EXPECT_EQ(Unit::radian, V2::ActuatorCurve::get_unit(pt));
  EXPECT_EQ("radian", pt.get<std::string>("<xmlattr>.unit"));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_unit(pt, Unit::degree));
  EXPECT_EQ(Unit::degree, V2::ActuatorCurve::get_unit(pt));
  EXPECT_EQ("degree", pt.get<std::string>("<xmlattr>.unit"));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_unit(pt, Unit::meter));
  EXPECT_EQ(Unit::meter, V2::ActuatorCurve::get_unit(pt));
  EXPECT_EQ("meter", pt.get<std::string>("<xmlattr>.unit"));

  EXPECT_NO_THROW(V2::ActuatorCurve::put_unit(pt, Unit::dimensionless));
  EXPECT_EQ(Unit::dimensionless, V2::ActuatorCurve::get_unit(pt));
  EXPECT_EQ("dimensionless", pt.get<std::string>("<xmlattr>.unit"));
}

TEST(ALMathQiAnimTest, ActuatorCurve_key)
{
  ptree pt;
  EXPECT_FALSE(V2::ActuatorCurve::get_key_optional(pt, 10));
  ptree &k10 = V2::ActuatorCurve::require_key(pt, 10);

  // the added key can be found back
  const ptree &cpt = pt;
  auto k10opt = V2::ActuatorCurve::get_key_optional(cpt, 10);
  ASSERT_TRUE(k10opt);
  EXPECT_EQ(&k10, &*k10opt);
  EXPECT_EQ(10, V2::Key::get_frame(k10));

  // requiring a key at this frame again returns the same node
  ptree &k10bis = V2::ActuatorCurve::require_key(pt, 10);
  EXPECT_EQ(k10, k10bis);

}

TEST(ALMathQiAnimTest, ActuatorCurve_key_sorted)
{
  ptree pt;
  ptree &a = pt.put_child("a", ptree());
  ptree &k20 = pt.put_child("Key", ptree());
  k20.put("<xmlattr>.frame", 20);
  ptree &b = pt.put_child("b", ptree());

  print(pt);
  // insert keys out of order
  ptree &k10 = V2::ActuatorCurve::require_key(pt, 10);
  print(pt);
  ptree &k40 = V2::ActuatorCurve::require_key(pt, 40);
  print(pt);
  // no-op
  EXPECT_EQ(&k20, &V2::ActuatorCurve::require_key(pt, 20));
  ptree &k30 = V2::ActuatorCurve::require_key(pt, 30);
  print(pt);

  // then iterate and check they are ordered
  auto cit = pt.begin();
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&a, &cit->second);
  ++cit;

  auto keys = V2::ActuatorCurve::get_keys(pt);
  auto kit = keys.begin();

  ASSERT_NE(kit, keys.end());
  EXPECT_EQ(10, V2::Key::get_frame(*kit));
  EXPECT_EQ(&k10, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&k10, &cit->second);
  ++cit;

  ASSERT_NE(kit, keys.end());
  EXPECT_EQ(&k20, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&k20, &cit->second);
  ++cit;

  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&b, &cit->second);
  ++cit;

  ASSERT_NE(kit, keys.end());
  EXPECT_EQ(&k30, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&k30, &cit->second);
  ++cit;

  ASSERT_NE(kit, keys.end());
  EXPECT_EQ(&k40, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&k40, &cit->second);
  ++cit;

  ASSERT_EQ(kit, keys.end());
  ASSERT_EQ(cit, pt.end());
}

TEST(ALMathQiAnimTest, ActuatorCurve_check_curve_nonincreasing)
{
  ptree pt;
  add_key(pt, 10, 0.f);
  add_key(pt, 10, 1.f);
  EXPECT_ANY_THROW(V2::ActuatorCurve::check_cubic_bezier<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_check_curve_nonincreasing_bis)
{
  ptree pt;
  add_key(pt, 10, 0.f);
  add_key(pt, 9, 1.f);
  EXPECT_ANY_THROW(V2::ActuatorCurve::check_cubic_bezier<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_check_curve_missing_tangents)
{
  ptree pt;
  add_key(pt, 0, 0.f);
  add_key(pt, 10, 1.f);
  EXPECT_ANY_THROW(V2::ActuatorCurve::check_cubic_bezier<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_check_curve_wrong_p1_tangent)
{
  ptree pt;
  add_tangent(add_key(pt, 0, 0.f), "right", -1.f, 0.f);
  add_tangent(add_key(pt, 10, 1.f), "left", 0.f, 0.f);
  EXPECT_ANY_THROW(V2::ActuatorCurve::check_cubic_bezier<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_check_curve_wrong_p1_tangent_bis)
{
  ptree pt;
  add_tangent(add_key(pt, 0, 0.f), "right", 11.f, 0.f);
  add_tangent(add_key(pt, 10, 1.f), "left", 0.f, 0.f);
  EXPECT_ANY_THROW(V2::ActuatorCurve::check_cubic_bezier<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_check_curve_wrong_p2_tangent)
{
  ptree pt;
  add_tangent(add_key(pt, 0, 0.f), "right", 0.f, 0.f);
  add_tangent(add_key(pt, 10, 1.f), "left", 1.f, 0.f);
  EXPECT_ANY_THROW(V2::ActuatorCurve::check_cubic_bezier<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_check_curve_wrong_p2_tangent_bis)
{
  ptree pt;
  add_tangent(add_key(pt, 0, 0.f), "right", 0.f, 0.f);
  add_tangent(add_key(pt, 10, 1.f), "left", -11.f, 0.f);
  EXPECT_ANY_THROW(V2::ActuatorCurve::check_cubic_bezier<float>(pt));
}

TEST(ALMathQiAnimTest, ActuatorCurve_get_keys)
{
  ptree pt;
  V2::ActuatorCurve::require_key(pt, 1);
  V2::ActuatorCurve::require_key(pt, 2);

  // check that range-based-for does compile
  const ptree & cpt = pt;
  int i = 0;
  for (const ptree &key: V2::ActuatorCurve::get_keys(cpt)) {
    ++i;
    EXPECT_EQ(i, V2::Key::get_frame(key));
  }

  i = 0;
  for (ptree &key: V2::ActuatorCurve::get_keys(pt)) {
    ++i;
    V2::Key::put_frame(key, 10 + i);
  }
}


struct IntervalStore {

  struct CubicBezierData {
    int p0_frame;
    float p0_value;
    float p1_dframe;
    float p1_dvalue;
    float p2_dframe;
    float p2_dvalue;
    int p3_frame;
    float p3_value;
  };
  std::vector<CubicBezierData> data;

  void operator()(const ptree &p0_key, const ptree &p3_key) {
    V2::Key::apply_cubic_bezier<float>(p0_key, p3_key, std::ref(*this));
  }

  void operator()(int p0_frame, float p0_value,
                  float p1_dframe, float p1_dvalue,
                  float p2_dframe, float p2_dvalue,
                  int p3_frame, float p3_value) {
    // throw if invalid
    V2::Key::check_cubic_bezier(p0_frame, p0_value,
                                p1_dframe, p1_dvalue,
                                p2_dframe, p2_dvalue,
                                p3_frame, p3_value);
    data.push_back(CubicBezierData{p0_frame, p0_value,
                                   p1_dframe, p1_dvalue,
                                   p2_dframe, p2_dvalue,
                                   p3_frame, p3_value});
  }
};

TEST(ALMathQiAnimTest, ActuatorCurve_adjacent_for_each)
{
  ptree pt;
  add_tangent(add_key(pt, 0, 0.f), "right", 0.f, 0.f);
  auto &middle_key = add_key(pt, 10, 1.f);
  add_tangent(middle_key, "left", 0.f, 0.f);
  add_tangent(middle_key, "right", 10.f, 0.f);
  add_tangent(add_key(pt, 20, 2.f), "left", -10.f, 0.f);
  auto keys = V2::ActuatorCurve::get_keys(pt);

  IntervalStore intervals;
  EXPECT_NO_THROW(intervals = V2::adjacent_for_each(keys.begin(), keys.end(), IntervalStore()));

  ASSERT_EQ(2u, intervals.data.size());
  // check the frames
  EXPECT_EQ(0, intervals.data[0].p0_frame);
  EXPECT_EQ(10, intervals.data[0].p3_frame);
  EXPECT_EQ(intervals.data[0].p3_frame, intervals.data[1].p0_frame);
  EXPECT_EQ(20, intervals.data[1].p3_frame);
  // check the values
  EXPECT_EQ(0.f, intervals.data[0].p0_value);
  EXPECT_EQ(1.f, intervals.data[0].p3_value);
  EXPECT_EQ(intervals.data[0].p3_value, intervals.data[1].p0_value);
  EXPECT_EQ(2.f, intervals.data[1].p3_value);
  // check the tangents
  // the first interval is a linear interpolation
  EXPECT_EQ(0.f, intervals.data[0].p1_dframe);
  EXPECT_EQ(0.f, intervals.data[0].p1_dvalue);
  EXPECT_EQ(0.f, intervals.data[0].p2_dframe);
  EXPECT_EQ(0.f, intervals.data[0].p2_dvalue);
  // the second interval is a S-like cubic Bezier interpolation
  EXPECT_EQ(10.f, intervals.data[1].p1_dframe);
  EXPECT_EQ(0.f, intervals.data[1].p1_dvalue);
  EXPECT_EQ(-10.f, intervals.data[1].p2_dframe);
  EXPECT_EQ(0.f, intervals.data[1].p2_dvalue);
}


TEST(ALMathQiAnimTest, Labels_add_label)
{
  ptree pt;
  ptree &k10 = V2::Labels::add_label(pt, 10, "my label");
  EXPECT_EQ(10, V2::Label::get_frame(k10));
  EXPECT_EQ("my label", V2::Label::get_value(k10));
}

TEST(ALMathQiAnimTest, Labels_label_sorted)
{
  ptree pt;
  ptree &a = pt.put_child("a", ptree());
  ptree &f20 = pt.put_child("Label", ptree("label_a"));
  f20.put("<xmlattr>.frame", 20);
  ptree &b = pt.put_child("b", ptree());

  print(pt);
  // insert labels out of order
  ptree &f10 = V2::Labels::add_label(pt, 10, "label_b");
  print(pt);
  ptree &f40 = V2::Labels::add_label(pt, 40, "label_c");
  print(pt);
  ptree &f10d = V2::Labels::add_label(pt, 10, "label_d");
  print(pt);
  ptree &f30 = V2::Labels::add_label(pt, 30, "label_e");
  print(pt);

  // then iterate and check they are ordered
  auto cit = pt.begin();
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&a, &cit->second);
  ++cit;

  auto labels = V2::Labels::get_labels(pt);
  auto kit = labels.begin();

  ASSERT_NE(kit, labels.end());
  EXPECT_EQ(10, V2::Label::get_frame(*kit));
  EXPECT_EQ(&f10, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&f10, &cit->second);
  ++cit;

  ASSERT_NE(kit, labels.end());
  EXPECT_EQ(10, V2::Label::get_frame(*kit));
  EXPECT_EQ(&f10d, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&f10d, &cit->second);
  ++cit;

  ASSERT_NE(kit, labels.end());
  EXPECT_EQ(&f20, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&f20, &cit->second);
  ++cit;

  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&b, &cit->second);
  ++cit;

  ASSERT_NE(kit, labels.end());
  EXPECT_EQ(30, V2::Label::get_frame(*kit));
  EXPECT_EQ(&f30, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&f30, &cit->second);
  ++cit;

  ASSERT_NE(kit, labels.end());
  EXPECT_EQ(&f40, &*kit);
  ++kit;
  ASSERT_NE(cit, pt.end());
  EXPECT_EQ(&f40, &cit->second);
  ++cit;

  ASSERT_EQ(kit, labels.end());
  ASSERT_EQ(cit, pt.end());
}

TEST(ALMathQiAnimTest, Animation_check_version)
{
  EXPECT_ANY_THROW(V2::Animation::check_version(ptree{}));

  ptree wrong;
  wrong.put("<xmlattr>.typeVersion", "1.0");
  EXPECT_ANY_THROW(V2::Animation::check_version(wrong));

  ptree good;
  good.put("<xmlattr>.typeVersion", "2.0");
  EXPECT_NO_THROW(V2::Animation::check_version(good));
}

TEST(ALMathQiAnimTest, get_require_animation)
{
  ptree wrong;
  wrong.put("Animation.<xmlattr>.typeVersion", "1.0");
  EXPECT_ANY_THROW(V2::get_animation(wrong));
  EXPECT_ANY_THROW(V2::require_animation(wrong));

  ptree good;
  good.put("Animation.<xmlattr>.typeVersion", "2.0");
  EXPECT_NO_THROW(V2::get_animation(good));
  EXPECT_NO_THROW(V2::require_animation(good));

  ptree root;

  EXPECT_ANY_THROW(V2::get_animation(root));

  ptree &animation = V2::require_animation(root);
  EXPECT_NO_THROW(V2::Animation::check_version(animation));
  EXPECT_NO_THROW(V2::get_animation(root));
}

TEST(ALMathQiAnimTest, Animation_get_actuatorcurves)
{
  ptree pt;
  V2::Animation::require_actuatorcurve(pt, "a");
  V2::Animation::require_actuatorcurve(pt, "b");

  std::string names[] = {"a", "b"};

  // check that range-based-for does compile
  const ptree & cpt = pt;
  size_t i =0u;
  for (const ptree &curve: V2::Animation::get_actuatorcurves(cpt)) {
    EXPECT_EQ(names[i], V2::ActuatorCurve::get_actuator(curve));
    ++i;
  }

  i = 0u;
  for (ptree &curve: V2::Animation::get_actuatorcurves(pt)) {
    V2::ActuatorCurve::put_actuator(curve, names[i] + "toto");
    ++i;
  }
}

TEST(ALMathQiAnimTest, Animation_require_actuatorcurve) {
  ptree pt;

  // insert a new element
  ptree &a = V2::Animation::require_actuatorcurve(pt, "a");
  EXPECT_EQ("a", V2::ActuatorCurve::get_actuator(a));

  // insert a new element
  ptree &b = V2::Animation::require_actuatorcurve(pt, "b");
  EXPECT_EQ("b", V2::ActuatorCurve::get_actuator(b));

  // we get the same element back
  EXPECT_EQ(&a, &V2::Animation::require_actuatorcurve(pt, "a"));

  // check the curves are in the expected order
  const auto curves = V2::Animation::get_actuatorcurves(pt);
  auto it = curves.begin();

  ASSERT_NE(it, curves.end());
  EXPECT_EQ(&a, &*it);

  ++it;
  ASSERT_NE(it, curves.end());
  EXPECT_EQ(&b, &*it);

  ++it;
  EXPECT_EQ(it, curves.end());
}

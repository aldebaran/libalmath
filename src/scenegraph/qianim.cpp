/*
 * Copyright (c) 2016 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/scenegraph/qianim.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/range/adaptor/map.hpp>
#include <sstream>

namespace AL {
namespace qianim {
namespace V2 {
namespace Tangent {

Side get_side(const ptree &pt) {
  const auto side = pt.get<std::string>("<xmlattr>.side");
  if (side == "left")
    return Side::left;
  if (side == "right")
    return Side::right;
  throw std::invalid_argument(
      "Tangent side shall be either \"left\" or \"right\"");
}

void put_side(ptree &pt, Side side) {
  pt.put<std::string>("<xmlattr>.side",
                      (side == Side::left) ? "left" : "right");
}

} // namespace Tangent

namespace Key {

bool is_key(const ptree::value_type &val) {
  return val.first == "Key";
}

int get_frame(const ptree &pt) {
  const auto frame = pt.get<int>("<xmlattr>.frame");
  if (frame < 0)
    throw std::invalid_argument("Key frame shall be non-negative");
  return frame;
}

void put_frame(ptree &pt, int frame) {
  if (frame < 0)
    throw std::invalid_argument("Key frame shall be non-negative");
  pt.put<int>("<xmlattr>.frame", frame);
}

ptree::size_type erase_tangent(ptree &pt, Side side) {
  ptree::size_type ret = 0;
  auto it = pt.begin();
  while (it != pt.end()) {
    if ((it->first == "Tangent") &&
        (Tangent::get_side(it->second) == side)) {
      it = pt.erase(it);
      ++ret;
    } else  {
      ++it;
    }
  }
  return ret;
}

template<typename ptree_ref>
optional<ptree_ref> get_tangent_optional_impl(ptree_ref pt, Side side) {
  for(auto &tangent: boost::make_iterator_range(pt.equal_range("Tangent"))) {
    if (side == Tangent::get_side(tangent.second))
      return tangent.second;
  }
  return optional<ptree_ref>();
}

optional<const ptree &> get_tangent_optional(const ptree &pt, Side side) {
  return get_tangent_optional_impl<const ptree &>(pt, side);
}

optional<ptree &> get_tangent_optional(ptree &pt, Side side) {
  return get_tangent_optional_impl<ptree &>(pt, side);
}

// private
ptree &add_tangent(ptree &pt, Side side) {
  assert(!get_tangent_optional(pt, side));
  ptree &ret = pt.add("Tangent", "");
  Tangent::put_side(ret, side);
  return ret;
}

ptree &require_tangent(ptree &pt, Side side) {
  return get_tangent_optional(pt, side).value_or_eval(
      [&pt, side] () -> ptree & { return add_tangent(pt, side); });
}
} // namespace Key

namespace ActuatorCurve {

bool is_actuatorcurve(const ptree::value_type &val) {
  return val.first == "ActuatorCurve";
}

std::string get_actuator(const ptree &pt) {
  return pt.get<std::string>("<xmlattr>.actuator");
}

void put_actuator(ptree &pt, const std::string &name) {
  pt.put("<xmlattr>.actuator", name);
}

// return fps, which is positive
int get_fps(const ptree &pt) {
  const auto ret = pt.get<int>("<xmlattr>.fps");
  if (ret <= 0)
    throw std::invalid_argument("ActuatorCurve fps shall be positive");
  return ret;
}

void put_fps(ptree &pt, int fps) {
  if (fps <= 0)
    throw std::invalid_argument("ActuatorCurve fps shall be positive");
  pt.put("<xmlattr>.fps", fps);
}

bool get_mute(const ptree &pt) {
  return pt.get<bool>("<xmlattr>.mute", false);
}

void put_mute(ptree &pt, bool mute) {
  pt.put("<xmlattr>.mute", mute);
}

Unit get_unit(const ptree &pt) {
  const auto unit = pt.get<std::string>("<xmlattr>.unit");
  if (unit == "dimensionless")
    return Unit::dimensionless;
  if (unit == "radian")
    return Unit::radian;
  if (unit == "degree")
    return Unit::degree;
  if (unit == "meter")
    return Unit::meter;
  throw std::invalid_argument("ActuatorCurve unit is invalid");
}

std::string to_string(Unit unit) {
  switch (unit)
  {
  case Unit::dimensionless:
    return "dimensionless";
  case Unit::radian:
    return "radian";
  case Unit::degree:
    return "degree";
  case Unit::meter:
    return "meter";
  }
  std::abort();
}

void put_unit(ptree &pt, Unit unit) {
  pt.put("<xmlattr>.unit", to_string(unit));
}

// private
bool key_comp(const ptree::value_type &key, int frame) {
  return (key.first != "Key") || (Key::get_frame(key.second) < frame);
}

optional<ptree &> get_key_optional(ptree &pt, int frame) {
  auto it = std::lower_bound(pt.begin(), pt.end(), frame, key_comp);
  if ((it != pt.end()) && (Key::get_frame(it->second) == frame))
    return it->second;
  return optional<ptree &>();
}

optional<const ptree &> get_key_optional(const ptree &pt, int frame) {
  auto it = std::lower_bound(pt.begin(), pt.end(), frame, key_comp);
  if ((it != pt.end()) && (Key::get_frame(it->second) == frame))
    return it->second;
  return optional<const ptree &>();
}

ptree &require_key(ptree &pt, int frame) {
  auto it = std::lower_bound(pt.begin(), pt.end(), frame, key_comp);
  if ((it != pt.end()) && (Key::get_frame(it->second) == frame))
    return it->second;
  ptree &newkey = pt.insert(it, std::make_pair("Key", ptree{}))->second;
  Key::put_frame(newkey, frame);
  return newkey;
}
} // namespace ActuatorCurve

namespace Label {

// private
bool label_comp(int frame, const ptree::value_type &label) {
  return (label.first == "Label") && (frame < Label::get_frame(label.second));
}


bool is_label(const ptree::value_type &val) {
  return val.first == "Label";
}

std::string get_value(const ptree &pt) {
  return pt.get_value<std::string>();
}
} // namespace Label

namespace Labels {

bool is_labels(const ptree::value_type &val) {
  return val.first == "Labels";
}

ptree &add_label(ptree &pt, int frame, const std::string value) {
  auto it = std::upper_bound(pt.begin(), pt.end(), frame, Label::label_comp);
  ptree &newkey = pt.insert(it, std::make_pair("Label", ptree{value}))->second;
  Key::put_frame(newkey, frame);
  return newkey;
}
} // namespace Labels


namespace Animation {

void check_version(const ptree& pt) {
  if (pt.get<std::string>("<xmlattr>.typeVersion") != "2.0")
    throw std::runtime_error("Animation typeVersion is unsupported");
}

void check_all(const ptree& pt) {
  check_version(pt);
  for (const auto &curve: get_actuatorcurves(pt)) {
     ActuatorCurve::get_actuator(curve);
     ActuatorCurve::get_fps(curve);
     ActuatorCurve::get_mute(curve);
     ActuatorCurve::get_unit(curve);
     ActuatorCurve::check_cubic_bezier<double>(curve);
  }
}

ptree &require_actuatorcurve(ptree &pt, const std::string &actuator) {
  auto comp = [&actuator] (ptree::value_type &child) {
    return child.first == "ActuatorCurve" &&
           ActuatorCurve::get_actuator(child.second) == actuator;};
  auto it = std::find_if(pt.begin(), pt.end(), comp);
  if (it != pt.end())
    return it->second;
  ptree &newactuator = pt.insert(it, std::make_pair("ActuatorCurve",
                                                    ptree{}))->second;
  ActuatorCurve::put_actuator(newactuator, actuator);
  return newactuator;
}

ptree &add_labels(ptree &pt) {
  return pt.add_child("Labels", ptree{});
}

} // namespace Animation


ptree &get_animation(ptree &root) {
  ptree &animation = root.get_child("Animation");
  Animation::check_version(animation);
  return animation;
}

const ptree &get_animation(const ptree &root) {
  const ptree &animation = root.get_child("Animation");
  Animation::check_version(animation);
  return animation;
}

ptree &require_animation(ptree &root) {
  auto opt = root.get_child_optional("Animation");
  if (opt) {
    Animation::check_version(*opt);
    return *opt;
  }
  ptree &animation = root.put_child("Animation", ptree{});
  animation.put("<xmlattr>.typeVersion", "2.0");
  return animation;
}

}
}
}

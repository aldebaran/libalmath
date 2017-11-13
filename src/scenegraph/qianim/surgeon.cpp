/*
 * Copyright (c) 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <iostream>
#include <almath/scenegraph/qianim/surgeon.h>
#include <almath/scenegraph/qianim/bezierautotangent.h>
#include <almath/scenegraph/qianim/unitutils.h>
#include <boost/range/algorithm.hpp>

namespace AL {
namespace qianim {

namespace Tangent {

void migrate_v1_to_v2(ptree &tangent, float max_abs_abscissa,
                      Side side) {
  ptree &attr = tangent.get_child("<xmlattr>");
  auto type = attr.get<std::string>("interpType");
  if (type == "constant")
    throw std::runtime_error("unsupported Tangent type \"constant\"");

  if (type == "bezier") {
    auto abscissa = V2::Tangent::get_abscissa<float>(tangent);
    auto ordinate = V2::Tangent::get_ordinate<float>(tangent);
    if ((side == Side::left) && (abscissa > 0.f)) {
      throw std::invalid_argument(
            "abscissa of a left Tangent shall be non-positive");
    } else if ((side == Side::right) && (abscissa < 0.f)) {
      throw std::invalid_argument(
            "abscissa of a right Tangent shall be non-negative");
    }
    assert(max_abs_abscissa >= 1.f);
    if (std::abs(abscissa) > max_abs_abscissa) {
      auto k = std::abs(max_abs_abscissa / abscissa);
      V2::Tangent::put_abscissa(tangent, k * abscissa);
      V2::Tangent::put_ordinate(tangent, k * ordinate);
    }
    attr.put("editor:interpType", "bezier");
  } else if (type == "linear") {
    V2::Tangent::put_abscissa(tangent, 0.f);
    V2::Tangent::put_ordinate(tangent, 0.f);
    attr.put("editor:interpType", "linear");
  } else {
    throw std::invalid_argument("invalid Tangent interpType");
  }
  attr.erase("interpType");
}
}

namespace Key {

void offset_frame(ptree &key, int value) {
  auto &frame = key.get_child("<xmlattr>.frame");
  frame.put_value(frame.get_value<int>() + value);
}

void migrate_smooth_symmetrical_v1_to_v2(ptree &key) {
  auto attr = key.get_child_optional("<xmlattr>");
  if (!attr)
    return;

  auto smooth = attr->get_child_optional("smooth");
  if (smooth) {
    attr->put("editor:smooth", smooth->get_value<bool>());
    attr->erase("smooth");
  }
  auto symmetrical = attr->get_child_optional("symmetrical");
  if (symmetrical) {
    attr->put("editor:symmetrical", symmetrical->get_value<bool>());
    attr->erase("symmetrical");
  }
}

void migrate_tangents_v1_to_v2(optional<const ptree &> prev_key,
                               ptree &key,
                               optional<const ptree &> next_key) {
  int frame = V2::Key::get_frame(key);
  auto value = V2::Key::get_value<float>(key);
  float left_dframe = 0.f;
  float left_dvalue = 0.f;
  if (prev_key) {
    left_dframe = static_cast<float>(frame - V2::Key::get_frame(*prev_key));
    if (left_dframe < 1.f) {
      throw std::invalid_argument(
          "successive Key elements shall have increasing frame attributes");
    }
    left_dvalue = value - V2::Key::get_value<float>(*prev_key);
  }

  float right_dframe = 0.f;
  float right_dvalue = 0.f;
  if (next_key) {
    right_dframe = static_cast<float>(V2::Key::get_frame(*next_key) - frame);
    if (right_dframe < 1.f) {
      throw std::invalid_argument(
          "successive Key elements shall have increasing frame attributes");
    }
    right_dvalue = V2::Key::get_value<float>(*next_key) - value;
  }

  // compute tangents as if interpType was BEZIER_AUTO.
  // We might need the result later.
  const auto tangents = computeBezierAutoTangents<float>(left_dframe, right_dframe,
                                                         left_dvalue, right_dvalue);

  auto left_tangent = V2::Key::get_tangent_optional(key, Side::left);
  if (left_tangent) {
    // There is a tangent, we'll to keep it.
    // If the key is the first one, we don't want to scale its abscissa so
    float max_abs = prev_key ? left_dframe :
                               std::numeric_limits<float>::infinity();
    Tangent::migrate_v1_to_v2(*left_tangent, max_abs, Side::left);
  } else if (prev_key) {
    // there is no tangent, and the key is not the first one, so
    // use the BEZIER_AUTO computation to create a tangent.
    V2::Key::put_tangent(key, Side::left,
                         tangents.first.first, tangents.first.second).put(
        "<xmlattr>.editor:interpType", "bezier_auto");
  }

  auto right_tangent = V2::Key::get_tangent_optional(key, Side::right);
  if (right_tangent) {
    float max_abs = next_key ? right_dframe :
                               std::numeric_limits<float>::infinity();
    Tangent::migrate_v1_to_v2(*right_tangent, max_abs, Side::right);
  } else if (next_key) {
    V2::Key::put_tangent(key, Side::right,
                         tangents.second.first, tangents.second.second).put(
        "<xmlattr>.editor:interpType", "bezier_auto");
  }
}
}

namespace ActuatorCurve {

void convert_unit(ptree &curve, Unit to, Unit from) {
  if (V2::ActuatorCurve::get_unit(curve) != from)
    return;
  auto a = getUnitConversionFactor<float>(to, from);
  V2::ActuatorCurve::put_unit(curve, to);
  for (ptree &key: V2::ActuatorCurve::get_keys(curve)) {
    V2::Key::put_value(key, a * V2::Key::get_value<float>(key));
    for (auto side : {Side::left, Side::right}) {
      auto tangent = V2::Key::get_tangent_optional(key, side);
      if (tangent) {
        V2::Tangent::put_ordinate(*tangent,
                                  a * V2::Tangent::get_ordinate<float>(*tangent));
      }
    }
  }
}

void fix_missing_unit(ptree &curve, bool verbose) {
  try {
    V2::ActuatorCurve::get_unit(curve);
  } catch (...) {
    auto actuator = V2::ActuatorCurve::get_actuator(curve);

    bool is_hand = ((actuator == "LHand") || (actuator == "RHand"));
    V2::ActuatorCurve::put_unit(curve,
                                is_hand ? Unit::dimensionless
                                        : Unit::degree);
    if (verbose) {
      std::cerr << "setting \"" << actuator << "\" actuator unit to "
                << (is_hand ? "dimensionless" : "degree") << "\n";
    }
  }
}
}

ptree create_root() {
  ptree root;
  ptree &v2_animation = AL::qianim::V2::require_animation(root);
  // we try to preserve editor tags
  v2_animation.put("<xmlattr>.xmlns:editor", "http://www.aldebaran.com/animation/editor");
  return root;
}

ptree v2_root_from_xar_timeline(const ptree &xar_timeline) {
  const auto fps = xar_timeline.get<int>("<xmlattr>.fps");
  auto tr = [fps](ptree::value_type kv) {
    // we take kv by copy so that we can alter it.
    ActuatorCurve::migrate_xar_to_v2(kv.second, fps);
    return kv;
  };
  ptree root = create_root();
  using AL::qianim::V2::ActuatorCurve::is_actuatorcurve;
  boost::transform(
        boost::adaptors::filter(xar_timeline.get_child("ActuatorList"),
                                is_actuatorcurve),
        std::back_inserter(AL::qianim::V2::get_animation(root)),
        tr);
  return root;
}

void _recurse_over_xar(const ptree::value_type & el,
                       std::vector<ptree> &out) {
  if (el.first == "Timeline" &&
      el.second.get_child_optional("<xmlattr>.fps") &&
      el.second.get_child_optional("ActuatorList")) {
    out.push_back(v2_root_from_xar_timeline(el.second));
  }
  if (el.first == "ChoregrapheProject" ||
      el.first == "ChoregrapheBox" ||
      el.first == "Timeline" ||
      el.first == "BehaviorLayer" ||
      el.first == "BehaviorKeyframe" ||
      el.first == "Diagram" ||
      el.first == "Box") {
    // el is an element which can have Timeline descendants,
    // let recurse
    for (const auto &child : el.second) {
      _recurse_over_xar(child, out);
    }
  }
}

std::vector<ptree> v2_roots_from_xar(const ptree &docroot) {
  std::vector<ptree> out;
  for (const auto &el : docroot) {
    _recurse_over_xar(el, out);
  }
  return out;
}

ptree v2_root_from_v1_root(const ptree &v1_root) {
  const auto &animation = v1_root.get_child("Animation");
  if (animation.get<std::string>("<xmlattr>.typeVersion") != "1.0")
    throw std::invalid_argument(".qianim file of version 1.0 is expected");

  auto tr = [](ptree::value_type kv) {
    // we take kv by copy so that we can alter it.
    ActuatorCurve::migrate_v1_to_v2(kv.second);
    return kv;
  };

  ptree root = create_root();
  using AL::qianim::V2::ActuatorCurve::is_actuatorcurve;
  boost::transform(
        boost::adaptors::filter(animation.get_child("ActuatorList"),
                                is_actuatorcurve),
        std::back_inserter( AL::qianim::V2::get_animation(root)),
        tr);
  return root;
}
}
}

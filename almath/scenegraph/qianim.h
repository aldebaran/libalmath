/*
 * Copyright (c) 2016 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#ifndef LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_H
#define LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_H

#include <almath/api.h>
#include <almath/scenegraph/qianim/unit.h>
#include <iterator>
#include <type_traits>
#include <boost/property_tree/ptree.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <functional>

namespace AL {
namespace qianim {

using ptree = boost::property_tree::ptree;

template<class T>
using optional = boost::optional<T>;

enum struct Side : bool {
  left,
  right
};

namespace V2 {

// Apply binary functor on each pair of adjacent iterators in the range
// [first, last).
template <typename InputIt, typename BinaryOperation>
BinaryOperation adjacent_for_each(InputIt first, InputIt last,
                                  BinaryOperation binary_op) {
  if (first == last)
    return binary_op;
  auto next = std::next(first);
  while (next != last) {
    binary_op(*first, *next);
    first = next;
    ++next;
  }
  return binary_op;
}

namespace Tangent {

// homogeneous to Key::frame()
// Note: this function does *not* check that the abscissa sign is consistent
// with the tangent side
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type
get_abscissa(const ptree &pt) {
  return pt.get<T>("<xmlattr>.abscissaParam");
}

// Note: this function does *not* check that the abscissa sign is consistent
// with the tangent side
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value>::type
put_abscissa(ptree &pt, T abscissa) {
  pt.put<T>("<xmlattr>.abscissaParam", abscissa);
}

// homogeneous to Key::value()
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type
get_ordinate(const ptree &pt) {
  return pt.get<T>("<xmlattr>.ordinateParam");
}

template <typename T>
typename std::enable_if<std::is_floating_point<T>::value>::type
put_ordinate(ptree &pt, T ordinate) {
  pt.put<T>("<xmlattr>.ordinateParam", ordinate);
}

ALMATH_API Side get_side(const ptree &pt);
ALMATH_API void put_side(ptree &pt, Side side);
}

namespace Key {
// return true if the argument is a Key element
ALMATH_API bool is_key(const ptree::value_type &val);

// return key frame, which is non-negative
ALMATH_API int get_frame(const ptree &pt);
ALMATH_API void put_frame(ptree &pt, int frame);

ALMATH_API ptree::size_type erase_tangent(ptree &pt, Side side);

ALMATH_API optional<const ptree &> get_tangent_optional(const ptree &pt,
                                                           Side side);
ALMATH_API optional<ptree &> get_tangent_optional(ptree &pt, Side side);

inline const ptree &get_tangent(const ptree &pt, Side side) {
  return get_tangent_optional(pt, side).value();
}

inline ptree &get_tangent(ptree &pt, Side side) {
  return get_tangent_optional(pt, side).value();
}

ALMATH_API ptree &require_tangent(ptree &pt, Side side);

// Note: this function does *not* check that the abscissa sign is consistent
// with the tangent side
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, ptree &>::type
put_tangent(ptree &pt, Side side, T abscissa, T ordinate) {
  ptree &tangent = require_tangent(pt, side);
  Tangent::put_abscissa<T>(tangent, abscissa);
  Tangent::put_ordinate<T>(tangent, ordinate);
  return tangent;
}

template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type
get_value(const ptree &pt) {
  return pt.get<T>("<xmlattr>.value");
}

template <typename T>
typename std::enable_if<std::is_floating_point<T>::value>::type
put_value(ptree &pt, T value){
  pt.put("<xmlattr>.value", value);
}


// throw if:
//   an interval has non-positive duration
//   an interval has a tangent with out of bounds abscissa
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value>::type
check_cubic_bezier(int p0_frame, T, T p1_dframe, T,
                   T p2_dframe, T, int p3_frame, T) {
  // interval duration, in frames
  auto dframe = p3_frame - p0_frame;
  if (dframe <= 0) {
    throw std::invalid_argument(
        "successive Key elements shall have increasing frame attributes");
  }
  if (p1_dframe  < 0 || p1_dframe > dframe) {
    throw std::invalid_argument(
        "Key has right Tangent with out of bound abscissa attribute");
  }
  if (p2_dframe > 0 || p2_dframe < -dframe) {
    throw std::invalid_argument(
        "Key has left Tangent with out of bound abscissa attribute");
  }
}

// Apply functor to cubic Bezier curve parameters
//
// The Bezier curve is defined by the four points read from two adjacent
// keys
//
//  p0: (p0_frame            , p0_value)
//  p1: (p0_frame + p1_dframe, p0_value + p1_dvalue)
//  p2: (p3_frame + p2_dframe, p3_value + p2_dvalue)
//  p3: (p3_frame, p3_value)
//
// The functor has the following signature:
//  void op(int p0_frame, Scalar p0_value,
//          Scalar p1_dframe, Scalar p1_dvalue,
//          Scalar p2_dframe, Scalar p2_dvalue,
//          int p3_frame, Scalar p3_value)
//
template <typename Scalar, typename Functor>
typename std::enable_if<std::is_floating_point<Scalar>::value>::type
apply_cubic_bezier(const ptree &p0_key, const ptree &p3_key, Functor op) {
  const ptree &p1_tangent = Key::get_tangent(p0_key, Side::right);
  const ptree &p2_tangent = Key::get_tangent(p3_key, Side::left);
  op(Key::get_frame(p0_key),
     Key::get_value<Scalar>(p0_key),
     Tangent::get_abscissa<Scalar>(p1_tangent),
     Tangent::get_ordinate<Scalar>(p1_tangent),
     Tangent::get_abscissa<Scalar>(p2_tangent),
     Tangent::get_ordinate<Scalar>(p2_tangent),
     Key::get_frame(p3_key),
     Key::get_value<Scalar>(p3_key));
}
}

namespace ActuatorCurve {
// return true if the argument is an ActuatorCurve element
ALMATH_API bool is_actuatorcurve(const ptree::value_type &val);

// return actuator name
ALMATH_API std::string get_actuator(const ptree &pt);
ALMATH_API void put_actuator(ptree &pt, const std::string &name);

// return fps, which is positive
ALMATH_API int get_fps(const ptree &pt);
ALMATH_API void put_fps(ptree &pt, int fps);

ALMATH_API bool get_mute(const ptree &pt);
ALMATH_API void put_mute(ptree &pt, bool mute);

// return unit, which is not UNIT_UNKNOWN
ALMATH_API Unit get_unit(const ptree &pt);
ALMATH_API void put_unit(ptree &pt, Unit unit);

// Search for a Key element with the given frame.
// If found, return it, otherwise return an invalid optional.
// This function assumes the Key elements are sorted by increasing frame,
// which should be the case if the document is conforming.
ALMATH_API optional<const ptree &> get_key_optional(const ptree &pt,
                                                       int frame);
ALMATH_API optional<ptree &> get_key_optional(ptree &pt, int frame);

// Search for a Key element with the given frame.
// Throw if not found, otherwise return an invalid optional.
// This function assumes the Key elements are sorted by increasing frame,
// which should be the case if the document is conforming.
inline const ptree &get_key(const ptree &pt, int frame) {
  return get_key_optional(pt, frame).value();
}
inline ptree &get_key(ptree &pt, int frame) {
  return get_key_optional(pt, frame).value();
}

// Search for a Key element with the given frame.
// If found, return it, otherwise insert a new Key.
// This function assumes the Key elements are sorted by increasing frame,
// which should be the case if the document is conforming.
ALMATH_API ptree &require_key(ptree &pt, int frame);

// return the range of child Key elements, in document order.
// If the document is conforming, the keys are sorted by increasing frame.
inline auto get_keys(const ptree &pt)
-> boost::select_second_const_range<
      decltype(boost::adaptors::filter(pt, Key::is_key))>
{
  return boost::adaptors::values(boost::adaptors::filter(pt, Key::is_key));
}

inline auto get_keys(ptree &pt)
-> boost::select_second_mutable_range<
       decltype(boost::adaptors::filter(pt, Key::is_key))> {
  auto kv = boost::adaptors::filter(pt, Key::is_key);
  return boost::adaptors::values(kv);
}

// throw if the curve does not describe a mathematical function as a
// sequence of cubic Bezier curves of positive duration
template <typename Scalar>
typename std::enable_if<std::is_floating_point<Scalar>::value>::type
check_cubic_bezier(const ptree &pt) {
  auto keys = get_keys(pt);
  adjacent_for_each(
      keys.begin(), keys.end(),
      [](const ptree &p0_key, const ptree &p3_key) {
          Key::apply_cubic_bezier<Scalar>(p0_key, p3_key,
                                          Key::check_cubic_bezier<Scalar>);
      });
}
}

namespace Animation {
// return the range of child ActuatorCurve elements, in document order.
// If the document is conforming, the keys are sorted by increasing frame.
inline auto get_actuatorcurves(const ptree &pt)
-> boost::select_second_const_range<
      decltype(boost::adaptors::filter(pt, ActuatorCurve::is_actuatorcurve))> {
  return boost::adaptors::values(
             boost::adaptors::filter(pt, ActuatorCurve::is_actuatorcurve));
}

inline auto get_actuatorcurves(ptree &pt)
-> boost::select_second_mutable_range<
       decltype(boost::adaptors::filter(pt, ActuatorCurve::is_actuatorcurve))> {
  auto kv = boost::adaptors::filter(pt, ActuatorCurve::is_actuatorcurve);
  return boost::adaptors::values(kv);
}

// throw if version is not 2.0
ALMATH_API void check_version(const ptree &pt);

// throw if animation or one of its children is invalid
ALMATH_API void check_all(const ptree &pt);

ALMATH_API ptree &require_actuatorcurve(ptree &pt,
                                        const std::string &actuator);

}

// return the Animation element, checking its version
ALMATH_API ptree &get_animation(ptree &root);
ALMATH_API const ptree &get_animation(const ptree &root);
}
}
}

#endif

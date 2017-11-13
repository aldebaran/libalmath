/*
 * Copyright (c) 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

// utility functions
#ifndef LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_SURGEON_H
#define LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_SURGEON_H

#include <almath/api.h>
#include <almath/scenegraph/qianim.h>
#include <boost/range/algorithm/for_each.hpp>
#include <memory>
#include <vector>

namespace AL {
namespace qianim {

namespace Tangent {

// migrate a v1 Tangent element of type "bezier" or "linear".
//
// In case of "bezier" that tangent is clamped if needed
//
// v1 Tangent elements of type "bezier_auto" should not exist:
// in this case there shall be no Tangent element.
// v1 Tangent elements of type "constant" are currently not supported,
// since I expect they are not used in practice.
ALMATH_API void migrate_v1_to_v2(ptree &tangent, float max_abs_abscissa,
                                 Side side);
}

namespace Key {

// offset the frame attibute of the given Key element with the given value
ALMATH_API void offset_frame(ptree &key, int value);

ALMATH_API void migrate_smooth_symmetrical_v1_to_v2(ptree &key);

// migrate v1 tangents of Key element
//
// key: Key element to be modifed. It must have v1 tangents.
// prev_key: previous Key element, if any
// next_key: next Key element, if any
//
// Notes
//
// * tangent children elements of prev_key and next_key elements won't
//   be used. As a consequence, it does not matter if those tangents are
//   in v1 or v2 format.
//
// * in order to resolve BEZIER_AUTO tangents, the algorithm uses the
//   difference of the frame attributes of prev_key, key and next_key.
//   As a consequence, all three keys shall use the same convention
//   (either be zero-based (like v1 Key) or one-based (like v2 Key).
//
// * if prev_key or next_key is missing, then "key" is the first or
//   the last key (or both if the curve has a single key). In such
//   a case, the Key is a local extremum and BEZIER_AUTO tangent
//   resolution picks an horizontal tangent.
//   Note that if the first Key right tangent is BEZIER_AUTO, the conversion
//   to v2 introduces a behavior change:
//   with v1, the BEZIER_AUTO resolution would have been done at runtime,
//   using the actual robot posture as "prev_key" at frame 0/1.
//   The resolved tangent might not have been horizontal. When converting
//   to v2, we resolve the tangent at resolution time and the resulting
//   tangent is horizontal.
//
// * "exterior tangents" (ie. the left Tangent of the first Key and the right
//    Tangent of the last Key) are not really part of the Actuator curve.
//    However, they are converted and kept if present.
ALMATH_API
void migrate_tangents_v1_to_v2(optional<const ptree &> prev_key,
                               ptree &key,
                               optional<const ptree &> next_key);

// migrate v1 tangents of all the Key elements from the range.
// Throw if the keys are not sorted by ascending frame attribute.
template <typename Keys>
void migrate_tangents_v1_to_v2(Keys keys) {
  auto cur = boost::begin(keys);
  if (cur == boost::end(keys))
    return; // no key to migrate
  optional<const ptree &> prev_key;
  auto next = cur;
  ++next;
  while (next != boost::end(keys)) {
    Key::migrate_tangents_v1_to_v2(prev_key, *cur, *next);
    prev_key = *cur;
    cur = next;
    ++next;
  }
  Key::migrate_tangents_v1_to_v2(prev_key, *cur, optional<const ptree &>());
}

template <typename Keys>
void migrate_v1_to_v2(Keys keys) {
  // beware: we cannot migrate the key frame and the tangents in the same loop
  Key::migrate_tangents_v1_to_v2(keys);
  boost::for_each(keys,
                  [] (ptree &key) {
                      Key::offset_frame(key, -1);
                      Key::migrate_smooth_symmetrical_v1_to_v2(key);});
}
}

namespace ActuatorCurve {

// convert a curve unit (typically degree <-> radian)
ALMATH_API void convert_unit(ptree &curve, Unit to, Unit from);

// add a unit if missing: "dimensionless" if the curve is named
// "LHand" or "RHand", "degree" otherwise.
//
// Note: maybe we could implement it another way when migrating a xar:
//   from http://opengrok.aldebaran.lan/xref/gui/lib/desktop/behavior/behaviormodel/behaviormodel/timeline/actuator_curve.h#48
//   and http://opengrok.aldebaran.lan/xref/behavior/framemanager/src/fmActuatorCurve.h#30
// one sees that 0 means degrees and 1 means dimensionless.
// Yet, they hardcode the names LHAnd and RHand there too:
//    http://opengrok.aldebaran.lan/xref/gui/lib/desktop/behavior/behavior_utils/behavior_utils/box_utils.h#45
ALMATH_API void fix_missing_unit(ptree &curve, bool verbose=false);

inline void migrate_v1_to_v2(ptree &curve) {
  Key::migrate_v1_to_v2(V2::ActuatorCurve::get_keys(curve));
  ActuatorCurve::fix_missing_unit(curve);
}

inline void migrate_xar_to_v2(ptree &curve, int fps) {
  AL::qianim::V2::ActuatorCurve::put_fps(curve, fps);
  Key::migrate_v1_to_v2(V2::ActuatorCurve::get_keys(curve));
  ActuatorCurve::fix_missing_unit(curve);
  // remove legacy attributes inherited from xar
  auto &attr = curve.get_child("<xmlattr>");
  attr.erase("name");
  attr.erase("recordable");
}
}

// create an animation document from a XAR timeline
ALMATH_API ptree v2_root_from_xar_timeline(const ptree &xar_timeline);

// create an animation document for each timeline in a XAR
ALMATH_API std::vector<ptree> v2_roots_from_xar(const ptree &docroot);

// create an animation document from a V1 .qianim document
ALMATH_API ptree v2_root_from_v1_root(const ptree &v1_root);

}
}
#endif

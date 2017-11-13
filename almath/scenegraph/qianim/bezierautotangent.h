/*
 * Copyright (c) 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#ifndef LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_BEZIERAUTOTANGENT_H
#define LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_BEZIERAUTOTANGENT_H

#include <almath/api.h>
#include <utility>
#include <cassert>
#include <cmath>
#include <type_traits>

namespace AL {
namespace qianim {

// Compute the two tangents of a legacy (xar/qianim v1) bezier auto key.
//
// pDeltaTime1 is the time between this key and previous key,
// pDeltaTime2 is the time between next key and this key.
// pDeltaAngle1 is the angle change between this key and previous key,
// pDeltaAngle2 is the angle change between next key and this key.
//
// pDeltaTime1 and pDeltaTime2 shall be non-negative
// If the key is not a local extremum (ie. if pDeltaAngle1 * pDeltaAngle2 > 0)
// then pDeltaTime1 + pDeltaTime2 shall be positive
//
// Return the (left tangent, right tangent) pair, where each tangent is
// an (abscissa, ordinate) pair.
template <typename Scalar,
         typename std::enable_if<std::is_floating_point<Scalar>::value, int>::type = 0 >
std::pair<std::pair<Scalar, Scalar>, std::pair<Scalar, Scalar>>
computeBezierAutoTangents(Scalar pDeltaTime1, Scalar pDeltaTime2,
                          Scalar pDeltaAngle1, Scalar pDeltaAngle2) {
  assert(pDeltaTime1 > -1e-5f);
  assert(pDeltaTime2 > -1e-5f);

  const auto alpha = Scalar(1) / 3;
  auto beta = Scalar(0);

  if (pDeltaAngle1 * pDeltaAngle2 > 0) {
    // Key is not a local extremum
    assert(pDeltaTime1 + pDeltaTime2 > 0);
    beta = (pDeltaAngle1 + pDeltaAngle2) / (pDeltaTime1 + pDeltaTime2);

    // avoid overshooting
    auto tgtHeight = std::abs(alpha * pDeltaTime2 * beta);
    auto absDeltaAngle = std::abs(pDeltaAngle2);
    if (tgtHeight > absDeltaAngle) {
      beta *= absDeltaAngle / tgtHeight;
    }
    tgtHeight = std::abs(alpha * pDeltaTime1 * beta);
    absDeltaAngle = std::abs(pDeltaAngle1);
    if (tgtHeight > absDeltaAngle) {
      beta *= absDeltaAngle / tgtHeight;
    }
  }
  return std::make_pair(std::make_pair(-alpha * pDeltaTime1,
                                       -alpha * beta * pDeltaTime1),
                        std::make_pair(alpha * pDeltaTime2,
                                       alpha * beta * pDeltaTime2));
}
}
}

#endif

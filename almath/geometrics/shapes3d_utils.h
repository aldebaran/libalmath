/**
 * @author Justine Lança - jlanca@aldebaran-robotics.com
 * @author Nicolas Garcia - ngarcia@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2014 All Rights Reserved
 *
 */

#ifndef LIB_ALMATH_GEOMETRICS_SHAPES3D_UTILS_H
#define LIB_ALMATH_GEOMETRICS_SHAPES3D_UTILS_H

#include <almath/api.h>
#include <almath/types/alposition3d.h>
#include <almath/types/altransform.h>

namespace AL {
namespace Math {

ALMATH_API
void getSphereCentersFromPill(float pHalfExtent,
                              const Math::Transform &pEndTfToPillCenter,
                              Math::Position3D &pCenterA,
                              Math::Position3D &pCenterB);

// compute the pill center and half extent so that the pill vector is oriented
// from A to B
ALMATH_API
void computePillParameters(const Math::Position3D &pEndTfToCenterA,
                           const Math::Position3D &pEndTfToCenterB,
                           float &pHalfExtent,
                           Math::Transform &pEndTfToPillCenter);

}  // End namespace Math.
}  // End namespace AL.

#endif  // LIB_ALMATH_GEOMETRICS_SHAPES3D_UTILS_H

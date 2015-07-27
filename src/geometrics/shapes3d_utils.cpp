/**
 * @author Justine Lança - jlanca@aldebaran-robotics.com
 * @author Nicolas Garcia - ngarcia@aldebaran.com
 * Aldebaran (c) 2014 All Rights Reserved
 *
 */

#include <almath/geometrics/shapes3d_utils.h>
#include <almath/tools/altransformhelpers.h>

namespace AL {
namespace Math {

void getSphereCentersFromPill(float pHalfExtent,
                              const Math::Transform &pEndTfToShape,
                              Math::Position3D &pCenterA,
                              Math::Position3D &pCenterB) {
  pCenterA = pEndTfToShape * Math::Position3D(0.f, 0.f, pHalfExtent);
  pCenterB = pEndTfToShape * Math::Position3D(0.f, 0.f, -pHalfExtent);
}

void computePillParameters(const Math::Position3D &pEndTfToCenterA,
                           const Math::Position3D &pEndTfToCenterB,
                           float &pHalfExtent,
                           Math::Transform &pEndTfToPillCenter) {
  pHalfExtent = 0.5f * pEndTfToCenterA.distance(pEndTfToCenterB);
  const Math::Position3D halfDelta = (pEndTfToCenterB - pEndTfToCenterA) * 0.5f;
  const Math::Position3D center = pEndTfToCenterA + halfDelta;
  pEndTfToPillCenter = Math::transformFromPosition3D(center);
  Math::orthogonalSpace(halfDelta, pEndTfToPillCenter);
}

}  // End namespace Math.
}  // End namespace AL.

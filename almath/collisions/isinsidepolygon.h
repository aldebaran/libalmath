#pragma once

#ifndef _LIB_ALMATH_ALMATH_ISINSIDEPOLYGON_H_
#define _LIB_ALMATH_ALMATH_ISINSIDEPOLYGON_H_

#include <almath/types/alposition2d.h>
#include <vector>

namespace AL
{
  namespace Math
  {

    /// <summary> Query if two segment intersect (AB <-> CD). </summary>
    /// <param name="pA"> Position2D of the point A. </param>
    /// <param name="pB"> Position2D of the point B. </param>
    /// <param name="pC"> Position2D of the point C. </param>
    /// <param name="pD"> Position2D of the point D. </param>
    /// <returns> true if segment intersect. </returns>
    const bool xIsSegementIntersect(
      const AL::Math::Position2D& pA,
      const AL::Math::Position2D& pB,
      const AL::Math::Position2D& pC,
      const AL::Math::Position2D& pD);


    /// <summary> Query if 'pPoint' is in support polygon. </summary>
    /// <param name="pPoint">   The point. </param>
    /// <param name="pPolygon"> The polygon. </param>
    /// <returns> true if in support polygon, false if not. </returns>
    const bool isInSupportPolygon(
      const Position2D&              pTestPoint,
      const std::vector<Position2D>& pPolygon);

  } // namespace Math

} // namespace AL

#endif  // _LIB_ALMATH_ALMATH_ISINSIDEPOLYGON_H_


/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Copyright (c) Aldebaran Robotics 2010 All Rights Reserved
 *
 */

#include <almath/collisions/isinsidepolygon.h>

namespace AL
{
  namespace Math
  {

    const bool xIsSegementIntersect(
      const AL::Math::Position2D& pA,
      const AL::Math::Position2D& pB,
      const AL::Math::Position2D& pC,
      const AL::Math::Position2D& pD)
    {
      // We can define the two segment by equation
      // pX = pA + Ua*(pB-pA)
      // pY = pC + Ub*(pD-pC)
      // Find intersection point is equal to solve pX = pY
      float denom = ((pD.y-pC.y)*(pB.x-pA.x) - (pB.y-pA.y)*(pD.x-pC.x));

      float num_a = ((pA.y-pC.y)*(pD.x-pC.x) - (pD.y-pC.y)*(pA.x-pC.x));
      float num_b = ((pA.y-pC.y)*(pB.x-pA.x) - (pB.y-pA.y)*(pA.x-pC.x));

      if (denom == 0.0f)
      {
        if( (num_a == 0.0f) && (num_b == 0.0f))
        {
          // COINCIDENT
          return true;
        }
        // PARALLEL
        return false;
      }

      float ua = num_a/denom;
      float ub = num_b/denom;
      if((ua >= 0.0f) && (ua <= 1.0f) && (ub >= 0.0f) && (ub <= 1.0f))
      {
        // INTERSECTION
        return true;
      }
      // NO INTERSECTION
      return false;
    }


    const bool isInSupportPolygon(
      const AL::Math::Position2D&              pTestPoint,
      const std::vector<AL::Math::Position2D>& pPolygon)
    {
      // test size of support polygon
      if (pPolygon.empty())
      {
        return false;
      }

      int counter = 0;

      // create point far away of polygon (1 meter)
      AL::Math::Position2D newPoint = pTestPoint + AL::Math::Position2D(10.0f, 9.0f);

      for (unsigned int i=0; i<pPolygon.size()-1; i++)
      {
        if(xIsSegementIntersect(pPolygon.at(i), pPolygon.at(i+1), pTestPoint, newPoint))
        {
          counter++;
        }
      }

      if(xIsSegementIntersect(pPolygon.back(), pPolygon.at(0), pTestPoint, newPoint))
      {
        counter++;
      }

      if(counter%2 == 0)
      {
        // EXTERIOR
        return false;
      }
      else
      {
        // INTERIOR
        return true;
      }
    } // end isInSupportPolygon

  } // namespace Math

} // namespace AL


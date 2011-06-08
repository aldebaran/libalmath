/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#include <almath/collisions/allinecollisions.h>

#include "float.h"
#include "math.h"
#include <vector>

#include <almath/types/alposition3d.h>

namespace AL
{
  namespace Math
  {

    float DistancePointToLineSegment3D(
        Position3D& pA,
        Position3D& pB,
        Position3D& pC)
    {
      Position3D closestPoint = ClosestPointOfLineSegment3D(pA, pB, pC);
      return distance(pC, closestPoint);
    }


    Position3D ClosestPointOfLineSegment3D(Position3D& pA, Position3D& pB, Position3D& pC)
    {
      //Compute vector of the line segment
      //and the vector from the point to the start of the line
      Position3D ABVector = pB - pA;
      Position3D ACVector = pC - pA;
      //Where is the point? (relative to the points of the segment)
      float ACDistance = dotProduct(ACVector,ABVector);

      //Point C is close to the start of the line segment
      if (ACDistance <= 0)
      {
        return pA;
      }
      float ABDistance = dotProduct(ABVector,ABVector);

      //Point C is close to the end of the line segment
      if (ABDistance <= ACDistance)
      {
        return pB;
      }

      //Point C is between the two points of the line segment
      float b = ACDistance / ABDistance;
      Position3D projectionOnAB = pA + (ABVector * b);
      return projectionOnAB;
    }


    float DistanceBetweenTwoLineSegment3D(
        Position3D& pA,
        Position3D& pB,
        Position3D& pC,
        Position3D& pD)
    {
      Position3D vectAB = pB - pA;
      Position3D vectCD = pD - pC;
      Position3D vectAC = pA - pC;
      float a = AL::Math::dotProduct(vectAB,vectAB); // always >= 0
      float b = AL::Math::dotProduct(vectAB,vectCD);
      float c = AL::Math::dotProduct(vectCD,vectCD); // always >= 0
      float d = AL::Math::dotProduct(vectAB,vectAC);
      float e = AL::Math::dotProduct(vectCD,vectAC);
      float D = a*c - b*b; // always >= 0
      float sc, sN, sD = D; // sc = sN / sD, default sD = D >= 0
      float tc, tN, tD = D; // tc = tN / tD, default tD = D >= 0

      // compute the line parameters of the two closest points
      if (D < FLT_MIN) { // the lines are almost parallel
        sN = 0.0f; // force using point P0 on segment S1
        sD = 1.0f; // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
      }
      else { // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0f) { // sc < 0 => the s=0 edge is visible
          sN = 0.0f;
          tN = e;
          tD = c;
        }
        else if (sN > sD) { // sc > 1 => the s=1 edge is visible
          sN = sD;
          tN = e + b;
          tD = c;
        }
      }

      if (tN < 0.0f) { // tc < 0 => the t=0 edge is visible
        tN = 0.0f;
        // recompute sc for this edge
        if (-d < 0.0f)
          sN = 0.0f;
        else if (-d > a)
          sN = sD;
        else {
          sN = -d;
          sD = a;
        }
      }
      else if (tN > tD) { // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0f)
          sN = 0.0f;
        else if ((-d + b) > a)
          sN = sD;
        else {
          sN = (-d + b);
          sD = a;
        }
      }
      // finally do the division to get sc and tc
      sc = ( fabsf(sN) < FLT_MIN ? 0.0f : sN / sD);
      tc = ( fabsf(tN) < FLT_MIN ? 0.0f : tN / tD);

      // get the difference of the two closest points
      Position3D dP = vectAC + (vectAB * sc) - (vectCD * tc); // = S1(sc) - S2(tc)

      return norm(dP); // return the closest distance
    }

  } // namespace Math

} // namespace AL

/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/avoidfootcollision.h>
#include <almath/tools/almath.h>
#include <cmath>

namespace AL
{
  namespace Math
  {

    /****************************
    PRIVATE FUNCTION
    ****************************/
    // <summary> Checks if segment A intersects segment B. </summary>
    // <param name="pxA1">  x component of first point of segment A. </param>
    // <param name="pyA1">  y component of first point of segment A. </param>
    // <param name="pxA2">  x component of second point of segment A. </param>
    // <param name="pyA2">  y component of second point of segment A. </param>
    // <param name="pxB1">  x component of first point of segment B. </param>
    // <param name="pyB1">  y component of first point of segment B. </param>
    // <param name="pxB2">  x component of second point of segment B. </param>
    // <param name="pyB2">  y component of second point of segment B. </param>
    // <param name="pxC">  computed x component of intersection. </param>
    // <param name="pyC">  computed y component of intersection. </param>
    // <returns> true if the segments intersect in a single point (pC). </returns>
    bool xIntersectionSegment2D(
        float pxA1, float pyA1,
        float pxA2, float pyA2,
        float pxB1, float pyB1,
        float pxB2, float pyB2,
        float &pxC, float &pyC)
    {
      // SegmentA --> pA1 + rA * (pA2 - pA1);
      // SegmentB --> pB1 + rB * (pB2 - pB1);
      const float gxA = pxA2 - pxA1;
      const float gyA = pyA2 - pyA1;
      const float gxB = pxB2 - pxB1;
      const float gyB = pyB2 - pyB1;

      const float lTol = 1e-5f;
      const float den  = gxB*gyA - gxA*gyB;
      if (std::abs(den) < lTol) //Parallel lines or infinitesimal segments
      {
        return false;
      }

      const float rB = (gyA*(pxA1 - pxB1) + gxA*(pyB1 - pyA1))/den;
      if (rB < 0.0f || rB > 1.0f) //Intersection out of the segment B
      {
        return false;
      }
      float rA = 0.0f;
      if (std::abs(gxA) <lTol)
      {
        if(std::abs(gyA) <lTol)
        {
          return false;
        }
        else
        {
          rA = (pyB1 - pyA1 + rB*gyB)/gyA;
        }
      }
      else
      {
         rA = (pxB1 - pxA1 + rB*gxB)/gxA;
      }
      if (rA < 0.0f || rA > 1.0f) //Intersection out of the segment A
      {
        return false;
      }
      pxC = pxA1 + rA * gxA;
      pyC = pyA1 + rA * gyA;
      return true;
    }

    bool intersectionSegment2D(
        const Position2D &pA1,
        const Position2D &pA2,
        const Position2D &pB1,
        const Position2D &pB2,
        Position2D       &pC)
    {
      return xIntersectionSegment2D(
            pA1.x, pA1.y,
            pA2.x, pA2.y,
            pB1.x, pB1.y,
            pB2.x, pB2.y,
            pC.x, pC.y);
    }

    /****************************
    PRIVATE FUNCTION
    ****************************/
    // <summary> Query if the points B is inside the box A. </summary>
    // <param name="pBoxA">  vector<Position2D> of the box A. </param>
    // <param name="pPointB"> Position2D of the point B. </param>
    // <returns> true if point B is inside Box A. </returns>
    const bool xPointsInsideBox(
        const std::vector<Position2D> &pBoxA,
        const Position2D              &pPointB);


    const bool xPointsInsideBox(
      const std::vector<Position2D>&  pBoxA,
      const Position2D&               pPointB)
    {
      // the main idea is to follow by the right side
      // the box A.
      // Then with cross produc and his determinant
      // we know if the points is on the left or on the right of
      // the actual segment
      // If always on theright the points is inside the boxes
      unsigned int length = pBoxA.size();
      unsigned int iPlusOne = 0;

      float Dx, Dy;
      float Tx, Ty;
      float Det;

      for(unsigned int i=0; i<length; ++i)
      {
        iPlusOne = i+1;
        if(iPlusOne == length)
        {
          iPlusOne = 0;
        }
        // Now just a cross product
        Dx = pBoxA[iPlusOne].x - pBoxA[i].x;
        Dy = pBoxA[iPlusOne].y - pBoxA[i].y;
        Tx = pPointB.x - pBoxA[i].x;
        Ty = pPointB.y - pBoxA[i].y;
        Det = Dx*Ty - Dy*Tx;
        if(Det >= 0.0f)
        {
          // on the left so NO INSIDE
          return false;
        }
      }
      return true;
    } // end xPointsInsideBox()


    const bool areTwoBoxesInCollision(
        const std::vector<Position2D> &pBoxA,
        const std::vector<Position2D> &pBoxB)
    {
      Position2D intersection;
      const std::size_t sizeA = pBoxA.size();
      const std::size_t sizeB = pBoxB.size();

      //Check intersection between the sides of the boxes
      for (std::size_t ia = 0u; ia < sizeA; ++ia)
      {
        for (std::size_t ib = 0u; ib < sizeB; ++ib)
        {
          if (intersectionSegment2D(
                pBoxA[ia], pBoxA[(ia+1) % sizeA],
                pBoxB[ib], pBoxB[(ib+1) % sizeB],
                intersection))
          {
            return true;
          }
        }
      }
      //Check if one box is inside the other
      if (sizeA > 0)
      {
        if (xPointsInsideBox(pBoxB, pBoxA[0]))
        {
          return true;
        }
      }
      if (sizeB > 0)
      {
        if (xPointsInsideBox(pBoxA,pBoxB[0]))
        {
          return true;
        }
      }

      return false;
    } // end xIsTwoBoxesAreInCollision()



    void computeBox(
        const std::vector<Position2D> &pInitBox,
        const Pose2D                  &pMovingPose,
        std::vector<Position2D>       &pMovedBox)
    {
      pMovedBox.clear();
      pMovedBox.reserve(pInitBox.size());
      for(unsigned int i=0u; i < pInitBox.size(); ++i)
      {
        pMovedBox.push_back(pMovingPose * pInitBox[i]);
      }
    } // end computeBox()


    const void dichotomie(
        const std::vector<Position2D> &pFixedBox,
        const std::vector<Position2D> &pMovingBox,
        Pose2D                        &pMovingPose)
    {
      // the dichotomie number of iteration = precision
      unsigned int nbIteration = 5u;

      float min = 0.0f;
      float max = pMovingPose.theta;
      float middle = 0.0f;
      float bestNoCollision = 0.0f;

      std::vector<Position2D> tmpMovingBox;

      for(unsigned int i=0; i<nbIteration; ++i)
      {
        middle = (min + max)*0.5f;
        pMovingPose.theta = middle;
        // compute nex box position
        computeBox(pMovingBox, pMovingPose, tmpMovingBox);
        // test collision
        if( areTwoBoxesInCollision(pFixedBox, tmpMovingBox) )
        {
          max = middle;
        }
        else
        {
          bestNoCollision = middle;
          min = middle;
        }
      }
      pMovingPose.theta = bestNoCollision;
    } // end xDichotomie()

    /****************************
    PUBLIC FUNCTION
    ****************************/

    const bool avoidFootCollision(
      const std::vector<Position2D>&  pLFootBoundingBox,
      const std::vector<Position2D>&  pRFootBoundingBox,
      const bool&                     pIsLeftSupport,
      Pose2D&                         pMove)
    {
      bool returnCollisionResult = false;
      std::vector<Position2D> tmpMovingBox;
      if (pIsLeftSupport)
      {
        // compute nex box position
        computeBox(pRFootBoundingBox, pMove, tmpMovingBox);
        // test collision
        if (areTwoBoxesInCollision(pLFootBoundingBox, tmpMovingBox))
        {
          returnCollisionResult = true;
          dichotomie(pLFootBoundingBox, pRFootBoundingBox, pMove);
        }
      }
      else
      {
        // compute nex box position
        computeBox(pLFootBoundingBox, pMove, tmpMovingBox);
        // test collision
        if (areTwoBoxesInCollision(pRFootBoundingBox, tmpMovingBox))
        {
          returnCollisionResult = true;
          dichotomie(pRFootBoundingBox, pLFootBoundingBox, pMove);
        }
      }
      return returnCollisionResult;
    } // end avoidFootCollision()


    const bool clipFootWithEllipse(
      const float&    pMaxFootX,
      const float&    pMaxFootY,
      Pose2D&         pMove)
    {
      // described ellipse parameters
      const float a = std::abs(pMaxFootX);
      const float b = std::abs(pMaxFootY);
      const float a2 = std::pow(a, 2);
      const float b2 = std::pow(b, 2);

      const float norm = pMove.x*pMove.x/a2 + pMove.y*pMove.y/b2;

      // first compute is point is outside ellipse or not
      if (norm < 1.00001f)
      {
        // pMove is inside ellipse
        return false;
      }
      else
      {
        // we have to clip pMove
        // compute angle
        const float theta = std::atan2(pMove.y, pMove.x);

        // then compute polar equation
        const float cosTheta = std::cos(theta);
        const float sinTheta = std::sin(theta);
        const float t = (a*b)/(std::sqrt(b2*cosTheta*cosTheta + a2*sinTheta*sinTheta) );

        // finally compute new pMove
        pMove.x = t*cosTheta;
        pMove.y = t*sinTheta;
        return true;
      }
    }
  } // namespace Math
} // namespace AL


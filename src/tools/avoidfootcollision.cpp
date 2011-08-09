/**
 * @author David Gouaillier - ccollette@aldebaran-robotics.com
 * Copyright (c) Aldebaran Robotics 2011 All Rights Reserved
 *
 */

#include <almath/tools/avoidfootcollision.h>
#include <cmath>

namespace AL
{
  namespace Math
  {
    /****************************
    PRIVATE FUNCTION
    ****************************/
    // <summary> Query if the points B is inside the box A. </summary>
    // <param name="pBoxA">  vector<Pose2D> of the box A. </param>
    // <param name="pPointB"> Pose2D of the point B. </param>
    // <returns> true if point B is inside Box A. </returns>
    const bool xPointsInsideBox(
      const std::vector<AL::Math::Pose2D>&  pBoxA,
      const AL::Math::Pose2D&               pPointB);

    // <summary> Query if the box A and the box B are in collision. </summary>
    // <param name="pBoxA"> vector<Pose2D> of the box A. </param>
    // <param name="pBoxB"> vector<Pose2D> of the box B. </param>
    // <returns> true if box A and the box B are in collision. </returns>
    const bool xIsTwoBoxesAreInCollision(
      const std::vector<AL::Math::Pose2D>&  pBoxA,
      const std::vector<AL::Math::Pose2D>&  pBoxB);

    // <summary> Compute the box point due to a move. </summary>
    // <param name="pInitBox">  vector<Pose2D> of the fixed box. </param>
    // <param name="pMove">      Pose2D the initial move. </param>
    // <returns> vector<Pose2D> the new box with pose2D transformation. </returns>
    const std::vector<AL::Math::Pose2D> xComputeBox(
      const std::vector<AL::Math::Pose2D>&  pInitBox,
      const AL::Math::Pose2D&               pMove);

    // <summary> Compute the best orientation of the moving box without
    //  collsion with fixed box. </summary>
    // <param name="pFixesBox">  vector<Pose2D> of the fixed box. </param>
    // <param name="pMovingBox"> vector<Pose2D> of the moving box. </param>
    // <param name="pMove">       Pose2D the initial move. </param>
    // <returns> Pose2D, the best position. </returns>
    const void xDichotomie(
      const std::vector<AL::Math::Pose2D>&  pFixesBox,
      const std::vector<AL::Math::Pose2D>&  pMovingBox,
      AL::Math::Pose2D&                     pMove);

    const bool xPointsInsideBox(
      const std::vector<AL::Math::Pose2D>&  pBoxA,
      const AL::Math::Pose2D&               pPointB)
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

      for(unsigned int i=0; i<length;i++)
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


    const bool xIsTwoBoxesAreInCollision(
      const std::vector<AL::Math::Pose2D>&  pBoxA,
      const std::vector<AL::Math::Pose2D>&  pBoxB)
    {
      bool test = false;
      // test if all points of box A is outside of box B
      for(unsigned i=0; i<pBoxA.size();i++)
      {
        test = xPointsInsideBox(pBoxB, pBoxA[i]);
        if (test)
          return test;
      }
      // test if all points of box B is outside of box A
      for(unsigned i=0; i<pBoxB.size();i++)
      {
        test = xPointsInsideBox(pBoxA, pBoxB[i]);
        if (test)
          return test;
      }
      return test;
    } // end xIsTwoBoxesAreInCollision()


    const std::vector<AL::Math::Pose2D> xComputeBox(
      const std::vector<AL::Math::Pose2D>&  pInitBox,
      const AL::Math::Pose2D&               pMove)
    {
      std::vector<AL::Math::Pose2D> returnNewBox;
      for(unsigned int i=0; i < pInitBox.size();i++)
      {
        returnNewBox.push_back(pMove * pInitBox[i]);
      }
      return returnNewBox;
    } // end xComputeBox()


    const void xDichotomie(
      const std::vector<AL::Math::Pose2D>&  pFixesBox,
      const std::vector<AL::Math::Pose2D>&  pMovingBox,
      AL::Math::Pose2D&                     pMove)
    {
      // the dichotomie number of iteration = precision
      unsigned int nbIteration = 5;

      float min = 0.0f;
      float max = pMove.theta;
      float middle = 0.0f;

      std::vector<AL::Math::Pose2D> tmpMovingBox;

      for(unsigned int i=0; i<nbIteration; i++)
      {
        middle = (min + max)/2.0f;
        pMove.theta = middle;
        // compute nex box position
        tmpMovingBox = xComputeBox(pMovingBox, pMove);
        // test collision
        if( xIsTwoBoxesAreInCollision(pFixesBox, tmpMovingBox) )
          max = middle;
        else
          min = middle;
      }
      pMove.theta = (min + max)/2.0f;
    } // end xDichotomie()

    /****************************
    PUBLIC FUNCTION
    ****************************/
    const bool avoidFootCollision(
      const std::vector<AL::Math::Pose2D>&  pLFootBoundingBox,
      const std::vector<AL::Math::Pose2D>&  pRFootBoundingBox,
      const bool&                           pIsLeftSupport,
      AL::Math::Pose2D&                     pMove)
    {
      bool returnCollisionResult = false;
      std::vector<AL::Math::Pose2D> tmpMovingBox;
      if (pIsLeftSupport)
      {
        // compute nex box position
        tmpMovingBox = xComputeBox(pRFootBoundingBox, pMove);
        // test collision
        if (xIsTwoBoxesAreInCollision(pLFootBoundingBox, tmpMovingBox))
        {
          returnCollisionResult = true;
          xDichotomie(pLFootBoundingBox, pRFootBoundingBox, pMove);
        }
      }
      else
      {
        // compute nex box position
        tmpMovingBox = xComputeBox(pLFootBoundingBox, pMove);
        // test collision
        if (xIsTwoBoxesAreInCollision(pRFootBoundingBox, tmpMovingBox))
        {
          returnCollisionResult = true;
          xDichotomie(pRFootBoundingBox, pLFootBoundingBox, pMove);
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
      float a = fabs(pMaxFootX);
      float b = fabs(pMaxFootY);
      float a2 = a*a;
      float b2 = b*b;

      // first compute is point is outside ellipse or not
      if( (pMove.x*pMove.x/a2 + pMove.y*pMove.y/b2) < 1 )
      {
        // pMove is inside ellipse
        return false;
      }
      else
      {
        // we have to clip pMove
        // compute angle
        float theta = atan2(pMove.y, pMove.x);

        // then compute polar equation
        float cosTheta = cos(theta);
        float sinTheta = sin(theta);
        float t = (a*b)/( sqrt(b2*cosTheta*cosTheta + a2*sinTheta*sinTheta) );

        // finally compute new pMove
        pMove.x = t*cosTheta;
        pMove.y = t*sinTheta;
        return true;
      }
    }
  } // namespace Math
} // namespace AL


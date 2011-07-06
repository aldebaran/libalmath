/**
 * @author David Gouaillier - ccollette@aldebaran-robotics.com
 * Copyright (c) Aldebaran Robotics 2011 All Rights Reserved
 *
 */

#include <almath/collisions/avoidfootcollision.h>

namespace AL
{
  namespace Math
  {
    // <summary> Query if the points B is inside the boxe A. </summary>
    // <param name="pBoxeA">  vector<Pose2D> of the boxe A. </param>
    // <param name="pPointB"> Pose2D of the point B. </param>
    // <returns> true if point B is inside Boxe A. </returns>
    const bool xPointsInsideBoxe(
      const std::vector<AL::Math::Pose2D>&  pBoxeA,
      const AL::Math::Pose2D&               pPointB);

    // <summary> Query if the boxe A and the boxe B are in collision. </summary>
    // <param name="pBoxeA"> vector<Pose2D> of the boxe A. </param>
    // <param name="pBoxeB"> vector<Pose2D> of the boxe B. </param>
    // <returns> true if boxe A and the boxe B are in collision. </returns>
    const bool xIsTwoBoxesAreInCollision(
      const std::vector<AL::Math::Pose2D>&  pBoxeA,
      const std::vector<AL::Math::Pose2D>&  pBoxeB);

    // <summary> Compute the boxe point due to a move. </summary>
    // <param name="pInitBoxe">  vector<Pose2D> of the fixed boxe. </param>
    // <param name="pMove">      Pose2D the initial move. </param>
    // <returns> vector<Pose2D> the new boxe with pose2D transformation. </returns>
    const std::vector<AL::Math::Pose2D> xComputeBoxe(
      const std::vector<AL::Math::Pose2D>&  pInitBoxe,
      const AL::Math::Pose2D&               pMove);

    // <summary> Compute the best orientation of the moving boxe without
    //  collsion with fixed boxe. </summary>
    // <param name="pFixesBoxe">  vector<Pose2D> of the fixed boxe. </param>
    // <param name="pMovingBoxe"> vector<Pose2D> of the moving boxe. </param>
    // <param name="pMove">       Pose2D the initial move. </param>
    // <returns> Pose2D, the best position. </returns>
    const void xDichotomie(
      const std::vector<AL::Math::Pose2D>&  pFixesBoxe,
      const std::vector<AL::Math::Pose2D>&  pMovingBoxe,
      AL::Math::Pose2D&                     pMove);

    const bool avoidFootCollision(
      const std::vector<AL::Math::Pose2D>&  pLFootBoundingBoxe,
      const std::vector<AL::Math::Pose2D>&  pRFootBoundingBoxe,
      const bool&                           pIsLeftSupport,
      AL::Math::Pose2D&                     pMove )
    {
      bool returnCollisionResult = false;
      std::vector<AL::Math::Pose2D> tmpMovingBoxe;
      if( pIsLeftSupport)
      {
        // compute nex boxe position
        tmpMovingBoxe = xComputeBoxe(pRFootBoundingBoxe, pMove);
        // test collision
        if( xIsTwoBoxesAreInCollision(pLFootBoundingBoxe, tmpMovingBoxe) )
        {
          returnCollisionResult = true;
          xDichotomie(pLFootBoundingBoxe, pRFootBoundingBoxe, pMove);
        }
      }
      else
      {
        // compute nex boxe position
        tmpMovingBoxe = xComputeBoxe(pLFootBoundingBoxe, pMove);
        // test collision
        if( xIsTwoBoxesAreInCollision(pRFootBoundingBoxe, tmpMovingBoxe) )
        {
          returnCollisionResult = true;
          xDichotomie(pRFootBoundingBoxe, pLFootBoundingBoxe, pMove);
        }
      }
      return returnCollisionResult;
    } // end avoidFootCollision()


    const bool xPointsInsideBoxe(
      const std::vector<AL::Math::Pose2D>&  pBoxeA,
      const AL::Math::Pose2D&               pPointB)
    {
      // the main idea is to follow by the right side
      // the boxes A.
      // Then with cross produc and his determinant
      // we know if the points is on the left or on the right of
      // the actual segment
      // If always on theright the points is inside the boxes
      unsigned int length = pBoxeA.size();
      unsigned int iPlusOne = 0;

      float Dx, Dy;
      float Tx, Ty;
      float Det;

      for(unsigned int i=0; i<length;i++)
      {
        iPlusOne = i+1;
        if(iPlusOne == length)
          iPlusOne = 0;
        // Now just a cross product
        Dx = pBoxeA[iPlusOne].x - pBoxeA[i].x;
        Dy = pBoxeA[iPlusOne].y - pBoxeA[i].y;
        Tx = pPointB.x - pBoxeA[i].x;
        Ty = pPointB.y - pBoxeA[i].y;
        Det = Dx*Ty - Dy*Tx;
        if(Det >= 0.0f)
        {
          // on the left so NO INSIDE
          return false;
        }
      }
      return true;
    } // end xPointsInsideBoxe()


    const bool xIsTwoBoxesAreInCollision(
      const std::vector<AL::Math::Pose2D>&  pBoxeA,
      const std::vector<AL::Math::Pose2D>&  pBoxeB)
    {
      bool test = false;
      // test if all points of boxe A is outside of boxe B
      for(unsigned i=0; i<pBoxeA.size();i++)
      {
        test = xPointsInsideBoxe(pBoxeB, pBoxeA[i]);
        if (test)
          return test;
      }
      // test if all points of boxe B is outside of boxe A
      for(unsigned i=0; i<pBoxeB.size();i++)
      {
        test = xPointsInsideBoxe(pBoxeA, pBoxeB[i]);
        if (test)
          return test;
      }
      return test;
    } // end xIsTwoBoxesAreInCollision()


    const std::vector<AL::Math::Pose2D> xComputeBoxe(
      const std::vector<AL::Math::Pose2D>&  pInitBoxe,
      const AL::Math::Pose2D&               pMove)
    {
      std::vector<AL::Math::Pose2D> returnNewBoxe;
      for(unsigned int i=0; i < pInitBoxe.size();i++)
      {
        returnNewBoxe.push_back(pMove * pInitBoxe[i]);
      }
      return returnNewBoxe;
    } // end xComputeBoxe()


    const void xDichotomie(
      const std::vector<AL::Math::Pose2D>&  pFixesBoxe,
      const std::vector<AL::Math::Pose2D>&  pMovingBoxe,
      AL::Math::Pose2D&                     pMove)
    {
      // the dichotomie number of iteration = precision
      unsigned int nbIteration = 5;

      float min = 0.0f;
      float max = pMove.theta;
      float middle = 0.0f;

      std::vector<AL::Math::Pose2D> tmpMovingBoxe;

      for(unsigned int i=0; i<nbIteration; i++)
      {
        middle = (min + max)/2.0f;
        pMove.theta = middle;
        // compute nex boxe position
        tmpMovingBoxe = xComputeBoxe(pMovingBoxe, pMove);
        // test collision
        if( xIsTwoBoxesAreInCollision(pFixesBoxe, tmpMovingBoxe) )
          max = middle;
        else
          min = middle;
      }
      pMove.theta = (min + max)/2.0f;
    } // end xDichotomie()

  } // namespace Math

} // namespace AL


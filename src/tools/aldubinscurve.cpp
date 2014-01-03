/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/aldubinscurve.h>

#include <almath/types/alposition2d.h>
#include <almath/tools/altrigonometry.h>

#include "float.h" // for FLT_MAX
#include <stdexcept>

#include <cmath>

namespace AL {
  namespace Math {

    struct dubinsTangent
    {
      float x, y;
      bool isLeft;

      dubinsTangent(
          const float pX      = 0.0f,
          const float pY      = 0.0f,
          const bool  pIsLeft = false) :
          x(pX),
          y(pY),
          isLeft(pIsLeft) {}
    };


    /// <summary> Calculates the best tangent. </summary>
    /// <param name="pTangents">    The tangent. </param>
    /// <param name="pbestTangent"> The best tangent. </param>
    void xComputeBestTangent(
        const std::vector<std::vector<dubinsTangent> >& pTangents,
        std::vector<dubinsTangent>&                     pBestTangent);


    /// <summary> Calculates the tangent. </summary>
    /// <param name="pCircle1">       The first circle. </param>
    /// <param name="pCircle2">       The second circle. </param>
    /// <param name="pSens">          The sens. </param>
    /// <param name="pCircleRadius">  The circle radius. </param>
    /// <param name="pLLorRR">        is LSL or RSR?. </param>
    /// <param name="pTangent">       The calculated tangent. </param>
    void xComputeTangent(
        const AL::Math::Position2D& pCircle1,
        const AL::Math::Position2D& pCircle2,
        const int&                  pSens,
        const float&                pCircleRadius,
        const bool&                 pLLorRR,
        std::vector<dubinsTangent>& pTangent);


    /// <summary> Gets the tangents. </summary>
    /// <param name="pCircles">       The circles. </param>
    /// <param name="pCircleRadius">  The circle radius. </param>
    /// <param name="pCircleRadius">  The tangents. </param>
    void xGetTangents(
        const std::vector<AL::Math::Position2D>&  pCircles,
        const float&                              pCircleRadius,
        std::vector<std::vector<dubinsTangent> >& pTangents);


    /// <summary> Calculates the circles. </summary>
    /// <param name="pPose">         The desired pose. </param>
    /// <param name="pCircleRadius"> The circle radius. </param>
    /// <param name="pCircles">      The calculated circles. </param>
    void xGetCircles(
        const AL::Math::Pose2D&            pPose,
        const float&                       pCircleRadius,
        std::vector<AL::Math::Position2D>& pCircles);

    void xComputeBestTangent(
        const std::vector<std::vector<dubinsTangent> >& pTangents,
        std::vector<dubinsTangent>&                     pBestTangent)
    {

      if (pTangents.size() != 4u)
      {
        throw std::invalid_argument("ALDubinsCurve: computeBestTangent Input Tangent must be size 4.");
      }

      if (pBestTangent.size() != 2u)
      {
        throw std::invalid_argument(
            "ALDubinsCurve: computeBestTangent "
            "Input Best Tangent must be size 2.");
      }

      float shortestTangent = FLT_MAX;
      float tangentLength;

      for (unsigned int i=0; i<4; ++i)
      {
        tangentLength = (
            (pTangents.at(i).at(0).x - pTangents.at(i).at(1).x) *
            (pTangents.at(i).at(0).x - pTangents.at(i).at(1).x) +
            (pTangents.at(i).at(0).y - pTangents.at(i).at(1).y) *
            (pTangents.at(i).at(0).y - pTangents.at(i).at(1).y) );

        if (tangentLength < shortestTangent)
        {
          shortestTangent = tangentLength;
          pBestTangent = pTangents.at(i);
        }
      }
    }


    void xComputeTangent(
        const AL::Math::Position2D& pCircle1,
        const AL::Math::Position2D& pCircle2,
        const int&                  pSens,
        const float&                pCircleRadius,
        const bool&                 pLLorRR,
        std::vector<dubinsTangent>& pTangent)
    {

      if (pTangent.size() != 2u)
      {
        throw std::invalid_argument(
            "ALDubinsCurve: computeTangent Input must be size 2.");
      }

      // distance between two center of circle
      const float dist =
          std::sqrt((pCircle1.x - pCircle2.x) * (pCircle1.x - pCircle2.x) +
                 (pCircle1.y - pCircle2.y) * (pCircle1.y - pCircle2.y) );

      const float rd = pCircleRadius / dist;
      float cos_theta = 0.0f;
      float sin_theta = 1.0f;
      if (!pLLorRR)
      {
        cos_theta = pCircleRadius*2.0f/dist;
        sin_theta = std::sqrt(1.0f - cos_theta*cos_theta);
      }

      AL::Math::Position2D slope;
      slope.x = (        cos_theta*(pCircle2.x - pCircle1.x) +
                         pSens*sin_theta*(pCircle2.y - pCircle1.y) );

      slope.y = ( -pSens*sin_theta*(pCircle2.x - pCircle1.x) +
                  cos_theta*(pCircle2.y - pCircle1.y) );

      dubinsTangent point1;
      dubinsTangent point2;

      point1.x = pCircle1.x + rd*slope.x;
      point1.y = pCircle1.y + rd*slope.y;

      if (pLLorRR)
      {
        point2.x = pCircle2.x + rd*slope.x;
        point2.y = pCircle2.y + rd*slope.y;
      }
      else
      {
        point2.x = pCircle2.x - rd*slope.x;
        point2.y = pCircle2.y - rd*slope.y;
      }

      // tangent
      pTangent[0] = point1;
      pTangent[1] = point2;

    } // end computeTangent


    void xGetTangents(
        const std::vector<AL::Math::Position2D>&   pCircles,
        const float&                               pCircleRadius,
        std::vector< std::vector<dubinsTangent> >& pTangents)
    {

      if (pTangents.size() != 4u)
      {
        throw std::invalid_argument(
            "ALDubinsCurve: getTangents Input must be size 4.");
      }

      /**
      * LSL
      */
      xComputeTangent(
          pCircles.at(0),   // pCircle1
          pCircles.at(2),   // pCircle2
          1,                // pSens
          pCircleRadius,    // pCircleRadius
          true,             // pLLorRR
          pTangents.at(0));
      pTangents.at(0).at(0).isLeft = true;
      pTangents.at(0).at(1).isLeft = true;

      /**
      * LSR
      */
      xComputeTangent(
          pCircles.at(0),  // pCircle1
          pCircles.at(3),  // pCircle2
          1,               // pSens
          pCircleRadius,   // pCircleRadius
          false,           // pLLorRR
          pTangents.at(1));
      pTangents.at(1).at(0).isLeft = true;
      pTangents.at(1).at(1).isLeft = false;

      /**
      * RSL
      */
      xComputeTangent(
          pCircles.at(1),  // pCircle1
          pCircles.at(2),  // pCircle2
          -1,              // pSens
          pCircleRadius,   // pCircleRadius
          false,           // pLLorRR
          pTangents.at(2));
      pTangents.at(2).at(0).isLeft = false;
      pTangents.at(2).at(1).isLeft = true;

      /**
      * RSR
      */
      xComputeTangent(
          pCircles.at(1),  // pCircle1
          pCircles.at(3),  // pCircle2
          -1,              // pSens
          pCircleRadius,   // pCircleRadius
          true,            // pLLorRR
          pTangents.at(3));
      pTangents.at(3).at(0).isLeft = false;
      pTangents.at(3).at(1).isLeft = false;

    } // end getTangents


    void xGetCircles(
        const AL::Math::Pose2D&            pPose,
        const float&                       pCircleRadius,
        std::vector<AL::Math::Position2D>& pCircles)
    {

      if (pCircles.size() != 4u)
      {
        throw std::invalid_argument(
            "ALDubinsCurve: getCircles Input must be size 4.");
      }

      AL::Math::Position2D tmpCircle;

      // Left Circle - init
      tmpCircle.x = 0.0f;
      tmpCircle.y = pCircleRadius;
      pCircles.at(0) = tmpCircle;

      // Right Circle - init
      tmpCircle.x = 0.0f;
      tmpCircle.y = -pCircleRadius;
      pCircles.at(1) = tmpCircle;

      // Left Circle - Desired
      tmpCircle.x = pPose.x - ( sin(pPose.theta)*pCircleRadius );
      tmpCircle.y = pPose.y + ( cos(pPose.theta)*pCircleRadius );
      pCircles.at(2) = tmpCircle;

      // Right Circle - Desired
      tmpCircle.x = pPose.x + ( sin(pPose.theta)*pCircleRadius );
      tmpCircle.y = pPose.y - ( cos(pPose.theta)*pCircleRadius );
      pCircles.at(3) = tmpCircle;
    } // end getCircles


    std::vector<AL::Math::Pose2D> getDubinsSolutions(
        const AL::Math::Pose2D& pTargetPose,
        const float             pCircleRadius)
    {
      // protection around small distance
      // in relation with circleRadius
      float dist = sqrt(pTargetPose.x*pTargetPose.x +
                        pTargetPose.y*pTargetPose.y );
      if(dist < 4.0f*pCircleRadius)
      {
        throw std::invalid_argument(
            "ALDubinsCurve: getDubinsSolutions pTargetPose.norm() < 4.0*pCircleRadius.");
      }

      std::vector<AL::Math::Pose2D> solutions;
      AL::Math::Pose2D tmpSolution;

      std::vector<AL::Math::Position2D> circles;
      circles.resize(4);
      xGetCircles(pTargetPose, pCircleRadius, circles);

      std::vector<dubinsTangent> tmpDubins;
      tmpDubins.resize(2);
      std::vector<std::vector<dubinsTangent> > tangents;
      tangents.resize(4);
      for (unsigned int i=0; i<4; ++i)
      {
        tangents.at(i) = tmpDubins;
      }
      xGetTangents(circles, pCircleRadius, tangents);

      std::vector<dubinsTangent> bestTangent;
      bestTangent.resize(2);

      xComputeBestTangent(tangents, bestTangent);

      //// First CheckPoint of this Dubins Curve
      tmpSolution.x = bestTangent.at(0).x;
      tmpSolution.y = bestTangent.at(0).y;
      tmpSolution.theta = std::atan2(bestTangent.at(1).y - bestTangent.at(0).y,
                                     bestTangent.at(1).x - bestTangent.at(0).x);
      solutions.push_back(tmpSolution);

      //// Second CheckPoint of this Dubins Curve
      tmpSolution.x = bestTangent.at(1).x;
      tmpSolution.y = bestTangent.at(1).y;
      // theta is equivalent in first and second checkPoint
      solutions.push_back(tmpSolution);

      /// Last CheckPoint is targetPose
      solutions.push_back(pTargetPose);

      /**********************************
      Check Solution (angle rotation)
      *********************************/
      AL::Math::Pose2D newSolutions;
      AL::Math::Pose2D newSolutionsLast;
      bool findNewSolutionFirst = false;
      bool findNewSolutionLast  = false;

      // first test is angle of rotation find with atan2 is in the good sens
      // first tangent
      float angle1 = solutions.at(0).theta;
      if (bestTangent.at(0).isLeft && angle1 < 0.0f)
      {
        solutions.at(0).theta = 2.0f * PI + angle1;
      }
      if (!bestTangent.at(0).isLeft && angle1 > 0.0f)
      {
        solutions.at(0).theta = 2.0f * PI + angle1;
      }
      // second tangent
      float angle2 = solutions.at(2).theta - solutions.at(1).theta;
      if (bestTangent.at(1).isLeft && angle2 < 0.0f)
      {
        solutions.at(1).theta = 2.0f * PI - angle2 + solutions.at(2).theta;
      }
      if (!bestTangent.at(1).isLeft && angle2 > 0.0f)
      {
        solutions.at(1).theta = 2.0f * PI - angle2 + solutions.at(2).theta;
      }
      return solutions;
    } // end getDubinsSolutions
  }
}

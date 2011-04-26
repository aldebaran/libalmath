/**
* @author David Gouaillier and chris Kilner and Cyrille Collette
* Copyright (c) Aldebaran Robotics 2010 All Rights Reserved
*/

#include <almath/tools/aldubinscurve.h>

#include "float.h" // for FLT_MAX

#include <alcore/alerror.h>


namespace AL {
  namespace Math {

    void computeBestTangent(
      const std::vector<std::vector<dubinsTangent> >& pTangents,
      std::vector<dubinsTangent>&                     pBestTangent)
    {

      if (pTangents.size() != 4)
      {
        throw ALERROR("ALDubinsCurve",
          "computeBestTangent",
          "Input Tangent must be size 4.");
      }

      if (pBestTangent.size() != 2)
      {
        throw ALERROR(
          "ALDubinsCurve",
          "computeBestTangent",
          "Input Best Tangent must be size 2.");
      }

      float shortestTangent = FLT_MAX;
      float tangentLength;

      for (unsigned int i=0; i<4; i++)
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


    void computeTangent(
      const AL::Math::Position2D& pCircle1,
      const AL::Math::Position2D& pCircle2,
      const int&                  pSens,
      const float&                pCircleRadius,
      const bool&                 pLLorRR,
      std::vector<dubinsTangent>& pTangent)
    {

      if (pTangent.size() != 2)
      {
        throw ALERROR(
          "ALDubinsCurve",
          "computeTangent",
          "Input must be size 2.");
      }

      float dist; // distance between two center of circle
      float rd;
      float cos_theta;
      float sin_theta;
      AL::Math::Position2D slope;
      dubinsTangent point1;
      dubinsTangent point2;

      dist = sqrtf( (pCircle1.x - pCircle2.x) * (pCircle1.x - pCircle2.x) +
                    (pCircle1.y - pCircle2.y) * (pCircle1.y - pCircle2.y) );
      rd = pCircleRadius / dist;
      if (pLLorRR)
      {
        cos_theta = 0.0f;
        sin_theta = 1.0f;
      }
      else
      {
        cos_theta = pCircleRadius*2.0f/dist;
        sin_theta = sqrtf( 1 - cos_theta*cos_theta);
      }

      slope.x = (        cos_theta*(pCircle2.x - pCircle1.x) +
                         pSens*sin_theta*(pCircle2.y - pCircle1.y) );

      slope.y = ( -pSens*sin_theta*(pCircle2.x - pCircle1.x) +
                  cos_theta*(pCircle2.y - pCircle1.y) );

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
      pTangent.at(0) = point1;
      pTangent.at(1) = point2;

    } // end computeTangent


    void getTangents(
      const std::vector<AL::Math::Position2D>&   pCircles,
      const float&                               pCircleRadius,
      std::vector< std::vector<dubinsTangent> >& pTangents)
    {

      if (pTangents.size() != 4)
      {
        throw ALERROR(
          "ALDubinsCurve",
          "getTangents",
          "Input must be size 4.");
      }

      /**
      * LSL
      */
      computeTangent(
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
      computeTangent(
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
      computeTangent(
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
      computeTangent(
        pCircles.at(1),  // pCircle1
        pCircles.at(3),  // pCircle2
        -1,              // pSens
        pCircleRadius,   // pCircleRadius
        true,            // pLLorRR
        pTangents.at(3));
      pTangents.at(3).at(0).isLeft = false;
      pTangents.at(3).at(1).isLeft = false;

    } // end getTangents


    void getCircles(
      const AL::Math::Pose2D&            pPose,
      const float&                       pCircleRadius,
      std::vector<AL::Math::Position2D>& pCircles)
    {

      if (pCircles.size() != 4)
      {
        throw ALERROR(
          "ALDubinsCurve",
          "getCircles",
          "Input must be size 4.");
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
      std::vector<AL::Math::Pose2D> solutions;
      AL::Math::Pose2D tmpSolution;

      std::vector<AL::Math::Position2D> circles;
      circles.resize(4);
      getCircles(pTargetPose, pCircleRadius, circles);

      std::vector<dubinsTangent> tmpDubins;
      tmpDubins.resize(2);
      std::vector<std::vector<dubinsTangent> > tangents;
      tangents.resize(4);
      for (unsigned int i=0; i<4; i++)
      {
        tangents.at(i) = tmpDubins;
      }
      getTangents(circles, pCircleRadius, tangents);

      std::vector<dubinsTangent> bestTangent;
      bestTangent.resize(2);

      computeBestTangent(tangents, bestTangent);

      //// First CheckPoint of this Dubins Curve
      tmpSolution.x = bestTangent.at(0).x;
      tmpSolution.y = bestTangent.at(0).y;
      tmpSolution.theta = atan2(bestTangent.at(1).y - bestTangent.at(0).y,
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
      // then look at this angle and check if >pi
      angle1 = solutions.at(0).theta;
      if (fabsf(angle1) >= PI)
      {
        // if more than pi, find new tangent in the middle
        if(bestTangent.at(0).isLeft)
        {
          findNewSolutionFirst = true;
          newSolutions.x = circles.at(0).x + pCircleRadius*sinf(angle1*0.5f);
          newSolutions.y = circles.at(0).y - pCircleRadius*cosf(angle1*0.5f);
          newSolutions.theta = angle1*0.5f;
        }
        else
        {
          findNewSolutionFirst = true;
          newSolutions.x = circles.at(1).x + pCircleRadius*sinf(angle1*0.5f);
          newSolutions.y = circles.at(1).y - pCircleRadius*cosf(angle1*0.5f);
          newSolutions.theta = angle1*0.5f;
        }
      }

      // the same for the second tangent
      angle2 = solutions.at(2).theta - solutions.at(1).theta;
      if ( fabsf(angle2) >= PI)
      {
        if(bestTangent.at(1).isLeft)
        {
          findNewSolutionLast = true;
          newSolutionsLast.x = circles.at(2).x +
                               pCircleRadius*sinf(angle2*0.5f + solutions.at(1).theta);
          newSolutionsLast.y = circles.at(2).y -
                               pCircleRadius*cosf(angle2*0.5f + solutions.at(1).theta);
          newSolutionsLast.theta = angle2*0.5f+solutions.at(1).theta;
        }
        else
        {
          findNewSolutionLast = true;
          newSolutionsLast.x = circles.at(3).x +
                               pCircleRadius*sinf(angle2*0.5f + solutions.at(1).theta);
          newSolutionsLast.y = circles.at(3).y -
                               pCircleRadius*cosf(angle2*0.5f + solutions.at(1).theta);
          newSolutionsLast.theta = angle2*0.5f + solutions.at(1).theta;
        }
      }

      // incorporate newSolution in first Dubins Solution
      if (findNewSolutionFirst)
      {
        solutions.insert(solutions.begin(), newSolutions);
      }
      if (findNewSolutionLast)
      {
        solutions.at(solutions.size()-1) = newSolutionsLast;
        solutions.push_back(pTargetPose);
      }

      return solutions;
    } // end getDubinsSolutions

  }
}

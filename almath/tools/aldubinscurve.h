/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALDUBINSCURVE_H_
#define _LIB_ALMATH_ALMATH_ALDUBINSCURVE_H_

#include <almath/types/alposition2d.h>
#include <almath/types/alpose2d.h>
#include <almath/tools/altrigonometry.h>

#include <vector>

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
    void computeBestTangent(
      const std::vector<std::vector<dubinsTangent> >& pTangents,
      std::vector<dubinsTangent>&                     pBestTangent);


    /// <summary> Calculates the tangent. </summary>
    /// <param name="pCircle1">       The first circle. </param>
    /// <param name="pCircle2">       The second circle. </param>
    /// <param name="pSens">          The sens. </param>
    /// <param name="pCircleRadius">  The circle radius. </param>
    /// <param name="pLLorRR">        is LSL or RSR?. </param>
    /// <param name="pTangent">       The calculated tangent. </param>
    void computeTangent(
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
    void getTangents(
      const std::vector<AL::Math::Position2D>&  pCircles,
      const float&                              pCircleRadius,
      std::vector<std::vector<dubinsTangent> >& pTangents);


    /// <summary> Calculates the circles. </summary>
    /// <param name="pPose">         The desired pose. </param>
    /// <param name="pCircleRadius"> The circle radius. </param>
    /// <param name="pCircles">      The calculated circles. </param>
    void getCircles(
      const AL::Math::Pose2D&            pPose,
      const float&                       pCircleRadius,
      std::vector<AL::Math::Position2D>& pCircles);


    /// <summary> Get the dubins solutions. </summary>
    /// <param name="pTargetPose">   The target pose. </param>
    /// <param name="pCircleRadius"> The circle radius. </param>
    /// <return> The dubins solution. </return>
    std::vector<AL::Math::Pose2D> getDubinsSolutions(
      const AL::Math::Pose2D& pTargetPose,
      const float             pCircleRadius);

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALDUBINSCURVE_H_

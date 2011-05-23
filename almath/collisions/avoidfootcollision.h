/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_AVOIDFOOTCOLLISION_H_
#define _LIB_ALMATH_ALMATH_AVOIDFOOTCOLLISION_H_

#include <almath/types/alpose2d.h>
#include <vector>

namespace AL {
  namespace Math {
    // <summary> Compute the best position(orientation) of the foot to avoid collision. </summary>
    // <param name="pLFootBoundingBoxe"> vector<Pose2D> of the left footBoundingBoxe. </param>
    // <param name="pRFootBoundingBoxe"> vector<Pose2D> of the right footBoundingBoxe. </param>
    // <param name="pMove">              Pose2D the initial move. </param>
    // <param name="pIsLeftSupport">     Bool true if left is the support leg. </param>
    // <returns> Pose2D, the best position without collsion. </returns>
    const bool avoidFootCollision(
      const std::vector<AL::Math::Pose2D>&  pLFootBoundingBoxe,
      const std::vector<AL::Math::Pose2D>&  pRFootBoundingBoxe,
      const bool&                           pIsLeftSupport,
      AL::Math::Pose2D&                     pMove );

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
  } // namespace Math

} // namespace AL

#endif  // _LIB_ALMATH_ALMATH_AVOIDFOOTCOLLISION_H_


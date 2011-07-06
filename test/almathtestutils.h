/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2010 All Rights Reserved
 *
 */

#pragma once

#ifndef _LIB_ALMATH_TEST_ALMATHTESTUTILS_H_
#define _LIB_ALMATH_TEST_ALMATHTESTUTILS_H_

#include <gtest/gtest.h>
#include <almath/tools/almath.h>
#include <stdexcept>

static const float kEpsilon = 1.0e-4f;

void validateFindRotation(
    const AL::Math::Position3D& pA,
    const AL::Math::Position3D& pB);

void comparePose2D(
  const AL::Math::Pose2D& pPos1,
  const AL::Math::Pose2D& pPos2,
  const float pEps = kEpsilon);


void compareTransform(
             const AL::Math::Transform& pH1,
             const AL::Math::Transform& pH2,
             const float pEps = kEpsilon);


void comparePosition2D(
  const AL::Math::Position2D& pPos1,
  const AL::Math::Position2D& pPos2,
  const float pEps = kEpsilon);


void comparePosition3D(
  const AL::Math::Position3D& pPos1,
  const AL::Math::Position3D& pPos2,
  const float pEps = kEpsilon);


void comparePosition6D(
  const AL::Math::Position6D& pPos1,
  const AL::Math::Position6D& pPos2,
  const float pEps = kEpsilon);


void compareVelocity3D(
  const AL::Math::Velocity3D& pVel1,
  const AL::Math::Velocity3D& pVel2,
  const float pEps = kEpsilon);


void compareVelocity6D(
  const AL::Math::Velocity6D& pVel1,
  const AL::Math::Velocity6D& pVel2,
  const float pEps = kEpsilon);


void compareRotation(
  const AL::Math::Rotation& pRot1,
  const AL::Math::Rotation& pRot2,
  const float pEps = kEpsilon);


void compareRotation3D(
  const AL::Math::Rotation3D& pRot1,
  const AL::Math::Rotation3D& pRot2,
  const float pEps = kEpsilon);


void comparePositionAndVelocity(
  const AL::Math::PositionAndVelocity& pPosVel1,
  const AL::Math::PositionAndVelocity& pPosVel2,
  const float pEps = kEpsilon);

void compareTransformAndVelocity6D(
  const AL::Math::TransformAndVelocity6D& pPosVel1,
  const AL::Math::TransformAndVelocity6D& pPosVel2,
  const float pEps = kEpsilon);

#endif  // _LIB_ALMATH_TEST_ALMATHTESTUTILS_H_

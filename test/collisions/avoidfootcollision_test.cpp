/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/avoidfootcollision.h>
#include <almath/types/alpose2d.h>
#include <almath/tools/altrigonometry.h>

#include <gtest/gtest.h>

TEST(avoidFootCollisionTest, t0_avoidFootCollision)
{

  AL::Math::Position2D pRFootFL = AL::Math::Position2D( 0.080f,  0.038f);
  AL::Math::Position2D pRFootFR = AL::Math::Position2D( 0.080f, -0.050f);
  AL::Math::Position2D pRFootRR = AL::Math::Position2D(-0.047f, -0.050f);
  AL::Math::Position2D pRFootRL = AL::Math::Position2D(-0.047f,  0.038f);

  AL::Math::Position2D pLFootFL = AL::Math::Position2D( 0.080f,  0.050f);
  AL::Math::Position2D pLFootFR = AL::Math::Position2D( 0.080f, -0.038f);
  AL::Math::Position2D pLFootRR = AL::Math::Position2D(-0.047f, -0.038f);
  AL::Math::Position2D pLFootRL = AL::Math::Position2D(-0.047f,  0.050f);

  std::vector<AL::Math::Position2D> pRFootBoundingBox;
  pRFootBoundingBox.push_back(pRFootFL);
  pRFootBoundingBox.push_back(pRFootFR);
  pRFootBoundingBox.push_back(pRFootRR);
  pRFootBoundingBox.push_back(pRFootRL);

  std::vector<AL::Math::Position2D> pLFootBoundingBox;
  pLFootBoundingBox.push_back(pLFootFL);
  pLFootBoundingBox.push_back(pLFootFR);
  pLFootBoundingBox.push_back(pLFootRR);
  pLFootBoundingBox.push_back(pLFootRL);

  AL::Math::Pose2D pMove;
  bool pResult;

  //std::cout << "***** avoidFootCollision *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.0f, 0.1f, 0.0f*AL::Math::TO_RAD);
  pResult = avoidFootCollision( pLFootBoundingBox,
                                pRFootBoundingBox,
                                false,
                                pMove);
  EXPECT_FALSE(pResult);
  EXPECT_NEAR(pMove.x, 0.0f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.1f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.0f, 0.0001f );

  //std::cout << "***** avoidFootCollision *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.0f, 0.085f, 40.0f*AL::Math::TO_RAD);
  pResult = avoidFootCollision( pLFootBoundingBox,
                                pRFootBoundingBox,
                                false,
                                pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, 0.0f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.085f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.196349f, 0.0001f );
}


TEST(avoidFootCollisionTest, t1_clipFootWithEllipse)
{
  AL::Math::Pose2D pMove;
  bool pResult;

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.05f, 0.04f, 0.1f);
  pResult = clipFootWithEllipse( 0.08f,
                                 0.06f,
                                 pMove);
  EXPECT_FALSE(pResult);
  EXPECT_NEAR(pMove.x, 0.05f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.1f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( 0.08f, 0.06f, 0.5f);
  pResult = clipFootWithEllipse( 0.08f,
                                 0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, 0.05656f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04242f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( -0.08f, -0.05f, 0.5f);
  pResult = clipFootWithEllipse( 0.08f,
                                 0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, -0.06145f, 0.0001f );
  EXPECT_NEAR(pMove.y, -0.03841f, 0.0001f );
  EXPECT_NEAR(pMove.theta, 0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( -0.04f, 0.06f, -0.5f);
  pResult = clipFootWithEllipse( 0.04f,
                                 0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, -0.02828f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04242f, 0.0001f );
  EXPECT_NEAR(pMove.theta, -0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  pMove = AL::Math::Pose2D( -0.04f, 0.06f, -0.5f);
  pResult = clipFootWithEllipse( -0.04f,
                                 -0.06f,
                                 pMove);
  EXPECT_TRUE(pResult);
  EXPECT_NEAR(pMove.x, -0.02828f, 0.0001f );
  EXPECT_NEAR(pMove.y, 0.04242f, 0.0001f );
  EXPECT_NEAR(pMove.theta, -0.5f, 0.0001f );

  //std::cout << "***** clipFootWithEllipse *****" << std::endl;
  //  {x: +0.00000, y:+0.160000, theta:+0.00000}
  //  {x: +0.00000, y:-0.160000, theta:+0.00000}

  pMove = AL::Math::Pose2D(0.0f, 0.16f, 0.0f);
  pResult = clipFootWithEllipse( 0.06f,
                                 0.16f,
                                 pMove);
  EXPECT_FALSE(pResult);
}


TEST(avoidFootCollisionTest, t2_intersectionSegment2D)
{
  const AL::Math::Position2D pA1 = AL::Math::Position2D( 0.0f, -1.0f);
  const AL::Math::Position2D pA2 = AL::Math::Position2D( 0.0f,  1.0f);
  AL::Math::Position2D pB1 = AL::Math::Position2D(-1.0f,  0.0f);
  AL::Math::Position2D pB2 = AL::Math::Position2D( 1.0f,  0.0f);
  AL::Math::Position2D pC;
  AL::Math::Position2D expectedSol;

  bool inter = intersectionSegment2D(pA1, pA2, pB1, pB2, pC);
  ASSERT_TRUE(inter);
  ASSERT_NEAR(pC.distanceSquared(expectedSol), 0.0f, 1e-5f);

  pB1.y = 1.0f;
  pB2.y = 1.0f;
  expectedSol.y = 1.0f;
  inter = intersectionSegment2D(pA1, pA2, pB1, pB2, pC);
  ASSERT_TRUE(inter);
  ASSERT_NEAR(pC.distanceSquared(expectedSol), 0.0f, 1e-5f);

  pB1.y = 1.0001f;
  pB2.y = 1.0001f;
  inter = intersectionSegment2D(pA1, pA2, pB1, pB2, pC);
  ASSERT_FALSE(inter);

  pB1.y = -1.0f;
  pB2.y = -1.0f;
  expectedSol.y = -1.0f;
  inter = intersectionSegment2D(pA1, pA2, pB1, pB2, pC);
  ASSERT_TRUE(inter);
  ASSERT_NEAR(pC.distanceSquared(expectedSol), 0.0f, 1e-5f);

  pB1.y = -1.0001f;
  pB2.y = -1.0001f;
  inter = intersectionSegment2D(pA1, pA2, pB1, pB2, pC);
  ASSERT_FALSE(inter);

  pB1 = AL::Math::Position2D(0.5f, 0.5f);
  pB1 = AL::Math::Position2D(1.5f, 0.5f);
  inter = intersectionSegment2D(pA1, pA2, pB1, pB2, pC);
  ASSERT_FALSE(inter);

  // Colinear intersections return false
  pB1 = AL::Math::Position2D(0.5f, 0.0f);
  pB1 = AL::Math::Position2D(1.5f, 0.0f);
  inter = intersectionSegment2D(pA1, pA2, pB1, pB2, pC);
  ASSERT_FALSE(inter);
}

// Useful to find intersections circumference-line
bool getCircleSideIntersection(
    float pConstantDim, float pR2, float &pVarDimRelativeCenter)
{
  const float aux = pR2 - std::pow(pConstantDim, 2.0f);
  if (aux > 0.0f)
  {
    pVarDimRelativeCenter = sqrt(aux);
    return true;
  }
  return false;
}

// Check if the point is in the area limited by a square placed at the origin
// If positive, adds the angle of the intersection
void checkAndAddPoint(
    float px, float py, float pSide, std::vector<float> & pCollisionAngles)
{
  const float lTol = 1e-5f;
  const float sideTol = pSide + lTol;
  if (std::abs(px) < sideTol && std::abs(py) < sideTol)
  {
    pCollisionAngles.push_back(std::atan2(px, py));
  }
}

// Particular solution for square-shaped bounding boxes
// Returns closer angle to zero that we have to rotate a square
// so that it intersects another.
float checkCollisionAngle(const AL::Math::Pose2D& pRelativePose)
{
  float resultTheta = 0.0f;
  const float side =  1.0f;
  const float diag2 = 2.0f*side;

  const float xC = pRelativePose.x;
  const float yC = pRelativePose.y;

  //Squares too close, always touch
  if (std::abs(xC) <= 2.0*side && std::abs(yC) <= 2.0*side)
  {
    return resultTheta;
  }
  else
  {
    std::vector<float> intersectionAngles;
    //Collision of sides x=+-1.0 with (x - xC)^2 + (y - yC)^2 = diag^2
    float x = side;
    float y = 0.0f;
    //Interseciton with fix x sides
    if (getCircleSideIntersection(x - xC,diag2, y))
    {
      checkAndAddPoint(x,  y + yC, side, intersectionAngles);
      checkAndAddPoint(x, -y + yC, side, intersectionAngles);
    }
    x = -side;
    if (getCircleSideIntersection(x - xC, diag2, y))
    {
      checkAndAddPoint(x,  y + yC, side, intersectionAngles);
      checkAndAddPoint(x, -y + yC , side, intersectionAngles);
    }
    y = side;
    if (getCircleSideIntersection(y - yC, diag2, x))
    {
      checkAndAddPoint( x + xC, y, side, intersectionAngles);
      checkAndAddPoint(-x + xC, y, side, intersectionAngles);
    }
    y = -side;
    if (getCircleSideIntersection(y - yC, diag2, x))
    {
      checkAndAddPoint( x + xC, y, side, intersectionAngles);
      checkAndAddPoint(-x + xC, y, side, intersectionAngles);
    }
    resultTheta = pRelativePose.theta;
    const float lTol = 1e-5f;
    if (resultTheta < lTol)
    {
      return resultTheta;
    }
    //Four corners of a square
    std::vector<float>cornerAngles(4u);
    cornerAngles[0] = AL::Math::TO_RAD *   45.0f;
    cornerAngles[1] = AL::Math::TO_RAD *  135.0f;
    cornerAngles[2] = AL::Math::TO_RAD * - 45.0f;
    cornerAngles[3] = AL::Math::TO_RAD * -135.0f;
    for (unsigned int i = 0; i < intersectionAngles.size(); ++i)
    {
      for (unsigned int j = 0; j < cornerAngles.size(); ++j)
      {
        //Rotation necessary to touch the square with this corner
        const float thetaCandidate = intersectionAngles[i] - cornerAngles[j];
        if (thetaCandidate < lTol)
        {
          return thetaCandidate;
        }
        float proportion = thetaCandidate / resultTheta;
        if (proportion < 1.0f && proportion > 0.0f)
        {
          resultTheta = thetaCandidate;
        }
      }
    }
  }
  return resultTheta;
}

void checkDichotomie(
    const std::vector<AL::Math::Position2D>  &pBoundingBoxPose2D,
    AL::Math::Pose2D& pFootstepMove)
{
  const float lTol = 0.1f;
  const float origAngle = pFootstepMove.theta;
  const float exactAngle = checkCollisionAngle(pFootstepMove);
  dichotomie(pBoundingBoxPose2D, pBoundingBoxPose2D, pFootstepMove);
  ASSERT_NEAR(pFootstepMove.theta, exactAngle,lTol);
  if (origAngle > 0.0f)
  {
    ASSERT_LE(pFootstepMove.theta, exactAngle);
  }
  else
  {
    ASSERT_GE(pFootstepMove.theta, exactAngle);
  }
}

TEST(avoidFootCollisionTest, t3_dichotomie)
{
  std::vector<AL::Math::Position2D>  boundingBoxPose2D(4u);
  boundingBoxPose2D[0] = AL::Math::Position2D( 1.0f,  1.0f);
  boundingBoxPose2D[1] = AL::Math::Position2D( 1.0f, -1.0f);
  boundingBoxPose2D[2] = AL::Math::Position2D(-1.0f, -1.0f);
  boundingBoxPose2D[3] = AL::Math::Position2D(-1.0f,  1.0f);

  std::vector<float> linDists;
  linDists.reserve(9u);
  linDists.push_back(0.0f);
  linDists.push_back(0.5f);
  linDists.push_back(1.0f);
  linDists.push_back(1.5f);
  linDists.push_back(3.0f);
  const size_t linDistsPSize = linDists.size();
  for (size_t i = 1u; i < linDistsPSize; ++i)
  {
    linDists.push_back(-linDists[i]);
  }

  std::vector<float> thDists;
  thDists.reserve(13u);
  thDists.push_back(0.0f);
  thDists.push_back(0.2f);
  thDists.push_back(0.6f);
  thDists.push_back(1.0f);
  thDists.push_back(1.5f);
  thDists.push_back(2.0f);
  thDists.push_back(3.0f);
  const size_t thDistsPSize = thDists.size();
  for (size_t i = 1u; i < thDistsPSize; ++i)
  {
    thDists.push_back(-thDists[i]);
  }

  std::vector<AL::Math::Pose2D> footstepMoves;
  for (size_t xi = 0u; xi < linDists.size(); ++xi)
  {
    for (size_t yi = 0u; yi < linDists.size(); ++yi)
    {
      for (size_t thi = 0u; thi < linDists.size(); ++thi)
      {
        footstepMoves.push_back(AL::Math::Pose2D(linDists[xi], linDists[yi], thDists[thi]));
      }
    }
  }

  const AL::Math::Pose2D origin = AL::Math::Pose2D();
  for (unsigned int i = 0u; i < footstepMoves.size(); ++i)
  {
    checkDichotomie(boundingBoxPose2D, footstepMoves[i]);
  }
}

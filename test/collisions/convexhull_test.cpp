/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/collisions/convexhull.h>
#include "../almathtestutils.h"


TEST(removeAlignedPoint, remove0)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);
  AL::Math::Position2D pA0 = AL::Math::Position2D(-1.0f, -1.0f);
  AL::Math::Position2D pA1 = AL::Math::Position2D(-1.0f, -0.5f);
  AL::Math::Position2D pA2 = AL::Math::Position2D(-1.0f, 0.5f);
  AL::Math::Position2D pA3 = AL::Math::Position2D(-1.0f, 1.0f);
  AL::Math::Position2D pB0 = AL::Math::Position2D(1.0f, -1.0f);
  AL::Math::Position2D pB1 = AL::Math::Position2D(1.0f, -0.5f);
  AL::Math::Position2D pB2 = AL::Math::Position2D(1.0f,  0.5f);
  AL::Math::Position2D pB3 = AL::Math::Position2D(1.0f, 1.0f);

  lContactPoints.at(0) = pA0;
  lContactPoints.at(1) = pA1;
  lContactPoints.at(2) = pA2;
  lContactPoints.at(3) = pA3;
  lContactPoints.at(4) = pB0;
  lContactPoints.at(5) = pB1;
  lContactPoints.at(6) = pB2;
  lContactPoints.at(7) = pB3;

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());

  lContactPoints.at(0) = pB3;
  lContactPoints.at(1) = pA0;
  lContactPoints.at(2) = pB2;
  lContactPoints.at(3) = pA1;
  lContactPoints.at(4) = pB1;
  lContactPoints.at(5) = pA2;
  lContactPoints.at(6) = pB0;
  lContactPoints.at(7) = pA3;

  result.clear();
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());
}

TEST(removeAlignedPoint, remove1)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  std::vector<AL::Math::Position2D> result;

  lContactPoints.resize(8);
  lContactPoints.at(0) = AL::Math::Position2D(0.0809578f, 0.1487f);
  lContactPoints.at(1) = AL::Math::Position2D(0.081005f, 0.0607021f);
  lContactPoints.at(2) = AL::Math::Position2D(-0.045994f, 0.0606381f);
  lContactPoints.at(3) = AL::Math::Position2D(-0.0460412f, 0.148636f);
  lContactPoints.at(4) = AL::Math::Position2D(0.0803307f, 0.0376083f);
  lContactPoints.at(5) = AL::Math::Position2D(0.0801544f, -0.0503918f);
  lContactPoints.at(6) = AL::Math::Position2D(-0.0468458f, -0.0501374f);
  lContactPoints.at(7) = AL::Math::Position2D(-0.0466695f, 0.0378627f);

  result.clear();
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());
}



TEST(isLeftBest, compare0)
{
  std::vector<AL::Math::Position2D> pt;
  AL::Math::Position2D pA;
  AL::Math::Position2D pB;
  AL::Math::Position2D pC;

  pt.resize(8);
  pt.at(0) = AL::Math::Position2D(0.0809578f, 0.1487f);
  pt.at(1) = AL::Math::Position2D(0.081005f, 0.0607021f);
  pt.at(2) = AL::Math::Position2D(-0.045994f, 0.0606381f);
  pt.at(3) = AL::Math::Position2D(-0.0460412f, 0.148636f);
  pt.at(4) = AL::Math::Position2D(0.0803307f, 0.0376083f);
  pt.at(5) = AL::Math::Position2D(0.0801544f, -0.0503918f);
  pt.at(6) = AL::Math::Position2D(-0.0468458f, -0.0501374f);
  pt.at(7) = AL::Math::Position2D(-0.0466695f, 0.0378627f);

  // -3: points are nearly aligned but pB is not in the middle of pA, pC
  // -2: points are aligned and pB is in the middle of pA, pC

  //  i: 0 j: 1 k: 5
  pA = pt.at(0);
  pB = pt.at(1);
  pC = pt.at(5);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  //  i: 0 j: 4 k: 5
  pA = pt.at(0);
  pB = pt.at(4);
  pC = pt.at(5);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  //  i: 0 j: 5 k: 1
  pA = pt.at(0);
  pB = pt.at(5);
  pC = pt.at(1);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  //  i: 3 j: 2 k: 6
  pA = pt.at(3);
  pB = pt.at(2);
  pC = pt.at(6);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  //  i: 3 j: 6 k: 2
  pA = pt.at(3);
  pB = pt.at(6);
  pC = pt.at(2);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  //  i: 3 j: 7 k: 6
  pA = pt.at(3);
  pB = pt.at(7);
  pC = pt.at(6);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  //  i: 4 j: 0 k: 5
  pA = pt.at(4);
  pB = pt.at(0);
  pC = pt.at(5);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  //  i: 6 j: 3 k: 2
  pA = pt.at(6);
  pB = pt.at(3);
  pC = pt.at(2);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));
}

TEST(isLeftBest, compare1)
{
  // -4: at least two points are equal
  // -3: points are nearly aligned but pB is not in the middle of pA, pC
  // -2: points are aligned and pB is in the middle of pA, pC
  // -1: pC is left
  // +1: pC is right
  AL::Math::Position2D pA;
  AL::Math::Position2D pB;
  AL::Math::Position2D pC;

  // At least, two points are equal
  // test 0-a
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // test 0-b
  pA = AL::Math::Position2D(-1.0f, 1.0f);
  pB = AL::Math::Position2D(-1.0f, 1.0f);
  pC = AL::Math::Position2D(-1.0f, 1.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // test 0-c
  pA = AL::Math::Position2D(1.0f, 1.0f);
  pB = AL::Math::Position2D(1.0f, 1.0f);
  pC = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // test 0-d
  pA = AL::Math::Position2D(1.0f, 1.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // test 0-e
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(1.0f, 1.0f);
  pC = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // points are nearly aligned but pB is not in the middle of pA, pC
  // test 1-a-a
  pA = AL::Math::Position2D(1.0f, 1.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(2.0f, 2.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-a-b
  pA = AL::Math::Position2D(0.0f, 1.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, 2.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-a-c
  pA = AL::Math::Position2D(1.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(2.0f, 0.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-b-a
  pA = AL::Math::Position2D(-1.0f, -1.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(-2.0f, -2.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-b-b
  pA = AL::Math::Position2D(0.0f, -1.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, -2.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-b-c
  pA = AL::Math::Position2D(-1.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(-2.0f, 0.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-c-a
  pA = AL::Math::Position2D(1.0f, 1.0f);
  pB = AL::Math::Position2D(0.1f, 0.1f);
  pC = AL::Math::Position2D(2.0f, 2.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-c-b
  pA = AL::Math::Position2D(0.0f, 1.0f);
  pB = AL::Math::Position2D(0.0f, 0.1f);
  pC = AL::Math::Position2D(0.0f, 2.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-c-c
  pA = AL::Math::Position2D(1.0f, 0.0f);
  pB = AL::Math::Position2D(0.1f, 0.0f);
  pC = AL::Math::Position2D(2.0f, 0.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-d-a
  pA = AL::Math::Position2D(2.0f, 2.0f);
  pB = AL::Math::Position2D(0.1f, 0.1f);
  pC = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-d-b
  pA = AL::Math::Position2D(0.0f, 2.0f);
  pB = AL::Math::Position2D(0.0f, 0.1f);
  pC = AL::Math::Position2D(0.0f, 1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-d-c
  pA = AL::Math::Position2D(2.0f, 0.0f);
  pB = AL::Math::Position2D(0.1f, 0.0f);
  pC = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-e-a
  pA = AL::Math::Position2D(-1.0f, -1.0f);
  pB = AL::Math::Position2D(2.0f, 2.0f);
  pC = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-e-b
  pA = AL::Math::Position2D(0.0f, -1.0f);
  pB = AL::Math::Position2D(0.0f, 2.0f);
  pC = AL::Math::Position2D(0.0f, 1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-e-c
  pA = AL::Math::Position2D(-1.0f, 0.0f);
  pB = AL::Math::Position2D(2.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-f-a
  pA = AL::Math::Position2D(-1.0f, -1.0f);
  pB = AL::Math::Position2D(-2.0f, -2.0f);
  pC = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-f-b
  pA = AL::Math::Position2D(0.0f, -1.0f);
  pB = AL::Math::Position2D(0.0f, -2.0f);
  pC = AL::Math::Position2D(0.0f, 1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-f-c
  pA = AL::Math::Position2D(-1.0f, 0.0f);
  pB = AL::Math::Position2D(-2.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-g-a
  pA = AL::Math::Position2D(1.0f, 1.0f);
  pB = AL::Math::Position2D(-2.0f, -2.0f);
  pC = AL::Math::Position2D(-1.0f, -1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-g-b
  pA = AL::Math::Position2D(1.0f, 0.0f);
  pB = AL::Math::Position2D(-2.0f, 0.0f);
  pC = AL::Math::Position2D(-1.0f, 0.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // test 1-g-c
  pA = AL::Math::Position2D(0.0f, 1.0f);
  pB = AL::Math::Position2D(0.0f, -2.0f);
  pC = AL::Math::Position2D(0.0f, -1.0f);
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));

  // points are nearly aligned but pB is in the middle of pA, pC
  // test 2-a-a
  pA = AL::Math::Position2D(1.0f, 1.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(-1.0f, -1.0f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  // test 2-a-b
  pA = AL::Math::Position2D(0.0f, 1.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, -1.0f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  // test 2-a-c
  pA = AL::Math::Position2D(1.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(-1.0f, 0.0f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  // test 2-b-a
  pA = AL::Math::Position2D(-1.0f, -1.0f);
  pB = AL::Math::Position2D(0.1f, 0.1f);
  pC = AL::Math::Position2D(1.0f, 1.0f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  // test 2-b-b
  pA = AL::Math::Position2D(0.0f, -1.0f);
  pB = AL::Math::Position2D(0.0f, 0.1f);
  pC = AL::Math::Position2D(0.0f, 1.0f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  // test 2-b-c
  pA = AL::Math::Position2D(-1.0f, 0.0f);
  pB = AL::Math::Position2D(0.1f, 0.0f);
  pC = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  // isLeft basic function
  // test 3-a
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // test 3-b
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(1.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // test 3-c
  pA = AL::Math::Position2D(1.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_EQ(-4, AL::Math::isLeftBest(pA, pB, pC));

  // test 3-d
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(2.0f*TO_RAD));
  EXPECT_EQ(-1, AL::Math::isLeftBest(pA, pB, pC));

  // test 3-e
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(-2.0f*TO_RAD));
  EXPECT_EQ(1, AL::Math::isLeftBest(pA, pB, pC));

  // test 3-f
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(4.0f*TO_RAD));
  EXPECT_EQ(-1, AL::Math::isLeftBest(pA, pB, pC));

  // test 3-g
  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(-4.0f*TO_RAD));
  EXPECT_EQ(1, AL::Math::isLeftBest(pA, pB, pC));

  // isLeftOld
  // test 4-a
  pA = AL::Math::Position2D(-0.0780565887689590454101562f, -0.0085528194904327392578125f);
  pB = AL::Math::Position2D(-0.0325009450316429138183594f, 0.0667377635836601257324219f);
  pC = AL::Math::Position2D(-0.0200766772031784057617188f, 0.0872715637087821960449219f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));

  // test 4-b
  pA = AL::Math::Position2D(-0.0780565887689590454101562f, -0.0085528194904327392578125f);
  pB = AL::Math::Position2D(-0.0325009450316429138183594f, 0.0667377635836601257324219f);
  pC = AL::Math::Position2D(0.0254789683967828750610352f, 0.1625621467828750610351562f);
  EXPECT_EQ(-2, AL::Math::isLeftBest(pA, pB, pC));
}


TEST(sortPredicate, compare)
{
  AL::Math::Position2D p1;
  AL::Math::Position2D p2;

  // test 0
  //std::cout << "test 0" << std::endl;
  p1 = AL::Math::Position2D(0.0f, 0.0f);
  p2 = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_FALSE(sortPredicate(p1, p2));

  // test 1
  //std::cout << "test 1" << std::endl;
  p1 = AL::Math::Position2D(0.0f, 1.0f);
  p2 = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_FALSE(sortPredicate(p1, p2));

  // test 2
  //std::cout << "test 2" << std::endl;
  p1 = AL::Math::Position2D(0.0f, 0.0f);
  p2 = AL::Math::Position2D(0.0f, 1.0f);
  EXPECT_TRUE(sortPredicate(p1, p2));

  // test 3
  //std::cout << "test 3" << std::endl;
  p1 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  p2 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  EXPECT_FALSE(sortPredicate(p1, p2));

  // test 4
  //std::cout << "test 4" << std::endl;
  p1 = AL::Math::Position2D(sinf(3.0001*TO_RAD), sinf(5.0*TO_RAD));
  p2 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  EXPECT_FALSE(sortPredicate(p1, p2));

  // test 5
  //std::cout << "test 5" << std::endl;
  p1 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  p2 = AL::Math::Position2D(sinf(3.0001*TO_RAD), sinf(5.0*TO_RAD));
  EXPECT_TRUE(sortPredicate(p1, p2));

  // test 6
  //std::cout << "test 6" << std::endl;
  p1 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0001*TO_RAD));
  p2 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  EXPECT_FALSE(sortPredicate(p1, p2));

  // test 7
  //std::cout << "test 7" << std::endl;
  p1 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  p2 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0001*TO_RAD));
  EXPECT_TRUE(sortPredicate(p1, p2));

  // test 8
  //std::cout << "test 8" << std::endl;
  p1 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  p2 = AL::Math::Position2D(sinf(5.0*TO_RAD), sinf(3.0*TO_RAD));
  EXPECT_TRUE(sortPredicate(p1, p2));

  // test 9
  //std::cout << "test 9" << std::endl;
  p1 = AL::Math::Position2D(sinf(5.0*TO_RAD), sinf(3.0*TO_RAD));
  p2 = AL::Math::Position2D(sinf(3.0*TO_RAD), sinf(5.0*TO_RAD));
  EXPECT_FALSE(sortPredicate(p1, p2));

}

TEST(deleteDoublesInNoneSortVector, Enveloppe)
{
  std::vector<AL::Math::Position2D> pPoints;

  // One point
  pPoints.resize(1);
  pPoints[0] = AL::Math::Position2D(0.0f, 0.0f);
  AL::Math::deleteDoublesInNoneSortVector(pPoints);
  EXPECT_EQ(1, (int)pPoints.size());

  // Two points a
  pPoints.resize(2);
  pPoints[0] = AL::Math::Position2D(0.0f, 0.0f);
  pPoints[1] = AL::Math::Position2D(0.0f, 0.0f);
  AL::Math::deleteDoublesInNoneSortVector(pPoints);
  EXPECT_EQ(1, (int)pPoints.size());

  // Two points b
  pPoints.resize(2);
  pPoints[0] = AL::Math::Position2D(1.0f, 0.0f);
  pPoints[1] = AL::Math::Position2D(1.0001f, 0.0f);
  AL::Math::deleteDoublesInNoneSortVector(pPoints, 0.001f);
  EXPECT_EQ(1, (int)pPoints.size());

  // Two points c
  pPoints.resize(2);
  pPoints[0] = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[1] = AL::Math::Position2D(1.0f, 1.001f);
  AL::Math::deleteDoublesInNoneSortVector(pPoints, 0.0001f);
  EXPECT_EQ(2, (int)pPoints.size());

  // Six Point
  pPoints.resize(6);
  pPoints[0] = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[1] = AL::Math::Position2D(-1.0f, -1.0f);
  pPoints[2] = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[3] = AL::Math::Position2D(-1.00001f, -1.0f);
  pPoints[4] = AL::Math::Position2D(1.0f, 1.00001f);
  pPoints[5] = AL::Math::Position2D(-1.0f, -1.00001f);

  AL::Math::deleteDoublesInNoneSortVector(pPoints, 0.0001f);
  EXPECT_EQ(2, (int)pPoints.size());

  // Lots of Points a
  pPoints.resize(11);
  pPoints[0]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[1]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[2]  = AL::Math::Position2D(-1.0f, -1.0f);
  pPoints[3]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[4]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[5]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[6]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[7]  = AL::Math::Position2D(-1.01f, -1.0f);
  pPoints[8]  = AL::Math::Position2D(1.0f, 1.01f);
  pPoints[9]  = AL::Math::Position2D(-1.0f, -1.01f);
  pPoints[10] = AL::Math::Position2D(2.01f, 1.0f);

  AL::Math::deleteDoublesInNoneSortVector(pPoints, 0.1f);
  EXPECT_EQ(3, (int)pPoints.size());

  // Lots of Points b
  pPoints.resize(11);
  pPoints[0]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[1]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[2]  = AL::Math::Position2D(-1.0f, -1.0f);
  pPoints[3]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[4]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[5]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[6]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[7]  = AL::Math::Position2D(-1.01f, -1.0f);
  pPoints[8]  = AL::Math::Position2D(1.0f, 1.01f);
  pPoints[9]  = AL::Math::Position2D(-1.0f, -1.01f);
  pPoints[10] = AL::Math::Position2D(2.01f, 1.0f);

  AL::Math::deleteDoublesInNoneSortVector(pPoints, 0.001f);
  EXPECT_EQ(7, (int)pPoints.size());
}

TEST(deleteDoublesInSortVector, Enveloppe)
{
  std::vector<AL::Math::Position2D> pPoints;

  // One point
  pPoints.resize(1);
  pPoints[0] = AL::Math::Position2D(0.0f, 0.0f);
  AL::Math::deleteDoublesInSortVector(pPoints);
  EXPECT_EQ(1, (int)pPoints.size());

  // Two points a
  pPoints.resize(2);
  pPoints[0] = AL::Math::Position2D(0.0f, 0.0f);
  pPoints[1] = AL::Math::Position2D(0.0f, 0.0f);
  AL::Math::deleteDoublesInSortVector(pPoints);
  EXPECT_EQ(1, (int)pPoints.size());

  // Two points b
  pPoints.resize(2);
  pPoints[0] = AL::Math::Position2D(1.0f, 0.0f);
  pPoints[1] = AL::Math::Position2D(1.0001f, 0.0f);
  AL::Math::deleteDoublesInSortVector(pPoints, 0.001f);
  EXPECT_EQ(1, (int)pPoints.size());

  // Two points c
  pPoints.resize(2);
  pPoints[0] = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[1] = AL::Math::Position2D(1.0f, 1.001f);
  AL::Math::deleteDoublesInSortVector(pPoints, 0.0001f);
  EXPECT_EQ(2, (int)pPoints.size());

  // Six Point
  pPoints.resize(6);
  pPoints[0] = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[1] = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[2] = AL::Math::Position2D(1.0f, 1.00001f);
  pPoints[3] = AL::Math::Position2D(-1.0f, -1.0f);
  pPoints[4] = AL::Math::Position2D(-1.00001f, -1.0f);
  pPoints[5] = AL::Math::Position2D(-1.0f, -1.00001f);

  AL::Math::deleteDoublesInSortVector(pPoints);
  EXPECT_EQ(2, (int)pPoints.size());

  // Lots of Points a
  pPoints.resize(11);
  pPoints[0]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[1]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[2]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[3]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[4]  = AL::Math::Position2D(2.01f, 1.0f);
  pPoints[5]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[6]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[7]  = AL::Math::Position2D(1.0f, 1.01f);
  pPoints[8]  = AL::Math::Position2D(-1.0f, -1.0f);
  pPoints[9]  = AL::Math::Position2D(-1.01f, -1.0f);
  pPoints[10] = AL::Math::Position2D(-1.0f, -1.01f);
  AL::Math::deleteDoublesInSortVector(pPoints, 0.1f);
  EXPECT_EQ(3, (int)pPoints.size());

  // Lots of Points b
  pPoints.resize(11);
  pPoints[0]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[1]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[2]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[3]  = AL::Math::Position2D(2.0f, 1.0f);
  pPoints[4]  = AL::Math::Position2D(2.01f, 1.0f);
  pPoints[5]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[6]  = AL::Math::Position2D(1.0f, 1.0f);
  pPoints[7]  = AL::Math::Position2D(1.0f, 1.01f);
  pPoints[8]  = AL::Math::Position2D(-1.0f, -1.0f);
  pPoints[9]  = AL::Math::Position2D(-1.01f, -1.0f);
  pPoints[10] = AL::Math::Position2D(-1.0f, -1.01f);

  AL::Math::deleteDoublesInSortVector(pPoints, 0.001f);
  EXPECT_EQ(7, (int)pPoints.size());
}


TEST(isLeft, test0)
{
  float epsilon = sinf(3.0f * TO_RAD);
  AL::Math::Position2D pA;
  AL::Math::Position2D pB;
  AL::Math::Position2D pC;

  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, 0.0f);
  EXPECT_EQ(0, AL::Math::isLeft(pA, pB, pC, epsilon));

  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(1.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_EQ(0, AL::Math::isLeft(pA, pB, pC, epsilon));

  pA = AL::Math::Position2D(1.0f, 0.0f);
  pB = AL::Math::Position2D(0.0f, 0.0f);
  pC = AL::Math::Position2D(0.0f, 0.0f);
  EXPECT_EQ(0, AL::Math::isLeft(pA, pB, pC, epsilon));

  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(2.0f*TO_RAD));
  EXPECT_EQ(0, AL::Math::isLeft(pA, pB, pC, epsilon));

  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(-2.0f*TO_RAD));
  EXPECT_EQ(0, AL::Math::isLeft(pA, pB, pC, epsilon));

  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(4.0f*TO_RAD));
  EXPECT_EQ(-1, AL::Math::isLeft(pA, pB, pC, epsilon));

  pA = AL::Math::Position2D(0.0f, 0.0f);
  pB = AL::Math::Position2D(-1.0f, 0.0f);
  pC = AL::Math::Position2D(1.0f, sinf(-4.0f*TO_RAD));
  EXPECT_EQ(1, AL::Math::isLeft(pA, pB, pC, epsilon));
}

TEST(isLeftOld, test0)
{
  AL::Math::Position2D pA;
  AL::Math::Position2D pB;
  AL::Math::Position2D pC;
  float epsilon = 0.000000001f;

  pA = AL::Math::Position2D(-0.0780565887689590454101562f, -0.0085528194904327392578125f);
  pB = AL::Math::Position2D(-0.0325009450316429138183594f, 0.0667377635836601257324219f);
  pC = AL::Math::Position2D(-0.0200766772031784057617188f, 0.0872715637087821960449219f);
  // cross plateform dependant
  // EXPECT_EQ(1, AL::Math::isLeftOld(pA, pB, pC, 0.0f));
  EXPECT_EQ(0, AL::Math::isLeftOld(pA, pB, pC, epsilon));

  pA = AL::Math::Position2D(-0.0780565887689590454101562f, -0.0085528194904327392578125f);
  pB = AL::Math::Position2D(-0.0325009450316429138183594f, 0.0667377635836601257324219f);
  pC = AL::Math::Position2D(0.0254789683967828750610352f, 0.1625621467828750610351562f);
  // cross plateform dependant
  //EXPECT_EQ(-1, AL::Math::isLeftOld(pA, pB, pC, 0.0f));
  EXPECT_EQ(0, AL::Math::isLeftOld(pA, pB, pC, epsilon));
}


TEST(ConvexhullTest, timeOut0)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);

  lContactPoints.at(0) = AL::Math::Position2D( 0.1340611875057220458984375f, 0.0968693196773529052734375f);
  lContactPoints.at(1) = AL::Math::Position2D( 0.0885055512189865112304688f, 0.0215787440538406372070312f);
  lContactPoints.at(2) = AL::Math::Position2D( -0.0200766772031784057617188f, 0.0872715637087821960449219f);
  lContactPoints.at(3) = AL::Math::Position2D( 0.0254789683967828750610352f, 0.1625621467828750610351562f);
  lContactPoints.at(4) = AL::Math::Position2D( 0.0760812759399414062500000f, 0.0010449402034282684326172f);
  lContactPoints.at(5) = AL::Math::Position2D( 0.0305256322026252746582031f, -0.0742456391453742980957031f);
  lContactPoints.at(6) = AL::Math::Position2D( -0.0780565887689590454101562f, -0.0085528194904327392578125f);
  lContactPoints.at(7) = AL::Math::Position2D( -0.0325009450316429138183594f, 0.0667377635836601257324219f);

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());

  lContactPoints = AL::Math::getConvexHull(lContactPoints);
  EXPECT_EQ(5, (int)lContactPoints.size());
}


TEST(ConvexhullTest, timeOut1)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);

  lContactPoints.at(0) = AL::Math::Position2D( -1.0f, -1.0f);
  lContactPoints.at(1) = AL::Math::Position2D( -0.5f, -1.5f);
  lContactPoints.at(2) = AL::Math::Position2D(  0.5f, -1.5f);
  lContactPoints.at(3) = AL::Math::Position2D(  1.0f, -1.0f);
  lContactPoints.at(4) = AL::Math::Position2D(  1.0f, 1.0f);
  lContactPoints.at(5) = AL::Math::Position2D(  0.5f, 1.5f);
  lContactPoints.at(6) = AL::Math::Position2D( -0.5f, 1.5f);
  lContactPoints.at(7) = AL::Math::Position2D( -1.0f, 1.0f);

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(8, (int)result.size());

  lContactPoints = AL::Math::getConvexHull(lContactPoints);
  EXPECT_EQ(9, (int)lContactPoints.size());
}

TEST(ConvexhullTest, timeOut2)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);

  lContactPoints.at(0) = AL::Math::Position2D( -1.0f, -1.0f); // pt 0
  lContactPoints.at(1) = AL::Math::Position2D( -0.5f, -1.0f); // pt 1
  lContactPoints.at(2) = AL::Math::Position2D(  0.5f, -1.0f); // pt 2
  lContactPoints.at(3) = AL::Math::Position2D(  1.0f, -1.0f); // pt 3
  lContactPoints.at(4) = AL::Math::Position2D(  1.0f, 1.0f); // pt 4
  lContactPoints.at(5) = AL::Math::Position2D(  0.5f, 1.0f); // pt 5
  lContactPoints.at(6) = AL::Math::Position2D( -0.5f, 1.0f); // pt 6
  lContactPoints.at(7) = AL::Math::Position2D( -1.0f, 1.0f); // pt 7

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());

  lContactPoints = AL::Math::getConvexHull(lContactPoints);
  EXPECT_EQ(5, (int)lContactPoints.size());
}

TEST(ConvexhullTest, timeOut3)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);

  lContactPoints.at(1) = AL::Math::Position2D( -0.5f, -1.0f); // pt 1
  lContactPoints.at(0) = AL::Math::Position2D( -1.0f, -1.0f); // pt 0
  lContactPoints.at(5) = AL::Math::Position2D(  0.5f, 1.0f); // pt 5
  lContactPoints.at(6) = AL::Math::Position2D( -0.5f, 1.0f); // pt 6
  lContactPoints.at(2) = AL::Math::Position2D(  0.5f, -1.0f); // pt 2
  lContactPoints.at(3) = AL::Math::Position2D(  1.0f, -1.0f); // pt 3
  lContactPoints.at(4) = AL::Math::Position2D(  1.0f, 1.0f); // pt 4
  lContactPoints.at(7) = AL::Math::Position2D( -1.0f, 1.0f); // pt 7
  //  std::cout << "    pt0 = [" << std::endl;
  //  for (unsigned int i=0; i<lContactPoints.size(); i++)
  //  {
  //    std::cout << "[ " << lContactPoints.at(i).x << ", " << lContactPoints.at(i).y << "]," << std::endl;
  //  }
  //  std::cout << "]" << std::endl;

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());

  lContactPoints = AL::Math::getConvexHull(lContactPoints);
  EXPECT_EQ(5, (int)lContactPoints.size());
  //  std::cout << "    pt1 = [" << std::endl;
  //  for (unsigned int i=0; i<result.size(); i++)
  //  {
  //    std::cout << "[ " << result.at(i).x << ", " << result.at(i).y << "]," << std::endl;
  //  }
  //  std::cout << "]" << std::endl;
  //  std::cout << std::endl;
}

TEST(ConvexhullTest, timeOut4)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);
  lContactPoints.at(0) = AL::Math::Position2D(0.134061f, 0.0968693f);
  lContactPoints.at(1) = AL::Math::Position2D(0.0885056f, 0.0215787f);
  lContactPoints.at(2) = AL::Math::Position2D(-0.0200767f, 0.0872716f);
  lContactPoints.at(3) = AL::Math::Position2D(0.025479f, 0.162562f);
  lContactPoints.at(4) = AL::Math::Position2D(0.0760813f, 0.00104494f);
  lContactPoints.at(5) = AL::Math::Position2D(0.0305256f, -0.0742456f);
  lContactPoints.at(6) = AL::Math::Position2D(-0.0780566f, -0.00855282f);
  lContactPoints.at(7) = AL::Math::Position2D(-0.0325009f, 0.0667378f);

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());

  lContactPoints = AL::Math::getConvexHull(lContactPoints);
  EXPECT_EQ(5, (int)lContactPoints.size());
}

TEST(ConvexhullTest, timeOut5)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);
  lContactPoints.at(0) = AL::Math::Position2D(0.0800774842500686645507812f, 0.1515965759754180908203125f);
  lContactPoints.at(1) = AL::Math::Position2D(0.0800782665610313415527344f, 0.0635965839028358459472656f);
  lContactPoints.at(2) = AL::Math::Position2D(-0.0469217263162136077880859f, 0.0635954439640045166015625);
  lContactPoints.at(3) = AL::Math::Position2D(-0.0469225160777568817138672f, 0.1515954434871673583984375);
  lContactPoints.at(4) = AL::Math::Position2D(0.0800784975290298461914062f, 0.0395965911448001861572266);
  lContactPoints.at(5) = AL::Math::Position2D(0.0800792798399925231933594f, -0.0484033934772014617919922);
  lContactPoints.at(6) = AL::Math::Position2D(-0.0469207167625427246093750f, -0.0484045296907424926757812);
  lContactPoints.at(7) = AL::Math::Position2D(-0.0469215027987957000732422f, 0.0395954549312591552734375);

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());

  lContactPoints = AL::Math::getConvexHull(lContactPoints);
  EXPECT_EQ(5, (int)lContactPoints.size());
}

TEST(ConvexhullTest, timeOut6)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);
  lContactPoints.at(0) = AL::Math::Position2D(-0.0469849556684494018554688f, 0.0607356913387775421142578f);
  lContactPoints.at(1) = AL::Math::Position2D(-0.0469849556684494018554688f, 0.1487356722354888916015625f);
  lContactPoints.at(2) = AL::Math::Position2D(-0.0469849519431591033935547f, -0.0512642934918403625488281f);
  lContactPoints.at(3) = AL::Math::Position2D(-0.0469849519431591033935547f, 0.0367356948554515838623047f);
  lContactPoints.at(4) = AL::Math::Position2D(0.0800150334835052490234375f, 0.0607356913387775421142578f);
  lContactPoints.at(5) = AL::Math::Position2D(0.0800150334835052490234375f, 0.1487356722354888916015625f);
  lContactPoints.at(6) = AL::Math::Position2D(0.0800150409340858459472656f, -0.0512642972171306610107422f);
  lContactPoints.at(7) = AL::Math::Position2D(0.0800150409340858459472656f, 0.0367356948554515838623047f);

  std::vector<AL::Math::Position2D> result;
  AL::Math::removeAlignedPoint(lContactPoints, result);
  EXPECT_EQ(4, (int)result.size());

  lContactPoints = AL::Math::getConvexHull(lContactPoints);
  EXPECT_EQ(5, (int)lContactPoints.size());
}


TEST(ConvexhullTest, isNotInside)
{
  std::vector<AL::Math::Position2D> lContactPoints;
  lContactPoints.resize(8);
  AL::Math::Position2D pA0 = AL::Math::Position2D(-1.0f, -1.0f);
  AL::Math::Position2D pA1 = AL::Math::Position2D(1.0f, 1.0f);
  AL::Math::Position2D pA2 = AL::Math::Position2D(-1.0f, 1.0f);
  AL::Math::Position2D pA3 = AL::Math::Position2D(1.0f, -1.0f);
  AL::Math::Position2D pB0 = AL::Math::Position2D(0.2f, 0.95f);
  AL::Math::Position2D pB1 = AL::Math::Position2D(-0.1f, -0.4f);
  AL::Math::Position2D pB2 = AL::Math::Position2D(-0.98f, -0.5f);
  AL::Math::Position2D pB3 = AL::Math::Position2D(-0.4f, 0.5f);

  lContactPoints.at(0) = pA0;
  lContactPoints.at(1) = pA1;
  lContactPoints.at(2) = pA2;
  lContactPoints.at(3) = pA3;
  lContactPoints.at(4) = pB0;
  lContactPoints.at(5) = pB1;
  lContactPoints.at(6) = pB2;
  lContactPoints.at(7) = pB3;

  std::vector<AL::Math::Position2D> result = AL::Math::getConvexHull(lContactPoints);
  // 4 + 1
  EXPECT_EQ(5, (int)result.size());

  std::vector<AL::Math::Position2D> isNotInside;
  isNotInside.resize(4);
  isNotInside.at(0) = pB0;
  isNotInside.at(1) = pB1;
  isNotInside.at(2) = pB2;
  isNotInside.at(3) = pB3;

  float eps = 0.0001f;
  for (unsigned i=0; i<isNotInside.size(); i++)
  {
    for (unsigned j=0; j<result.size(); j++)
    {
      EXPECT_FALSE(isNotInside.at(i).isNear(result.at(j), eps));
    }
  }

}

TEST(ConvexhullTest, randomData0)
{
  std::vector<AL::Math::Position2D> data;

  data.push_back(AL::Math::Position2D(0.677310585976f, 0.445326238871f)); // 0
  data.push_back(AL::Math::Position2D(0.328563779593f, 0.526800453663f)); // 1
  data.push_back(AL::Math::Position2D(0.239555180073f, 0.951048254967f)); // 2
  data.push_back(AL::Math::Position2D(0.321292936802f, 0.440291583538f)); // 3
  data.push_back(AL::Math::Position2D(0.859434187412f, 0.896033167839f)); // 4
  data.push_back(AL::Math::Position2D(0.386588722467f, 0.0177024994045f)); // 5
  data.push_back(AL::Math::Position2D(0.0532262027264f, 0.750740170479f)); // 6
  data.push_back(AL::Math::Position2D(0.936870276928f, 0.43109151721f)); // 7
  data.push_back(AL::Math::Position2D(0.975853681564f, 0.0119071817026f)); // 8
  data.push_back(AL::Math::Position2D(0.435120731592f, 0.55413287878f)); // 9
  data.push_back(AL::Math::Position2D(0.682298719883f, 0.297037899494f)); // 10
  data.push_back(AL::Math::Position2D(0.730857431889f, 0.305490285158f)); // 11
  data.push_back(AL::Math::Position2D(0.559589266777f, 0.54561150074f)); // 12
  data.push_back(AL::Math::Position2D(0.296131283045f, 0.174172952771f)); // 13
  data.push_back(AL::Math::Position2D(0.54928201437f, 0.912389099598f)); // 14
  data.push_back(AL::Math::Position2D(0.516762733459f, 0.0313000865281f)); // 15
  data.push_back(AL::Math::Position2D(0.558867096901f, 0.267187654972f)); // 16
  data.push_back(AL::Math::Position2D(0.240093365312f, 0.457963705063f)); // 17
  data.push_back(AL::Math::Position2D(0.00794032495469f, 0.383496522903f)); // 18
  data.push_back(AL::Math::Position2D(0.171508356929f, 0.756026089191f)); // 19
  data.push_back(AL::Math::Position2D(0.729119002819f, 0.701911389828f)); // 20
  data.push_back(AL::Math::Position2D(0.45401674509f, 0.290544658899f)); // 21
  data.push_back(AL::Math::Position2D(0.44874599576f, 0.912504076958f)); // 22
  data.push_back(AL::Math::Position2D(0.901389360428f, 0.997823297977f)); // 23
  data.push_back(AL::Math::Position2D(0.409188061953f, 0.438418835402f)); // 24
  data.push_back(AL::Math::Position2D(0.497078597546f, 0.955515980721f)); // 25
  data.push_back(AL::Math::Position2D(0.081779949367f, 0.124488346279f)); // 26
  data.push_back(AL::Math::Position2D(0.259384989738f, 0.577743411064f)); // 27
  data.push_back(AL::Math::Position2D(0.234114795923f, 0.337345838547f)); // 28
  data.push_back(AL::Math::Position2D(0.567814469337f, 0.493972271681f)); // 29
  data.push_back(AL::Math::Position2D(0.922242164612f, 0.645041167736f)); // 30
  data.push_back(AL::Math::Position2D(0.482959985733f, 0.131904512644f)); // 31
  data.push_back(AL::Math::Position2D(0.197735369205f, 0.763662993908f)); // 32
  data.push_back(AL::Math::Position2D(0.28520283103f, 0.878617286682f)); // 33
  data.push_back(AL::Math::Position2D(0.44627264142f, 0.315655231476f)); // 34
  data.push_back(AL::Math::Position2D(0.267806380987f, 0.681060552597f)); // 35
  data.push_back(AL::Math::Position2D(0.875261843204f, 0.30532887578f)); // 36
  data.push_back(AL::Math::Position2D(0.628345012665f, 0.42461976409f)); // 37
  data.push_back(AL::Math::Position2D(0.779474496841f, 0.833099603653f)); // 38
  data.push_back(AL::Math::Position2D(0.811452865601f, 0.86954075098f)); // 39
  data.push_back(AL::Math::Position2D(0.12665514648f, 0.523886144161f)); // 40
  data.push_back(AL::Math::Position2D(0.826174616814f, 0.0483366213739f)); // 41
  data.push_back(AL::Math::Position2D(0.098691470921f, 0.626796782017f)); // 42
  data.push_back(AL::Math::Position2D(0.181802302599f, 0.954275369644f)); // 43
  data.push_back(AL::Math::Position2D(0.0776682198048f, 0.53852301836f)); // 44
  data.push_back(AL::Math::Position2D(0.879096329212f, 0.746224284172f)); // 45
  data.push_back(AL::Math::Position2D(0.662254691124f, 0.133642658591f)); // 46
  data.push_back(AL::Math::Position2D(0.344097524881f, 0.00698854168877f)); // 47
  data.push_back(AL::Math::Position2D(0.961684823036f, 0.348055362701f)); // 48
  data.push_back(AL::Math::Position2D(0.109212383628f, 0.330442219973f)); // 49
  data.push_back(AL::Math::Position2D(0.216911241412f, 0.899994015694f)); // 50
  data.push_back(AL::Math::Position2D(0.121607467532f, 0.409139275551f)); // 51
  data.push_back(AL::Math::Position2D(0.464431732893f, 0.393364429474f)); // 52
  data.push_back(AL::Math::Position2D(0.300092637539f, 0.864602982998f)); // 53
  data.push_back(AL::Math::Position2D(0.989485740662f, 0.587934613228f)); // 54
  data.push_back(AL::Math::Position2D(0.304044246674f, 0.484463214874f)); // 55
  data.push_back(AL::Math::Position2D(0.213271528482f, 0.0417677238584f)); // 56
  data.push_back(AL::Math::Position2D(0.74173951149f, 0.106455594301f)); // 57
  data.push_back(AL::Math::Position2D(0.406044155359f, 0.244777172804f)); // 58
  data.push_back(AL::Math::Position2D(0.718091189861f, 0.508838474751f)); // 59
  data.push_back(AL::Math::Position2D(0.430689334869f, 0.914450585842f)); // 60
  data.push_back(AL::Math::Position2D(0.143121793866f, 0.787936806679f)); // 61
  data.push_back(AL::Math::Position2D(0.555941164494f, 0.639904499054f)); // 62
  data.push_back(AL::Math::Position2D(0.551331520081f, 0.961266815662f)); // 63
  data.push_back(AL::Math::Position2D(0.79120016098f, 0.588202118874f)); // 64
  data.push_back(AL::Math::Position2D(0.926067590714f, 0.269413948059f)); // 65
  data.push_back(AL::Math::Position2D(0.686304092407f, 0.803648531437f)); // 66
  data.push_back(AL::Math::Position2D(0.2507365942f, 0.653416872025f)); // 67
  data.push_back(AL::Math::Position2D(0.0221011377871f, 0.162954747677f)); // 68
  data.push_back(AL::Math::Position2D(0.675253272057f, 0.26399487257f)); // 69
  data.push_back(AL::Math::Position2D(0.417556762695f, 0.646476984024f)); // 70
  data.push_back(AL::Math::Position2D(0.213026463985f, 0.109371192753f)); // 71
  data.push_back(AL::Math::Position2D(0.114393182099f, 0.00712344888598f)); // 72
  data.push_back(AL::Math::Position2D(0.574607789516f, 0.267561435699f)); // 73
  data.push_back(AL::Math::Position2D(0.233883917332f, 0.60515666008f)); // 74
  data.push_back(AL::Math::Position2D(0.906184494495f, 0.563978970051f)); // 75
  data.push_back(AL::Math::Position2D(0.282278716564f, 0.0484978444874f)); // 76
  data.push_back(AL::Math::Position2D(0.0742930024862f, 0.215870693326f)); // 77
  data.push_back(AL::Math::Position2D(0.51386320591f, 0.335956454277f)); // 78
  data.push_back(AL::Math::Position2D(0.226919591427f, 0.610356390476f)); // 79
  data.push_back(AL::Math::Position2D(0.0444538705051f, 0.749680459499f)); // 80
  data.push_back(AL::Math::Position2D(0.493995189667f, 0.650762140751f)); // 81
  data.push_back(AL::Math::Position2D(0.627190589905f, 0.59732145071f)); // 82
  data.push_back(AL::Math::Position2D(0.188941627741f, 0.626328766346f)); // 83
  data.push_back(AL::Math::Position2D(0.141846910119f, 0.696808993816f)); // 84
  data.push_back(AL::Math::Position2D(0.20946021378f, 0.117243401706f)); // 85
  data.push_back(AL::Math::Position2D(0.175381317735f, 0.168043121696f)); // 86
  data.push_back(AL::Math::Position2D(0.533763229847f, 0.318442374468f)); // 87
  data.push_back(AL::Math::Position2D(0.576832950115f, 0.951615393162f)); // 88
  data.push_back(AL::Math::Position2D(0.854773283005f, 0.838581323624f)); // 89
  data.push_back(AL::Math::Position2D(0.0158754810691f, 0.69892013073f)); // 90
  data.push_back(AL::Math::Position2D(0.741671919823f, 0.24392645061f)); // 91
  data.push_back(AL::Math::Position2D(0.706568241119f, 0.375537097454f)); // 92
  data.push_back(AL::Math::Position2D(0.353552401066f, 0.0667596310377f)); // 93
  data.push_back(AL::Math::Position2D(0.09920604527f, 0.632477045059f)); // 94
  data.push_back(AL::Math::Position2D(0.191138833761f, 0.689574599266f)); // 95
  data.push_back(AL::Math::Position2D(0.402021467686f, 0.148451805115f)); // 96
  data.push_back(AL::Math::Position2D(0.210900634527f, 0.294371426105f)); // 97
  data.push_back(AL::Math::Position2D(0.75238853693f, 0.697931349277f)); // 98
  data.push_back(AL::Math::Position2D(0.7789234519f, 0.360797941685f)); // 99

  std::vector<AL::Math::Position2D> pPoints;
  AL::Math::removeAlignedPoint(data, pPoints);
  EXPECT_EQ(12, (int)pPoints.size());

  std::vector<AL::Math::Position2D> result = AL::Math::getConvexHull(data);
  EXPECT_EQ(10, (int)result.size());

  AL::Math::deleteDoublesInNoneSortVector(data);
  EXPECT_EQ(100, (int)data.size());

//  // Usefull for plot in python
//  for (unsigned int i=0; i<pPoints.size(); i++)
//  {
//    std::cout << "data.append(almath.Position2D("
//              << pPoints.at(i).x << ", " << pPoints.at(i).y << "))" << std::endl;
//  }

}

TEST(ConvexhullTest, randomData1)
{
  std::vector<AL::Math::Position2D> data;
  data.resize(500);
  data.at(0)= AL::Math::Position2D(0.630846619606f, 0.131235033274f); // 0
  data.at(1)= AL::Math::Position2D(0.487262606621f, 0.959441244602f); // 1
  data.at(2)= AL::Math::Position2D(0.512066781521f, 0.679092049599f); // 2
  data.at(3)= AL::Math::Position2D(0.474194377661f, 0.127570673823f); // 3
  data.at(4)= AL::Math::Position2D(0.728863596916f, 0.803492665291f); // 4
  data.at(5)= AL::Math::Position2D(0.535410404205f, 0.330518960953f); // 5
  data.at(6)= AL::Math::Position2D(0.709564447403f, 0.12137670815f); // 6
  data.at(7)= AL::Math::Position2D(0.707871496677f, 0.579263985157f); // 7
  data.at(8)= AL::Math::Position2D(0.858471035957f, 0.964552402496f); // 8
  data.at(9)= AL::Math::Position2D(0.0782881528139f, 0.811011731625f); // 9
  data.at(10)= AL::Math::Position2D(0.104481637478f, 0.918904066086f); // 10
  data.at(11)= AL::Math::Position2D(0.592768251896f, 0.123962685466f); // 11
  data.at(12)= AL::Math::Position2D(0.981907784939f, 0.13960005343f); // 12
  data.at(13)= AL::Math::Position2D(0.9721326828f, 0.205506965518f); // 13
  data.at(14)= AL::Math::Position2D(0.378041684628f, 0.884655296803f); // 14
  data.at(15)= AL::Math::Position2D(0.903445959091f, 0.599282383919f); // 15
  data.at(16)= AL::Math::Position2D(0.161805540323f, 0.586614608765f); // 16
  data.at(17)= AL::Math::Position2D(0.761485099792f, 0.809429049492f); // 17
  data.at(18)= AL::Math::Position2D(0.660933494568f, 0.096675157547f); // 18
  data.at(19)= AL::Math::Position2D(0.188252970576f, 0.695738315582f); // 19
  data.at(20)= AL::Math::Position2D(0.694077193737f, 0.625601291656f); // 20
  data.at(21)= AL::Math::Position2D(0.345076590776f, 0.155140250921f); // 21
  data.at(22)= AL::Math::Position2D(0.518367528915f, 0.575807511806f); // 22
  data.at(23)= AL::Math::Position2D(0.27375459671f, 0.901900410652f); // 23
  data.at(24)= AL::Math::Position2D(0.205828666687f, 0.717871010303f); // 24
  data.at(25)= AL::Math::Position2D(0.684620261192f, 0.347572118044f); // 25
  data.at(26)= AL::Math::Position2D(0.028348820284f, 0.468789160252f); // 26
  data.at(27)= AL::Math::Position2D(0.214682593942f, 0.678087234497f); // 27
  data.at(28)= AL::Math::Position2D(0.83506155014f, 0.845031499863f); // 28
  data.at(29)= AL::Math::Position2D(0.754118800163f, 0.527025640011f); // 29
  data.at(30)= AL::Math::Position2D(0.264193296432f, 0.174717783928f); // 30
  data.at(31)= AL::Math::Position2D(0.370301812887f, 0.511237919331f); // 31
  data.at(32)= AL::Math::Position2D(0.0222186353058f, 0.370338052511f); // 32
  data.at(33)= AL::Math::Position2D(0.37446731329f, 0.74449968338f); // 33
  data.at(34)= AL::Math::Position2D(0.846843779087f, 0.312390357256f); // 34
  data.at(35)= AL::Math::Position2D(0.195510700345f, 0.317973166704f); // 35
  data.at(36)= AL::Math::Position2D(0.331373274326f, 0.733948707581f); // 36
  data.at(37)= AL::Math::Position2D(0.271980434656f, 0.138201266527f); // 37
  data.at(38)= AL::Math::Position2D(0.710441708565f, 0.708458781242f); // 38
  data.at(39)= AL::Math::Position2D(0.741879045963f, 0.301469296217f); // 39
  data.at(40)= AL::Math::Position2D(0.138780400157f, 0.949292004108f); // 40
  data.at(41)= AL::Math::Position2D(0.272359699011f, 0.0939516201615f); // 41
  data.at(42)= AL::Math::Position2D(0.323293805122f, 0.685826778412f); // 42
  data.at(43)= AL::Math::Position2D(0.966361105442f, 0.84337246418f); // 43
  data.at(44)= AL::Math::Position2D(0.843968808651f, 0.226242646575f); // 44
  data.at(45)= AL::Math::Position2D(0.983069717884f, 0.990286946297f); // 45
  data.at(46)= AL::Math::Position2D(0.0473419837654f, 0.758865535259f); // 46
  data.at(47)= AL::Math::Position2D(0.234538272023f, 0.21124817431f); // 47
  data.at(48)= AL::Math::Position2D(0.209718748927f, 0.336928486824f); // 48
  data.at(49)= AL::Math::Position2D(0.836091458797f, 0.482457607985f); // 49
  data.at(50)= AL::Math::Position2D(0.620043456554f, 0.632832288742f); // 50
  data.at(51)= AL::Math::Position2D(0.232830509543f, 0.0307392273098f); // 51
  data.at(52)= AL::Math::Position2D(0.370378673077f, 0.724772810936f); // 52
  data.at(53)= AL::Math::Position2D(0.447830706835f, 0.668277680874f); // 53
  data.at(54)= AL::Math::Position2D(0.660688996315f, 0.638355910778f); // 54
  data.at(55)= AL::Math::Position2D(0.989452362061f, 0.991963028908f); // 55
  data.at(56)= AL::Math::Position2D(0.726708889008f, 0.116265982389f); // 56
  data.at(57)= AL::Math::Position2D(0.862146735191f, 0.94163185358f); // 57
  data.at(58)= AL::Math::Position2D(0.33614680171f, 0.3892968297f); // 58
  data.at(59)= AL::Math::Position2D(0.725461781025f, 0.535447835922f); // 59
  data.at(60)= AL::Math::Position2D(0.797825932503f, 0.89073818922f); // 60
  data.at(61)= AL::Math::Position2D(0.4855209589f, 0.569038331509f); // 61
  data.at(62)= AL::Math::Position2D(0.703807473183f, 0.181643277407f); // 62
  data.at(63)= AL::Math::Position2D(0.317476570606f, 0.509352624416f); // 63
  data.at(64)= AL::Math::Position2D(0.91603654623f, 0.834636747837f); // 64
  data.at(65)= AL::Math::Position2D(0.977710068226f, 0.212925568223f); // 65
  data.at(66)= AL::Math::Position2D(0.254540324211f, 0.0808759853244f); // 66
  data.at(67)= AL::Math::Position2D(0.0918958261609f, 0.502280473709f); // 67
  data.at(68)= AL::Math::Position2D(0.687414705753f, 0.165264353156f); // 68
  data.at(69)= AL::Math::Position2D(0.209274813533f, 0.684693813324f); // 69
  data.at(70)= AL::Math::Position2D(0.545781075954f, 0.679328799248f); // 70
  data.at(71)= AL::Math::Position2D(0.598384678364f, 0.380478352308f); // 71
  data.at(72)= AL::Math::Position2D(0.397645384073f, 0.270192623138f); // 72
  data.at(73)= AL::Math::Position2D(0.861783862114f, 0.436762452126f); // 73
  data.at(74)= AL::Math::Position2D(0.12450183928f, 0.186725303531f); // 74
  data.at(75)= AL::Math::Position2D(0.714895904064f, 0.875653624535f); // 75
  data.at(76)= AL::Math::Position2D(0.421828716993f, 0.787335395813f); // 76
  data.at(77)= AL::Math::Position2D(0.256512761116f, 0.0150716360658f); // 77
  data.at(78)= AL::Math::Position2D(0.692923486233f, 0.924615323544f); // 78
  data.at(79)= AL::Math::Position2D(0.0762142315507f, 0.42940813303f); // 79
  data.at(80)= AL::Math::Position2D(0.774227380753f, 0.566074669361f); // 80
  data.at(81)= AL::Math::Position2D(0.09894669801f, 0.439404368401f); // 81
  data.at(82)= AL::Math::Position2D(0.676480293274f, 0.515183091164f); // 82
  data.at(83)= AL::Math::Position2D(0.871188640594f, 0.266047805548f); // 83
  data.at(84)= AL::Math::Position2D(0.275940924883f, 0.362296491861f); // 84
  data.at(85)= AL::Math::Position2D(0.243963301182f, 0.652403116226f); // 85
  data.at(86)= AL::Math::Position2D(0.496082335711f, 0.116514235735f); // 86
  data.at(87)= AL::Math::Position2D(0.765866696835f, 0.0512890145183f); // 87
  data.at(88)= AL::Math::Position2D(0.721534311771f, 0.627261519432f); // 88
  data.at(89)= AL::Math::Position2D(0.129001945257f, 0.895295858383f); // 89
  data.at(90)= AL::Math::Position2D(0.331524670124f, 0.186273917556f); // 90
  data.at(91)= AL::Math::Position2D(0.814960718155f, 0.442887216806f); // 91
  data.at(92)= AL::Math::Position2D(0.676512658596f, 0.28601911664f); // 92
  data.at(93)= AL::Math::Position2D(0.781223356724f, 0.22683981061f); // 93
  data.at(94)= AL::Math::Position2D(0.489106684923f, 0.66792267561f); // 94
  data.at(95)= AL::Math::Position2D(0.0845558717847f, 0.132287293673f); // 95
  data.at(96)= AL::Math::Position2D(0.643392145634f, 0.955872535706f); // 96
  data.at(97)= AL::Math::Position2D(0.902115225792f, 0.825219213963f); // 97
  data.at(98)= AL::Math::Position2D(0.752575099468f, 0.394358843565f); // 98
  data.at(99)= AL::Math::Position2D(0.703024804592f, 0.950172662735f); // 99
  data.at(100)= AL::Math::Position2D(0.67413944006f, 0.576347708702f); // 100
  data.at(101)= AL::Math::Position2D(0.798583269119f, 0.075881794095f); // 101
  data.at(102)= AL::Math::Position2D(0.948325574398f, 0.476324141026f); // 102
  data.at(103)= AL::Math::Position2D(0.213622286916f, 0.238042935729f); // 103
  data.at(104)= AL::Math::Position2D(0.718696653843f, 0.393064647913f); // 104
  data.at(105)= AL::Math::Position2D(0.344175964594f, 0.402402251959f); // 105
  data.at(106)= AL::Math::Position2D(0.629895091057f, 0.228421986103f); // 106
  data.at(107)= AL::Math::Position2D(0.0205710362643f, 0.0437297709286f); // 107
  data.at(108)= AL::Math::Position2D(0.0379168018699f, 0.281520426273f); // 108
  data.at(109)= AL::Math::Position2D(0.0866056904197f, 0.593845903873f); // 109
  data.at(110)= AL::Math::Position2D(0.382068902254f, 0.704332232475f); // 110
  data.at(111)= AL::Math::Position2D(0.351948946714f, 0.172564223409f); // 111
  data.at(112)= AL::Math::Position2D(0.0278392471373f, 0.125393971801f); // 112
  data.at(113)= AL::Math::Position2D(0.0120478300378f, 0.485544800758f); // 113
  data.at(114)= AL::Math::Position2D(0.955704629421f, 0.683179855347f); // 114
  data.at(115)= AL::Math::Position2D(0.0558641441166f, 0.524181008339f); // 115
  data.at(116)= AL::Math::Position2D(0.857355892658f, 0.65628772974f); // 116
  data.at(117)= AL::Math::Position2D(0.773080706596f, 0.0695986449718f); // 117
  data.at(118)= AL::Math::Position2D(0.739238321781f, 0.424909472466f); // 118
  data.at(119)= AL::Math::Position2D(0.197705686092f, 0.464898735285f); // 119
  data.at(120)= AL::Math::Position2D(0.648743748665f, 0.61195397377f); // 120
  data.at(121)= AL::Math::Position2D(0.382228851318f, 0.0499582365155f); // 121
  data.at(122)= AL::Math::Position2D(0.19865770638f, 0.694912433624f); // 122
  data.at(123)= AL::Math::Position2D(0.320533245802f, 0.0407115183771f); // 123
  data.at(124)= AL::Math::Position2D(0.178019121289f, 0.504747450352f); // 124
  data.at(125)= AL::Math::Position2D(0.240491256118f, 0.315289825201f); // 125
  data.at(126)= AL::Math::Position2D(0.1044979617f, 0.62996417284f); // 126
  data.at(127)= AL::Math::Position2D(0.264792561531f, 0.218966811895f); // 127
  data.at(128)= AL::Math::Position2D(0.92709094286f, 0.856458365917f); // 128
  data.at(129)= AL::Math::Position2D(0.722064495087f, 0.590558111668f); // 129
  data.at(130)= AL::Math::Position2D(0.60188138485f, 0.133012533188f); // 130
  data.at(131)= AL::Math::Position2D(0.889636874199f, 0.133030712605f); // 131
  data.at(132)= AL::Math::Position2D(0.949947595596f, 0.894047260284f); // 132
  data.at(133)= AL::Math::Position2D(0.983167827129f, 0.341620355844f); // 133
  data.at(134)= AL::Math::Position2D(0.6968729496f, 0.768132984638f); // 134
  data.at(135)= AL::Math::Position2D(0.32586812973f, 0.69217312336f); // 135
  data.at(136)= AL::Math::Position2D(0.844094693661f, 0.425926983356f); // 136
  data.at(137)= AL::Math::Position2D(0.514107704163f, 0.444015055895f); // 137
  data.at(138)= AL::Math::Position2D(0.870029330254f, 0.60736566782f); // 138
  data.at(139)= AL::Math::Position2D(0.383093953133f, 0.233669891953f); // 139
  data.at(140)= AL::Math::Position2D(0.151494309306f, 0.354553759098f); // 140
  data.at(141)= AL::Math::Position2D(0.262834817171f, 0.817739665508f); // 141
  data.at(142)= AL::Math::Position2D(0.272132962942f, 0.133487001061f); // 142
  data.at(143)= AL::Math::Position2D(0.835722982883f, 0.785923540592f); // 143
  data.at(144)= AL::Math::Position2D(0.943804502487f, 0.63190728426f); // 144
  data.at(145)= AL::Math::Position2D(0.0752700269222f, 0.649121820927f); // 145
  data.at(146)= AL::Math::Position2D(0.101102292538f, 0.449589312077f); // 146
  data.at(147)= AL::Math::Position2D(0.990177512169f, 0.867301404476f); // 147
  data.at(148)= AL::Math::Position2D(0.433017581701f, 0.529163002968f); // 148
  data.at(149)= AL::Math::Position2D(0.946043610573f, 0.685666263103f); // 149
  data.at(150)= AL::Math::Position2D(0.191918030381f, 0.342847526073f); // 150
  data.at(151)= AL::Math::Position2D(0.362669736147f, 0.19412624836f); // 151
  data.at(152)= AL::Math::Position2D(0.114387981594f, 0.192680731416f); // 152
  data.at(153)= AL::Math::Position2D(0.529534041882f, 0.215999409556f); // 153
  data.at(154)= AL::Math::Position2D(0.779931604862f, 0.356532096863f); // 154
  data.at(155)= AL::Math::Position2D(0.570464074612f, 0.322263300419f); // 155
  data.at(156)= AL::Math::Position2D(0.389661371708f, 0.561268568039f); // 156
  data.at(157)= AL::Math::Position2D(0.767404139042f, 0.821297764778f); // 157
  data.at(158)= AL::Math::Position2D(0.16513967514f, 0.822518110275f); // 158
  data.at(159)= AL::Math::Position2D(0.753649115562f, 0.216685131192f); // 159
  data.at(160)= AL::Math::Position2D(0.644758999348f, 0.340874224901f); // 160
  data.at(161)= AL::Math::Position2D(0.713824629784f, 0.187717109919f); // 161
  data.at(162)= AL::Math::Position2D(0.0156610645354f, 0.757755875587f); // 162
  data.at(163)= AL::Math::Position2D(0.0509978570044f, 0.680421829224f); // 163
  data.at(164)= AL::Math::Position2D(0.94623196125f, 0.933996260166f); // 164
  data.at(165)= AL::Math::Position2D(0.979192078114f, 0.176652997732f); // 165
  data.at(166)= AL::Math::Position2D(0.845599412918f, 0.704173028469f); // 166
  data.at(167)= AL::Math::Position2D(0.229802981019f, 0.350813001394f); // 167
  data.at(168)= AL::Math::Position2D(0.18767003715f, 0.0915888473392f); // 168
  data.at(169)= AL::Math::Position2D(0.751757860184f, 0.615651249886f); // 169
  data.at(170)= AL::Math::Position2D(0.424981683493f, 0.52426224947f); // 170
  data.at(171)= AL::Math::Position2D(0.679110169411f, 0.083636239171f); // 171
  data.at(172)= AL::Math::Position2D(0.269509881735f, 0.345141202211f); // 172
  data.at(173)= AL::Math::Position2D(0.0807877033949f, 0.0632751658559f); // 173
  data.at(174)= AL::Math::Position2D(0.819520354271f, 0.341018736362f); // 174
  data.at(175)= AL::Math::Position2D(0.683601498604f, 0.349919199944f); // 175
  data.at(176)= AL::Math::Position2D(0.76752692461f, 0.898188352585f); // 176
  data.at(177)= AL::Math::Position2D(0.64547008276f, 0.760661959648f); // 177
  data.at(178)= AL::Math::Position2D(0.0497447438538f, 0.288666605949f); // 178
  data.at(179)= AL::Math::Position2D(0.409064024687f, 0.444813996553f); // 179
  data.at(180)= AL::Math::Position2D(0.926204264164f, 0.111002720892f); // 180
  data.at(181)= AL::Math::Position2D(0.0524270012975f, 0.774621784687f); // 181
  data.at(182)= AL::Math::Position2D(0.822618484497f, 0.78802216053f); // 182
  data.at(183)= AL::Math::Position2D(0.765956878662f, 0.498249351978f); // 183
  data.at(184)= AL::Math::Position2D(0.0877697616816f, 0.0778427943587f); // 184
  data.at(185)= AL::Math::Position2D(0.323877334595f, 0.110121913254f); // 185
  data.at(186)= AL::Math::Position2D(0.0524193495512f, 0.267403692007f); // 186
  data.at(187)= AL::Math::Position2D(0.935426294804f, 0.00498818838969f); // 187
  data.at(188)= AL::Math::Position2D(0.312673449516f, 0.268988519907f); // 188
  data.at(189)= AL::Math::Position2D(0.440490990877f, 0.887975096703f); // 189
  data.at(190)= AL::Math::Position2D(0.433886349201f, 0.256473600864f); // 190
  data.at(191)= AL::Math::Position2D(0.253272563219f, 0.719524085522f); // 191
  data.at(192)= AL::Math::Position2D(0.601948678493f, 0.387998729944f); // 192
  data.at(193)= AL::Math::Position2D(0.921154737473f, 0.734603404999f); // 193
  data.at(194)= AL::Math::Position2D(0.339146018028f, 0.990621030331f); // 194
  data.at(195)= AL::Math::Position2D(0.290259569883f, 0.90013474226f); // 195
  data.at(196)= AL::Math::Position2D(0.803870916367f, 0.953724741936f); // 196
  data.at(197)= AL::Math::Position2D(0.93191242218f, 0.191294059157f); // 197
  data.at(198)= AL::Math::Position2D(0.585320115089f, 0.858949124813f); // 198
  data.at(199)= AL::Math::Position2D(0.984850883484f, 0.714115381241f); // 199
  data.at(200)= AL::Math::Position2D(0.108619257808f, 0.604779422283f); // 200
  data.at(201)= AL::Math::Position2D(0.75844925642f, 0.686398565769f); // 201
  data.at(202)= AL::Math::Position2D(0.310820281506f, 0.761824965477f); // 202
  data.at(203)= AL::Math::Position2D(0.0657907351851f, 0.0132551668212f); // 203
  data.at(204)= AL::Math::Position2D(0.230647280812f, 0.284997105598f); // 204
  data.at(205)= AL::Math::Position2D(0.978518366814f, 0.648049294949f); // 205
  data.at(206)= AL::Math::Position2D(0.0750685930252f, 0.897846281528f); // 206
  data.at(207)= AL::Math::Position2D(0.867860376835f, 0.965100824833f); // 207
  data.at(208)= AL::Math::Position2D(0.166884586215f, 0.733925819397f); // 208
  data.at(209)= AL::Math::Position2D(0.982153892517f, 0.71132183075f); // 209
  data.at(210)= AL::Math::Position2D(0.868165910244f, 0.149924516678f); // 210
  data.at(211)= AL::Math::Position2D(0.772591173649f, 0.381329298019f); // 211
  data.at(212)= AL::Math::Position2D(0.83798199892f, 0.510673105717f); // 212
  data.at(213)= AL::Math::Position2D(0.434865325689f, 0.969035089016f); // 213
  data.at(214)= AL::Math::Position2D(0.074248097837f, 0.214251950383f); // 214
  data.at(215)= AL::Math::Position2D(0.51963365078f, 0.64869582653f); // 215
  data.at(216)= AL::Math::Position2D(0.984986245632f, 0.367391645908f); // 216
  data.at(217)= AL::Math::Position2D(0.897328019142f, 0.177170157433f); // 217
  data.at(218)= AL::Math::Position2D(0.226120173931f, 0.141343727708f); // 218
  data.at(219)= AL::Math::Position2D(0.776619315147f, 0.857803761959f); // 219
  data.at(220)= AL::Math::Position2D(0.951362252235f, 0.0719318166375f); // 220
  data.at(221)= AL::Math::Position2D(0.399882555008f, 0.546809613705f); // 221
  data.at(222)= AL::Math::Position2D(0.97564291954f, 0.068302102387f); // 222
  data.at(223)= AL::Math::Position2D(0.0600154772401f, 0.733639240265f); // 223
  data.at(224)= AL::Math::Position2D(0.477629780769f, 0.509079396725f); // 224
  data.at(225)= AL::Math::Position2D(0.294104456902f, 0.00691156415269f); // 225
  data.at(226)= AL::Math::Position2D(0.447309583426f, 0.95877969265f); // 226
  data.at(227)= AL::Math::Position2D(0.16851632297f, 0.816543936729f); // 227
  data.at(228)= AL::Math::Position2D(0.361156851053f, 0.517720401287f); // 228
  data.at(229)= AL::Math::Position2D(0.43924805522f, 0.267927378416f); // 229
  data.at(230)= AL::Math::Position2D(0.783248245716f, 0.554809808731f); // 230
  data.at(231)= AL::Math::Position2D(0.359850615263f, 0.360028117895f); // 231
  data.at(232)= AL::Math::Position2D(0.591669142246f, 0.608714580536f); // 232
  data.at(233)= AL::Math::Position2D(0.851435482502f, 0.723512768745f); // 233
  data.at(234)= AL::Math::Position2D(0.897290766239f, 0.68868970871f); // 234
  data.at(235)= AL::Math::Position2D(0.851080954075f, 0.653697013855f); // 235
  data.at(236)= AL::Math::Position2D(0.119193851948f, 0.76022195816f); // 236
  data.at(237)= AL::Math::Position2D(0.0844513624907f, 0.274660497904f); // 237
  data.at(238)= AL::Math::Position2D(0.934788525105f, 0.122784487903f); // 238
  data.at(239)= AL::Math::Position2D(0.660516440868f, 0.343486458063f); // 239
  data.at(240)= AL::Math::Position2D(0.971240222454f, 0.461707949638f); // 240
  data.at(241)= AL::Math::Position2D(0.583421170712f, 0.0530866459012f); // 241
  data.at(242)= AL::Math::Position2D(0.0522355288267f, 0.34949785471f); // 242
  data.at(243)= AL::Math::Position2D(0.897863149643f, 0.336856663227f); // 243
  data.at(244)= AL::Math::Position2D(0.92257553339f, 0.171565413475f); // 244
  data.at(245)= AL::Math::Position2D(0.408074349165f, 0.848014235497f); // 245
  data.at(246)= AL::Math::Position2D(0.817167937756f, 0.0825645923615f); // 246
  data.at(247)= AL::Math::Position2D(0.667625665665f, 0.806687176228f); // 247
  data.at(248)= AL::Math::Position2D(0.691732347012f, 0.137478724122f); // 248
  data.at(249)= AL::Math::Position2D(0.39248085022f, 0.432448327541f); // 249
  data.at(250)= AL::Math::Position2D(0.5068141222f, 0.193209588528f); // 250
  data.at(251)= AL::Math::Position2D(0.288315117359f, 0.346813738346f); // 251
  data.at(252)= AL::Math::Position2D(0.733943462372f, 0.924195408821f); // 252
  data.at(253)= AL::Math::Position2D(0.174568772316f, 0.0319439955056f); // 253
  data.at(254)= AL::Math::Position2D(0.164299145341f, 0.810794711113f); // 254
  data.at(255)= AL::Math::Position2D(0.0084721930325f, 0.562394440174f); // 255
  data.at(256)= AL::Math::Position2D(0.992915213108f, 0.835575520992f); // 256
  data.at(257)= AL::Math::Position2D(0.833559155464f, 0.848595798016f); // 257
  data.at(258)= AL::Math::Position2D(0.475522309542f, 0.246211439371f); // 258
  data.at(259)= AL::Math::Position2D(0.924625396729f, 0.24537910521f); // 259
  data.at(260)= AL::Math::Position2D(0.498999118805f, 0.852951645851f); // 260
  data.at(261)= AL::Math::Position2D(0.457086831331f, 0.63721293211f); // 261
  data.at(262)= AL::Math::Position2D(0.241024762392f, 0.420816808939f); // 262
  data.at(263)= AL::Math::Position2D(0.872329175472f, 0.90053999424f); // 263
  data.at(264)= AL::Math::Position2D(0.152481496334f, 0.234998196363f); // 264
  data.at(265)= AL::Math::Position2D(0.135699376464f, 0.596186280251f); // 265
  data.at(266)= AL::Math::Position2D(0.366705685854f, 0.184456512332f); // 266
  data.at(267)= AL::Math::Position2D(0.568740367889f, 0.0868887528777f); // 267
  data.at(268)= AL::Math::Position2D(0.530089795589f, 0.675582826138f); // 268
  data.at(269)= AL::Math::Position2D(0.81225079298f, 0.316448509693f); // 269
  data.at(270)= AL::Math::Position2D(0.729995727539f, 0.770072042942f); // 270
  data.at(271)= AL::Math::Position2D(0.578355431557f, 0.617463707924f); // 271
  data.at(272)= AL::Math::Position2D(0.243669643998f, 0.862336337566f); // 272
  data.at(273)= AL::Math::Position2D(0.594406545162f, 0.871572434902f); // 273
  data.at(274)= AL::Math::Position2D(0.156982436776f, 0.882327854633f); // 274
  data.at(275)= AL::Math::Position2D(0.740485966206f, 0.705927252769f); // 275
  data.at(276)= AL::Math::Position2D(0.649007499218f, 0.616582393646f); // 276
  data.at(277)= AL::Math::Position2D(0.819022417068f, 0.121223241091f); // 277
  data.at(278)= AL::Math::Position2D(0.788683474064f, 0.715469062328f); // 278
  data.at(279)= AL::Math::Position2D(0.860756635666f, 0.128442063928f); // 279
  data.at(280)= AL::Math::Position2D(0.483156889677f, 0.777843296528f); // 280
  data.at(281)= AL::Math::Position2D(0.00881949812174f, 0.0656608566642f); // 281
  data.at(282)= AL::Math::Position2D(0.198441594839f, 0.647755146027f); // 282
  data.at(283)= AL::Math::Position2D(0.51615858078f, 0.256754547358f); // 283
  data.at(284)= AL::Math::Position2D(0.417298346758f, 0.862604320049f); // 284
  data.at(285)= AL::Math::Position2D(0.79478764534f, 0.0634765028954f); // 285
  data.at(286)= AL::Math::Position2D(0.0726884379983f, 0.886295318604f); // 286
  data.at(287)= AL::Math::Position2D(0.958180248737f, 0.133348062634f); // 287
  data.at(288)= AL::Math::Position2D(0.555819630623f, 0.0506741702557f); // 288
  data.at(289)= AL::Math::Position2D(0.353787481785f, 0.589080750942f); // 289
  data.at(290)= AL::Math::Position2D(0.628144145012f, 0.98948097229f); // 290
  data.at(291)= AL::Math::Position2D(0.351041704416f, 0.0243733432144f); // 291
  data.at(292)= AL::Math::Position2D(0.442582905293f, 0.0410497747362f); // 292
  data.at(293)= AL::Math::Position2D(0.0243531819433f, 0.882907032967f); // 293
  data.at(294)= AL::Math::Position2D(0.723506391048f, 0.822690665722f); // 294
  data.at(295)= AL::Math::Position2D(0.71434378624f, 0.743813335896f); // 295
  data.at(296)= AL::Math::Position2D(0.685433983803f, 0.213927194476f); // 296
  data.at(297)= AL::Math::Position2D(0.490817219019f, 0.144152879715f); // 297
  data.at(298)= AL::Math::Position2D(0.87025141716f, 0.9361795187f); // 298
  data.at(299)= AL::Math::Position2D(0.228219807148f, 0.549289762974f); // 299
  data.at(300)= AL::Math::Position2D(0.600999176502f, 0.569524288177f); // 300
  data.at(301)= AL::Math::Position2D(0.955596148968f, 0.59081608057f); // 301
  data.at(302)= AL::Math::Position2D(0.54510229826f, 0.422059029341f); // 302
  data.at(303)= AL::Math::Position2D(0.19594578445f, 0.645318448544f); // 303
  data.at(304)= AL::Math::Position2D(0.0166922919452f, 0.813899874687f); // 304
  data.at(305)= AL::Math::Position2D(0.555337131023f, 0.390572488308f); // 305
  data.at(306)= AL::Math::Position2D(0.311791628599f, 0.77017980814f); // 306
  data.at(307)= AL::Math::Position2D(0.778568208218f, 0.467027664185f); // 307
  data.at(308)= AL::Math::Position2D(0.669553875923f, 0.951919913292f); // 308
  data.at(309)= AL::Math::Position2D(0.246700927615f, 0.412994116545f); // 309
  data.at(310)= AL::Math::Position2D(0.150483816862f, 0.839860916138f); // 310
  data.at(311)= AL::Math::Position2D(0.399312019348f, 0.674076735973f); // 311
  data.at(312)= AL::Math::Position2D(0.458154290915f, 0.897298872471f); // 312
  data.at(313)= AL::Math::Position2D(0.8419033885f, 0.702567815781f); // 313
  data.at(314)= AL::Math::Position2D(0.189851403236f, 0.970901489258f); // 314
  data.at(315)= AL::Math::Position2D(0.873166143894f, 0.541228830814f); // 315
  data.at(316)= AL::Math::Position2D(0.738241016865f, 0.733038842678f); // 316
  data.at(317)= AL::Math::Position2D(0.11172554642f, 0.565614640713f); // 317
  data.at(318)= AL::Math::Position2D(0.06194569543f, 0.177018523216f); // 318
  data.at(319)= AL::Math::Position2D(0.641288280487f, 0.487526655197f); // 319
  data.at(320)= AL::Math::Position2D(0.545871436596f, 0.688457131386f); // 320
  data.at(321)= AL::Math::Position2D(0.267493367195f, 0.88139373064f); // 321
  data.at(322)= AL::Math::Position2D(0.958615541458f, 0.662721395493f); // 322
  data.at(323)= AL::Math::Position2D(0.337330579758f, 0.323455423117f); // 323
  data.at(324)= AL::Math::Position2D(0.107116870582f, 0.79012298584f); // 324
  data.at(325)= AL::Math::Position2D(0.264728754759f, 0.0266267545521f); // 325
  data.at(326)= AL::Math::Position2D(0.245021671057f, 0.878745734692f); // 326
  data.at(327)= AL::Math::Position2D(0.726959824562f, 0.0297696068883f); // 327
  data.at(328)= AL::Math::Position2D(0.416102170944f, 0.445342212915f); // 328
  data.at(329)= AL::Math::Position2D(0.351203680038f, 0.226487502456f); // 329
  data.at(330)= AL::Math::Position2D(0.0962453261018f, 0.509014487267f); // 330
  data.at(331)= AL::Math::Position2D(0.758263587952f, 0.690127551556f); // 331
  data.at(332)= AL::Math::Position2D(0.841219007969f, 0.684116184711f); // 332
  data.at(333)= AL::Math::Position2D(0.266535431147f, 0.483362853527f); // 333
  data.at(334)= AL::Math::Position2D(0.634849369526f, 0.225689664483f); // 334
  data.at(335)= AL::Math::Position2D(0.760549008846f, 0.341800868511f); // 335
  data.at(336)= AL::Math::Position2D(0.791939735413f, 0.917514979839f); // 336
  data.at(337)= AL::Math::Position2D(0.617906153202f, 0.132213875651f); // 337
  data.at(338)= AL::Math::Position2D(0.641634762287f, 0.0187755059451f); // 338
  data.at(339)= AL::Math::Position2D(0.328074753284f, 0.124489858747f); // 339
  data.at(340)= AL::Math::Position2D(0.743274450302f, 0.549326598644f); // 340
  data.at(341)= AL::Math::Position2D(0.376974612474f, 0.945349633694f); // 341
  data.at(342)= AL::Math::Position2D(0.676274359226f, 0.272828400135f); // 342
  data.at(343)= AL::Math::Position2D(0.0228600353003f, 0.848699808121f); // 343
  data.at(344)= AL::Math::Position2D(0.33963394165f, 0.356810003519f); // 344
  data.at(345)= AL::Math::Position2D(0.181425854564f, 0.0267671551555f); // 345
  data.at(346)= AL::Math::Position2D(0.924302101135f, 0.0963016971946f); // 346
  data.at(347)= AL::Math::Position2D(0.70917993784f, 0.120164580643f); // 347
  data.at(348)= AL::Math::Position2D(0.554241120815f, 0.939854025841f); // 348
  data.at(349)= AL::Math::Position2D(0.0854607149959f, 0.90959841013f); // 349
  data.at(350)= AL::Math::Position2D(0.487067073584f, 0.946439504623f); // 350
  data.at(351)= AL::Math::Position2D(0.932199180126f, 0.19541144371f); // 351
  data.at(352)= AL::Math::Position2D(0.0861306935549f, 0.578774154186f); // 352
  data.at(353)= AL::Math::Position2D(0.736771464348f, 0.978683233261f); // 353
  data.at(354)= AL::Math::Position2D(0.556062459946f, 0.0541491732001f); // 354
  data.at(355)= AL::Math::Position2D(0.219582006335f, 0.664316475391f); // 355
  data.at(356)= AL::Math::Position2D(0.550499618053f, 0.965996623039f); // 356
  data.at(357)= AL::Math::Position2D(0.282671689987f, 0.998535573483f); // 357
  data.at(358)= AL::Math::Position2D(0.556283116341f, 0.463340640068f); // 358
  data.at(359)= AL::Math::Position2D(0.431702941656f, 0.591323018074f); // 359
  data.at(360)= AL::Math::Position2D(0.640181601048f, 0.456518858671f); // 360
  data.at(361)= AL::Math::Position2D(0.777863681316f, 0.167209088802f); // 361
  data.at(362)= AL::Math::Position2D(0.648478806019f, 0.485707074404f); // 362
  data.at(363)= AL::Math::Position2D(0.582629203796f, 0.524883806705f); // 363
  data.at(364)= AL::Math::Position2D(0.491673856974f, 0.425614565611f); // 364
  data.at(365)= AL::Math::Position2D(0.684038639069f, 0.480253338814f); // 365
  data.at(366)= AL::Math::Position2D(0.924008488655f, 0.998571097851f); // 366
  data.at(367)= AL::Math::Position2D(0.888794541359f, 0.345156401396f); // 367
  data.at(368)= AL::Math::Position2D(0.979946136475f, 0.607314646244f); // 368
  data.at(369)= AL::Math::Position2D(0.329945206642f, 0.16275216639f); // 369
  data.at(370)= AL::Math::Position2D(0.59205198288f, 0.0543661303818f); // 370
  data.at(371)= AL::Math::Position2D(0.742443740368f, 0.249981597066f); // 371
  data.at(372)= AL::Math::Position2D(0.852424740791f, 0.667489230633f); // 372
  data.at(373)= AL::Math::Position2D(0.353469699621f, 0.289063960314f); // 373
  data.at(374)= AL::Math::Position2D(0.973187685013f, 0.754120230675f); // 374
  data.at(375)= AL::Math::Position2D(0.134982272983f, 0.927737116814f); // 375
  data.at(376)= AL::Math::Position2D(0.0211075972766f, 0.40775462985f); // 376
  data.at(377)= AL::Math::Position2D(0.515508294106f, 0.00997465383261f); // 377
  data.at(378)= AL::Math::Position2D(0.239845842123f, 0.683652222157f); // 378
  data.at(379)= AL::Math::Position2D(0.688071846962f, 0.974404811859f); // 379
  data.at(380)= AL::Math::Position2D(0.841576218605f, 0.57481700182f); // 380
  data.at(381)= AL::Math::Position2D(0.172208204865f, 0.948233366013f); // 381
  data.at(382)= AL::Math::Position2D(0.665061056614f, 0.40829077363f); // 382
  data.at(383)= AL::Math::Position2D(0.361408442259f, 0.539335906506f); // 383
  data.at(384)= AL::Math::Position2D(0.654330193996f, 0.906513094902f); // 384
  data.at(385)= AL::Math::Position2D(0.939723372459f, 0.502923309803f); // 385
  data.at(386)= AL::Math::Position2D(0.645104706287f, 0.991178035736f); // 386
  data.at(387)= AL::Math::Position2D(0.339811176062f, 0.727249801159f); // 387
  data.at(388)= AL::Math::Position2D(0.0399396158755f, 0.430503308773f); // 388
  data.at(389)= AL::Math::Position2D(0.0513412244618f, 0.800426840782f); // 389
  data.at(390)= AL::Math::Position2D(0.858141124249f, 0.333698064089f); // 390
  data.at(391)= AL::Math::Position2D(0.0349585935473f, 0.464725166559f); // 391
  data.at(392)= AL::Math::Position2D(0.490186452866f, 0.271073609591f); // 392
  data.at(393)= AL::Math::Position2D(0.876362144947f, 0.569648087025f); // 393
  data.at(394)= AL::Math::Position2D(0.560409426689f, 0.0680409967899f); // 394
  data.at(395)= AL::Math::Position2D(0.620566785336f, 0.348545461893f); // 395
  data.at(396)= AL::Math::Position2D(0.112429141998f, 0.934626340866f); // 396
  data.at(397)= AL::Math::Position2D(0.528553009033f, 0.896250724792f); // 397
  data.at(398)= AL::Math::Position2D(0.842514753342f, 0.462469249964f); // 398
  data.at(399)= AL::Math::Position2D(0.555317521095f, 0.623792231083f); // 399
  data.at(400)= AL::Math::Position2D(0.604129016399f, 0.255736559629f); // 400
  data.at(401)= AL::Math::Position2D(0.442647725344f, 0.672437012196f); // 401
  data.at(402)= AL::Math::Position2D(0.889971911907f, 0.737260878086f); // 402
  data.at(403)= AL::Math::Position2D(0.735423922539f, 0.478100389242f); // 403
  data.at(404)= AL::Math::Position2D(0.060295663774f, 0.236310362816f); // 404
  data.at(405)= AL::Math::Position2D(0.639677941799f, 0.443281114101f); // 405
  data.at(406)= AL::Math::Position2D(0.46538233757f, 0.59619474411f); // 406
  data.at(407)= AL::Math::Position2D(0.122602820396f, 0.236894547939f); // 407
  data.at(408)= AL::Math::Position2D(0.217398002744f, 0.169873818755f); // 408
  data.at(409)= AL::Math::Position2D(0.0720723941922f, 0.431099802256f); // 409
  data.at(410)= AL::Math::Position2D(0.287338376045f, 0.0689571201801f); // 410
  data.at(411)= AL::Math::Position2D(0.719866812229f, 0.61157989502f); // 411
  data.at(412)= AL::Math::Position2D(0.993398189545f, 0.375781625509f); // 412
  data.at(413)= AL::Math::Position2D(0.0380190052092f, 0.261122137308f); // 413
  data.at(414)= AL::Math::Position2D(0.510216534138f, 0.318167060614f); // 414
  data.at(415)= AL::Math::Position2D(0.0827377960086f, 0.399522215128f); // 415
  data.at(416)= AL::Math::Position2D(0.635922908783f, 0.253016829491f); // 416
  data.at(417)= AL::Math::Position2D(0.821371853352f, 0.919616699219f); // 417
  data.at(418)= AL::Math::Position2D(0.393646121025f, 0.232085138559f); // 418
  data.at(419)= AL::Math::Position2D(0.111849859357f, 0.960625886917f); // 419
  data.at(420)= AL::Math::Position2D(0.289876937866f, 0.54313069582f); // 420
  data.at(421)= AL::Math::Position2D(0.614639282227f, 0.219711765647f); // 421
  data.at(422)= AL::Math::Position2D(0.276595920324f, 0.356507927179f); // 422
  data.at(423)= AL::Math::Position2D(0.89056456089f, 0.791910409927f); // 423
  data.at(424)= AL::Math::Position2D(0.704877197742f, 0.28718021512f); // 424
  data.at(425)= AL::Math::Position2D(0.355591028929f, 0.150796204805f); // 425
  data.at(426)= AL::Math::Position2D(0.382448464632f, 0.0380488485098f); // 426
  data.at(427)= AL::Math::Position2D(0.24619191885f, 0.9929048419f); // 427
  data.at(428)= AL::Math::Position2D(0.868934988976f, 0.947203218937f); // 428
  data.at(429)= AL::Math::Position2D(0.875464916229f, 0.473717302084f); // 429
  data.at(430)= AL::Math::Position2D(0.146101146936f, 0.16473634541f); // 430
  data.at(431)= AL::Math::Position2D(0.452114790678f, 0.355524122715f); // 431
  data.at(432)= AL::Math::Position2D(0.792818427086f, 0.192713573575f); // 432
  data.at(433)= AL::Math::Position2D(0.285854876041f, 0.88222771883f); // 433
  data.at(434)= AL::Math::Position2D(0.810318529606f, 0.924976527691f); // 434
  data.at(435)= AL::Math::Position2D(0.515755593777f, 0.154542505741f); // 435
  data.at(436)= AL::Math::Position2D(0.501412928104f, 0.659504652023f); // 436
  data.at(437)= AL::Math::Position2D(0.599516034126f, 0.270863413811f); // 437
  data.at(438)= AL::Math::Position2D(0.365639448166f, 0.779304265976f); // 438
  data.at(439)= AL::Math::Position2D(0.390194118023f, 0.780576825142f); // 439
  data.at(440)= AL::Math::Position2D(0.454901754856f, 0.086322709918f); // 440
  data.at(441)= AL::Math::Position2D(0.23312497139f, 0.799032628536f); // 441
  data.at(442)= AL::Math::Position2D(0.0572920180857f, 0.849983692169f); // 442
  data.at(443)= AL::Math::Position2D(0.809768021107f, 0.507057905197f); // 443
  data.at(444)= AL::Math::Position2D(0.336184322834f, 0.154361948371f); // 444
  data.at(445)= AL::Math::Position2D(0.676124513149f, 0.237214550376f); // 445
  data.at(446)= AL::Math::Position2D(0.313751935959f, 0.0935736820102f); // 446
  data.at(447)= AL::Math::Position2D(0.541179835796f, 0.161069914699f); // 447
  data.at(448)= AL::Math::Position2D(0.563160836697f, 0.0702216103673f); // 448
  data.at(449)= AL::Math::Position2D(0.437919467688f, 0.936292469501f); // 449
  data.at(450)= AL::Math::Position2D(0.353640794754f, 0.162997111678f); // 450
  data.at(451)= AL::Math::Position2D(0.571450173855f, 0.873709261417f); // 451
  data.at(452)= AL::Math::Position2D(0.960599184036f, 0.988342642784f); // 452
  data.at(453)= AL::Math::Position2D(0.568540990353f, 0.221804648638f); // 453
  data.at(454)= AL::Math::Position2D(0.325752586126f, 0.851882278919f); // 454
  data.at(455)= AL::Math::Position2D(0.30856359005f, 0.0279845073819f); // 455
  data.at(456)= AL::Math::Position2D(0.929974198341f, 0.130643308163f); // 456
  data.at(457)= AL::Math::Position2D(0.804189264774f, 0.91606003046f); // 457
  data.at(458)= AL::Math::Position2D(0.772240459919f, 0.94375705719f); // 458
  data.at(459)= AL::Math::Position2D(0.136489391327f, 0.773236095905f); // 459
  data.at(460)= AL::Math::Position2D(0.211272597313f, 0.917190313339f); // 460
  data.at(461)= AL::Math::Position2D(0.509991943836f, 0.0624229833484f); // 461
  data.at(462)= AL::Math::Position2D(0.766973674297f, 0.0810660421848f); // 462
  data.at(463)= AL::Math::Position2D(0.786211252213f, 0.0791907981038f); // 463
  data.at(464)= AL::Math::Position2D(0.243661776185f, 0.39935401082f); // 464
  data.at(465)= AL::Math::Position2D(0.283813565969f, 0.429520219564f); // 465
  data.at(466)= AL::Math::Position2D(0.799682199955f, 0.467002898455f); // 466
  data.at(467)= AL::Math::Position2D(0.836888194084f, 0.393895804882f); // 467
  data.at(468)= AL::Math::Position2D(0.124957814813f, 0.333078622818f); // 468
  data.at(469)= AL::Math::Position2D(0.432113438845f, 0.58502560854f); // 469
  data.at(470)= AL::Math::Position2D(0.432923465967f, 0.474428236485f); // 470
  data.at(471)= AL::Math::Position2D(0.617922008038f, 0.841449201107f); // 471
  data.at(472)= AL::Math::Position2D(0.237942144275f, 0.826228201389f); // 472
  data.at(473)= AL::Math::Position2D(0.54993391037f, 0.27848470211f); // 473
  data.at(474)= AL::Math::Position2D(0.872800648212f, 0.417367994785f); // 474
  data.at(475)= AL::Math::Position2D(0.610759854317f, 0.695105850697f); // 475
  data.at(476)= AL::Math::Position2D(0.555864810944f, 0.60990524292f); // 476
  data.at(477)= AL::Math::Position2D(0.404489725828f, 0.63298368454f); // 477
  data.at(478)= AL::Math::Position2D(0.0169893670827f, 0.195698618889f); // 478
  data.at(479)= AL::Math::Position2D(0.293355882168f, 0.662914931774f); // 479
  data.at(480)= AL::Math::Position2D(0.311160475016f, 0.722516775131f); // 480
  data.at(481)= AL::Math::Position2D(0.232179835439f, 0.195595115423f); // 481
  data.at(482)= AL::Math::Position2D(0.0703662335873f, 0.913955628872f); // 482
  data.at(483)= AL::Math::Position2D(0.62164747715f, 0.198680818081f); // 483
  data.at(484)= AL::Math::Position2D(0.265581190586f, 0.972025692463f); // 484
  data.at(485)= AL::Math::Position2D(0.469515949488f, 0.193888187408f); // 485
  data.at(486)= AL::Math::Position2D(0.877720594406f, 0.359540313482f); // 486
  data.at(487)= AL::Math::Position2D(0.671588301659f, 0.806265056133f); // 487
  data.at(488)= AL::Math::Position2D(0.721205234528f, 0.63310956955f); // 488
  data.at(489)= AL::Math::Position2D(0.0357954017818f, 0.625014901161f); // 489
  data.at(490)= AL::Math::Position2D(0.473149746656f, 0.0379095412791f); // 490
  data.at(491)= AL::Math::Position2D(0.395418405533f, 0.172190919518f); // 491
  data.at(492)= AL::Math::Position2D(0.837752699852f, 0.190486490726f); // 492
  data.at(493)= AL::Math::Position2D(0.00361267360859f, 0.400709927082f); // 493
  data.at(494)= AL::Math::Position2D(0.262716323137f, 0.972920238972f); // 494
  data.at(495)= AL::Math::Position2D(0.429113119841f, 0.890528857708f); // 495
  data.at(496)= AL::Math::Position2D(0.595228374004f, 0.678937017918f); // 496
  data.at(497)= AL::Math::Position2D(0.0212360564619f, 0.957652509212f); // 497
  data.at(498)= AL::Math::Position2D(0.901395618916f, 0.211026832461f); // 498
  data.at(499)= AL::Math::Position2D(0.869735181332f, 0.579254627228f); // 499

//  std::vector<AL::Math::Position2D> pPoints;
//  AL::Math::removeAlignedPoint(data, pPoints);
//  EXPECT_EQ(13, (int)pPoints.size());

//  std::vector<AL::Math::Position2D> result = AL::Math::getConvexHull(data);
//  EXPECT_EQ(14, (int)result.size());

//  AL::Math::deleteDoublesInNoneSortVector(data);
//  EXPECT_EQ(500, (int)data.size());

  // Usefull for plot in python
  //    for (unsigned int i=0; i<result.size(); i++)
  //    {
  //      std::cout << "data.append(almath.Position2D("
  //      << result.at(i).x << ", " << result.at(i).y << "))" << std::endl;
  //    }

}


//TEST(ConvexhullTest, randomData2)
//{
//  std::vector<AL::Math::Position2D> data;
//  data.resize(400);
//  data.at(0)= AL::Math::Position2D(-1.0f, -0.446804851294f); // 0
//  data.at(1)= AL::Math::Position2D(1.0f, 0.831940710545f); // 1
//  data.at(2)= AL::Math::Position2D(-0.198347881436f, -1.0f); // 2
//  data.at(3)= AL::Math::Position2D(-0.122689545155f, 1.0f); // 3
//  data.at(4)= AL::Math::Position2D(-1.0f, -0.10264210403f); // 4
//  data.at(5)= AL::Math::Position2D(1.0f, -0.894295811653f); // 5
//  data.at(6)= AL::Math::Position2D(0.328404426575f, -1.0f); // 6
//  data.at(7)= AL::Math::Position2D(0.938923358917f, 1.0f); // 7
//  data.at(8)= AL::Math::Position2D(-1.0f, 0.219311073422f); // 8
//  data.at(9)= AL::Math::Position2D(1.0f, 0.367222636938f); // 9
//  data.at(10)= AL::Math::Position2D(0.580633223057f, -1.0f); // 10
//  data.at(11)= AL::Math::Position2D(-0.347913563251f, 1.0f); // 11
//  data.at(12)= AL::Math::Position2D(-1.0f, -0.603081882f); // 12
//  data.at(13)= AL::Math::Position2D(1.0f, -0.564284265041f); // 13
//  data.at(14)= AL::Math::Position2D(0.567598342896f, -1.0f); // 14
//  data.at(15)= AL::Math::Position2D(-0.243827432394f, 1.0f); // 15
//  data.at(16)= AL::Math::Position2D(-1.0f, -0.405109912157f); // 16
//  data.at(17)= AL::Math::Position2D(1.0f, 0.490473687649f); // 17
//  data.at(18)= AL::Math::Position2D(-0.051644705236f, -1.0f); // 18
//  data.at(19)= AL::Math::Position2D(-0.570603251457f, 1.0f); // 19
//  data.at(20)= AL::Math::Position2D(-1.0f, 0.126282319427f); // 20
//  data.at(21)= AL::Math::Position2D(1.0f, -0.427623778582f); // 21
//  data.at(22)= AL::Math::Position2D(0.639392793179f, -1.0f); // 22
//  data.at(23)= AL::Math::Position2D(-0.820161640644f, 1.0f); // 23
//  data.at(24)= AL::Math::Position2D(-1.0f, -0.214604735374f); // 24
//  data.at(25)= AL::Math::Position2D(1.0f, 0.676067650318f); // 25
//  data.at(26)= AL::Math::Position2D(-0.715437293053f, -1.0f); // 26
//  data.at(27)= AL::Math::Position2D(-0.0971464663744f, 1.0f); // 27
//  data.at(28)= AL::Math::Position2D(-1.0f, -0.504795968533f); // 28
//  data.at(29)= AL::Math::Position2D(1.0f, -0.805170297623f); // 29
//  data.at(30)= AL::Math::Position2D(0.255645334721f, -1.0f); // 30
//  data.at(31)= AL::Math::Position2D(0.519216895103f, 1.0f); // 31
//  data.at(32)= AL::Math::Position2D(-1.0f, 0.940836131573f); // 32
//  data.at(33)= AL::Math::Position2D(1.0f, 0.519241333008f); // 33
//  data.at(34)= AL::Math::Position2D(0.775143742561f, -1.0f); // 34
//  data.at(35)= AL::Math::Position2D(-0.0182752162218f, 1.0f); // 35
//  data.at(36)= AL::Math::Position2D(-1.0f, -0.855382680893f); // 36
//  data.at(37)= AL::Math::Position2D(1.0f, 0.876670539379f); // 37
//  data.at(38)= AL::Math::Position2D(0.0785210356116f, -1.0f); // 38
//  data.at(39)= AL::Math::Position2D(-0.650805592537f, 1.0f); // 39
//  data.at(40)= AL::Math::Position2D(-1.0f, -0.230878010392f); // 40
//  data.at(41)= AL::Math::Position2D(1.0f, 0.620259404182f); // 41
//  data.at(42)= AL::Math::Position2D(-0.415594786406f, -1.0f); // 42
//  data.at(43)= AL::Math::Position2D(-0.0357880964875f, 1.0f); // 43
//  data.at(44)= AL::Math::Position2D(-1.0f, -0.561155855656f); // 44
//  data.at(45)= AL::Math::Position2D(1.0f, 0.425160169601f); // 45
//  data.at(46)= AL::Math::Position2D(0.397685289383f, -1.0f); // 46
//  data.at(47)= AL::Math::Position2D(-0.271635770798f, 1.0f); // 47
//  data.at(48)= AL::Math::Position2D(-1.0f, 0.421557664871f); // 48
//  data.at(49)= AL::Math::Position2D(1.0f, -0.601459026337f); // 49
//  data.at(50)= AL::Math::Position2D(0.0724089816213f, -1.0f); // 50
//  data.at(51)= AL::Math::Position2D(-0.651813805103f, 1.0f); // 51
//  data.at(52)= AL::Math::Position2D(-1.0f, 0.0173651631922f); // 52
//  data.at(53)= AL::Math::Position2D(1.0f, -0.493654072285f); // 53
//  data.at(54)= AL::Math::Position2D(-0.226542130113f, -1.0f); // 54
//  data.at(55)= AL::Math::Position2D(0.999452471733f, 1.0f); // 55
//  data.at(56)= AL::Math::Position2D(-1.0f, -0.468002468348f); // 56
//  data.at(57)= AL::Math::Position2D(1.0f, 0.399474233389f); // 57
//  data.at(58)= AL::Math::Position2D(-0.709527671337f, -1.0f); // 58
//  data.at(59)= AL::Math::Position2D(-0.0704187527299f, 1.0f); // 59
//  data.at(60)= AL::Math::Position2D(-1.0f, -0.561790764332f); // 60
//  data.at(61)= AL::Math::Position2D(1.0f, -0.330915421247f); // 61
//  data.at(62)= AL::Math::Position2D(0.391036719084f, -1.0f); // 62
//  data.at(63)= AL::Math::Position2D(-0.620523393154f, 1.0f); // 63
//  data.at(64)= AL::Math::Position2D(-1.0f, 0.0376922599971f); // 64
//  data.at(65)= AL::Math::Position2D(1.0f, 0.268423467875f); // 65
//  data.at(66)= AL::Math::Position2D(-0.281400501728f, -1.0f); // 66
//  data.at(67)= AL::Math::Position2D(-0.0114410473034f, 1.0f); // 67
//  data.at(68)= AL::Math::Position2D(-1.0f, 0.0529104433954f); // 68
//  data.at(69)= AL::Math::Position2D(1.0f, 0.743270099163f); // 69
//  data.at(70)= AL::Math::Position2D(0.506779313087f, -1.0f); // 70
//  data.at(71)= AL::Math::Position2D(0.676407933235f, 1.0f); // 71
//  data.at(72)= AL::Math::Position2D(-1.0f, -0.214361384511f); // 72
//  data.at(73)= AL::Math::Position2D(1.0f, 0.442629098892f); // 73
//  data.at(74)= AL::Math::Position2D(-0.826814711094f, -1.0f); // 74
//  data.at(75)= AL::Math::Position2D(-0.582073271275f, 1.0f); // 75
//  data.at(76)= AL::Math::Position2D(-1.0f, 0.380002260208f); // 76
//  data.at(77)= AL::Math::Position2D(1.0f, -0.850284039974f); // 77
//  data.at(78)= AL::Math::Position2D(0.903813779354f, -1.0f); // 78
//  data.at(79)= AL::Math::Position2D(0.428320944309f, 1.0f); // 79
//  data.at(80)= AL::Math::Position2D(-1.0f, -0.686765193939f); // 80
//  data.at(81)= AL::Math::Position2D(1.0f, 0.407080739737f); // 81
//  data.at(82)= AL::Math::Position2D(0.954542517662f, -1.0f); // 82
//  data.at(83)= AL::Math::Position2D(-0.353682905436f, 1.0f); // 83
//  data.at(84)= AL::Math::Position2D(-1.0f, -0.0618978850543f); // 84
//  data.at(85)= AL::Math::Position2D(1.0f, -0.816445827484f); // 85
//  data.at(86)= AL::Math::Position2D(0.0806177556515f, -1.0f); // 86
//  data.at(87)= AL::Math::Position2D(-0.700315773487f, 1.0f); // 87
//  data.at(88)= AL::Math::Position2D(-1.0f, -0.488505989313f); // 88
//  data.at(89)= AL::Math::Position2D(1.0f, -0.637856125832f); // 89
//  data.at(90)= AL::Math::Position2D(0.567297697067f, -1.0f); // 90
//  data.at(91)= AL::Math::Position2D(-0.50206553936f, 1.0f); // 91
//  data.at(92)= AL::Math::Position2D(-1.0f, 0.813305437565f); // 92
//  data.at(93)= AL::Math::Position2D(1.0f, -0.00766856456175f); // 93
//  data.at(94)= AL::Math::Position2D(-0.0765049383044f, -1.0f); // 94
//  data.at(95)= AL::Math::Position2D(-0.546765387058f, 1.0f); // 95
//  data.at(96)= AL::Math::Position2D(-1.0f, -0.341696113348f); // 96
//  data.at(97)= AL::Math::Position2D(1.0f, 0.511673986912f); // 97
//  data.at(98)= AL::Math::Position2D(0.468416303396f, -1.0f); // 98
//  data.at(99)= AL::Math::Position2D(-0.585760891438f, 1.0f); // 99
//  data.at(100)= AL::Math::Position2D(-1.0f, -0.635513424873f); // 100
//  data.at(101)= AL::Math::Position2D(1.0f, 0.545143127441f); // 101
//  data.at(102)= AL::Math::Position2D(-0.673808455467f, -1.0f); // 102
//  data.at(103)= AL::Math::Position2D(-0.41132375598f, 1.0f); // 103
//  data.at(104)= AL::Math::Position2D(-1.0f, -0.858831524849f); // 104
//  data.at(105)= AL::Math::Position2D(1.0f, 0.0438254699111f); // 105
//  data.at(106)= AL::Math::Position2D(-0.358507752419f, -1.0f); // 106
//  data.at(107)= AL::Math::Position2D(0.698491156101f, 1.0f); // 107
//  data.at(108)= AL::Math::Position2D(-1.0f, -0.958425879478f); // 108
//  data.at(109)= AL::Math::Position2D(1.0f, 0.128347545862f); // 109
//  data.at(110)= AL::Math::Position2D(-0.219675481319f, -1.0f); // 110
//  data.at(111)= AL::Math::Position2D(-0.112247161567f, 1.0f); // 111
//  data.at(112)= AL::Math::Position2D(-1.0f, 0.372705072165f); // 112
//  data.at(113)= AL::Math::Position2D(1.0f, -0.613283038139f); // 113
//  data.at(114)= AL::Math::Position2D(0.115123219788f, -1.0f); // 114
//  data.at(115)= AL::Math::Position2D(0.919606804848f, 1.0f); // 115
//  data.at(116)= AL::Math::Position2D(-1.0f, -0.153443843126f); // 116
//  data.at(117)= AL::Math::Position2D(1.0f, 0.580471098423f); // 117
//  data.at(118)= AL::Math::Position2D(0.628908991814f, -1.0f); // 118
//  data.at(119)= AL::Math::Position2D(0.353942841291f, 1.0f); // 119
//  data.at(120)= AL::Math::Position2D(-1.0f, -0.586061537266f); // 120
//  data.at(121)= AL::Math::Position2D(1.0f, 0.78567135334f); // 121
//  data.at(122)= AL::Math::Position2D(0.958238124847f, -1.0f); // 122
//  data.at(123)= AL::Math::Position2D(-0.193119168282f, 1.0f); // 123
//  data.at(124)= AL::Math::Position2D(-1.0f, 0.0341553874314f); // 124
//  data.at(125)= AL::Math::Position2D(1.0f, -0.499334394932f); // 125
//  data.at(126)= AL::Math::Position2D(-0.188144624233f, -1.0f); // 126
//  data.at(127)= AL::Math::Position2D(0.0649817064404f, 1.0f); // 127
//  data.at(128)= AL::Math::Position2D(-1.0f, 0.969097614288f); // 128
//  data.at(129)= AL::Math::Position2D(1.0f, 0.771354734898f); // 129
//  data.at(130)= AL::Math::Position2D(-0.105910584331f, -1.0f); // 130
//  data.at(131)= AL::Math::Position2D(-0.452493965626f, 1.0f); // 131
//  data.at(132)= AL::Math::Position2D(-1.0f, -0.979184091091f); // 132
//  data.at(133)= AL::Math::Position2D(1.0f, -0.298536270857f); // 133
//  data.at(134)= AL::Math::Position2D(0.163637757301f, -1.0f); // 134
//  data.at(135)= AL::Math::Position2D(-0.555234014988f, 1.0f); // 135
//  data.at(136)= AL::Math::Position2D(-1.0f, 0.13957734406f); // 136
//  data.at(137)= AL::Math::Position2D(1.0f, 0.52682185173f); // 137
//  data.at(138)= AL::Math::Position2D(0.02250283584f, -1.0f); // 138
//  data.at(139)= AL::Math::Position2D(-0.676374793053f, 1.0f); // 139
//  data.at(140)= AL::Math::Position2D(-1.0f, -0.73066920042f); // 140
//  data.at(141)= AL::Math::Position2D(1.0f, 0.444440573454f); // 141
//  data.at(142)= AL::Math::Position2D(0.631085455418f, -1.0f); // 142
//  data.at(143)= AL::Math::Position2D(-0.167134568095f, 1.0f); // 143
//  data.at(144)= AL::Math::Position2D(-1.0f, -0.433526724577f); // 144
//  data.at(145)= AL::Math::Position2D(1.0f, 0.347336232662f); // 145
//  data.at(146)= AL::Math::Position2D(-0.0847209915519f, -1.0f); // 146
//  data.at(147)= AL::Math::Position2D(0.888176500797f, 1.0f); // 147
//  data.at(148)= AL::Math::Position2D(-1.0f, 0.122317411005f); // 148
//  data.at(149)= AL::Math::Position2D(1.0f, 0.809071183205f); // 149
//  data.at(150)= AL::Math::Position2D(-0.900987684727f, -1.0f); // 150
//  data.at(151)= AL::Math::Position2D(0.432181298733f, 1.0f); // 151
//  data.at(152)= AL::Math::Position2D(-1.0f, 0.608914554119f); // 152
//  data.at(153)= AL::Math::Position2D(1.0f, -0.00679517025128f); // 153
//  data.at(154)= AL::Math::Position2D(0.167569860816f, -1.0f); // 154
//  data.at(155)= AL::Math::Position2D(0.178124502301f, 1.0f); // 155
//  data.at(156)= AL::Math::Position2D(-1.0f, -0.99185782671f); // 156
//  data.at(157)= AL::Math::Position2D(1.0f, 0.619429469109f); // 157
//  data.at(158)= AL::Math::Position2D(-0.354623436928f, -1.0f); // 158
//  data.at(159)= AL::Math::Position2D(0.0269049331546f, 1.0f); // 159
//  data.at(160)= AL::Math::Position2D(-1.0f, -0.39569285512f); // 160
//  data.at(161)= AL::Math::Position2D(1.0f, -0.19713178277f); // 161
//  data.at(162)= AL::Math::Position2D(-0.640074789524f, -1.0f); // 162
//  data.at(163)= AL::Math::Position2D(0.769285142422f, 1.0f); // 163
//  data.at(164)= AL::Math::Position2D(-1.0f, -0.498618155718f); // 164
//  data.at(165)= AL::Math::Position2D(1.0f, -0.811736941338f); // 165
//  data.at(166)= AL::Math::Position2D(-0.168763056397f, -1.0f); // 166
//  data.at(167)= AL::Math::Position2D(0.233365729451f, 1.0f); // 167
//  data.at(168)= AL::Math::Position2D(-1.0f, -0.930927455425f); // 168
//  data.at(169)= AL::Math::Position2D(1.0f, -0.0252936724573f); // 169
//  data.at(170)= AL::Math::Position2D(0.045550622046f, -1.0f); // 170
//  data.at(171)= AL::Math::Position2D(-0.769372463226f, 1.0f); // 171
//  data.at(172)= AL::Math::Position2D(-1.0f, 0.956544816494f); // 172
//  data.at(173)= AL::Math::Position2D(1.0f, 0.351806938648f); // 173
//  data.at(174)= AL::Math::Position2D(0.878166913986f, -1.0f); // 174
//  data.at(175)= AL::Math::Position2D(-0.854134321213f, 1.0f); // 175
//  data.at(176)= AL::Math::Position2D(-1.0f, -0.367167562246f); // 176
//  data.at(177)= AL::Math::Position2D(1.0f, 0.849012494087f); // 177
//  data.at(178)= AL::Math::Position2D(-0.916960716248f, -1.0f); // 178
//  data.at(179)= AL::Math::Position2D(0.441826492548f, 1.0f); // 179
//  data.at(180)= AL::Math::Position2D(-1.0f, 0.631312251091f); // 180
//  data.at(181)= AL::Math::Position2D(1.0f, 0.0258876793087f); // 181
//  data.at(182)= AL::Math::Position2D(0.70799946785f, -1.0f); // 182
//  data.at(183)= AL::Math::Position2D(0.309800207615f, 1.0f); // 183
//  data.at(184)= AL::Math::Position2D(-1.0f, 0.939734578133f); // 184
//  data.at(185)= AL::Math::Position2D(1.0f, -0.885999381542f); // 185
//  data.at(186)= AL::Math::Position2D(-0.113305293024f, -1.0f); // 186
//  data.at(187)= AL::Math::Position2D(-0.231733962893f, 1.0f); // 187
//  data.at(188)= AL::Math::Position2D(-1.0f, -0.773718297482f); // 188
//  data.at(189)= AL::Math::Position2D(1.0f, 0.619028091431f); // 189
//  data.at(190)= AL::Math::Position2D(0.348928570747f, -1.0f); // 190
//  data.at(191)= AL::Math::Position2D(0.803607225418f, 1.0f); // 191
//  data.at(192)= AL::Math::Position2D(-1.0f, 0.463140189648f); // 192
//  data.at(193)= AL::Math::Position2D(1.0f, -0.793345451355f); // 193
//  data.at(194)= AL::Math::Position2D(-0.00414864066988f, -1.0f); // 194
//  data.at(195)= AL::Math::Position2D(-0.762422442436f, 1.0f); // 195
//  data.at(196)= AL::Math::Position2D(-1.0f, 0.392349839211f); // 196
//  data.at(197)= AL::Math::Position2D(1.0f, -0.948990225792f); // 197
//  data.at(198)= AL::Math::Position2D(-0.906333446503f, -1.0f); // 198
//  data.at(199)= AL::Math::Position2D(-0.937721610069f, 1.0f); // 199
//  data.at(200)= AL::Math::Position2D(-1.0f, 0.259398639202f); // 200
//  data.at(201)= AL::Math::Position2D(1.0f, 0.861361384392f); // 201
//  data.at(202)= AL::Math::Position2D(0.0608270019293f, -1.0f); // 202
//  data.at(203)= AL::Math::Position2D(0.159332439303f, 1.0f); // 203
//  data.at(204)= AL::Math::Position2D(-1.0f, 0.652806341648f); // 204
//  data.at(205)= AL::Math::Position2D(1.0f, -0.00800554361194f); // 205
//  data.at(206)= AL::Math::Position2D(0.0985623374581f, -1.0f); // 206
//  data.at(207)= AL::Math::Position2D(0.0578101426363f, 1.0f); // 207
//  data.at(208)= AL::Math::Position2D(-1.0f, -0.444664716721f); // 208
//  data.at(209)= AL::Math::Position2D(1.0f, 0.568272173405f); // 209
//  data.at(210)= AL::Math::Position2D(0.92127507925f, -1.0f); // 210
//  data.at(211)= AL::Math::Position2D(0.929978132248f, 1.0f); // 211
//  data.at(212)= AL::Math::Position2D(-1.0f, -0.780300319195f); // 212
//  data.at(213)= AL::Math::Position2D(1.0f, 0.753322184086f); // 213
//  data.at(214)= AL::Math::Position2D(0.117532208562f, -1.0f); // 214
//  data.at(215)= AL::Math::Position2D(-0.216102614999f, 1.0f); // 215
//  data.at(216)= AL::Math::Position2D(-1.0f, 0.507620573044f); // 216
//  data.at(217)= AL::Math::Position2D(1.0f, -0.294957488775f); // 217
//  data.at(218)= AL::Math::Position2D(-0.0888517647982f, -1.0f); // 218
//  data.at(219)= AL::Math::Position2D(-0.987100601196f, 1.0f); // 219
//  data.at(220)= AL::Math::Position2D(-1.0f, -0.417905151844f); // 220
//  data.at(221)= AL::Math::Position2D(1.0f, 0.848590493202f); // 221
//  data.at(222)= AL::Math::Position2D(0.435161739588f, -1.0f); // 222
//  data.at(223)= AL::Math::Position2D(-0.93688172102f, 1.0f); // 223
//  data.at(224)= AL::Math::Position2D(-1.0f, 0.595445513725f); // 224
//  data.at(225)= AL::Math::Position2D(1.0f, -0.180838853121f); // 225
//  data.at(226)= AL::Math::Position2D(0.741573154926f, -1.0f); // 226
//  data.at(227)= AL::Math::Position2D(-0.474747359753f, 1.0f); // 227
//  data.at(228)= AL::Math::Position2D(-1.0f, -0.826177477837f); // 228
//  data.at(229)= AL::Math::Position2D(1.0f, 0.637075543404f); // 229
//  data.at(230)= AL::Math::Position2D(-0.966842293739f, -1.0f); // 230
//  data.at(231)= AL::Math::Position2D(0.740065217018f, 1.0f); // 231
//  data.at(232)= AL::Math::Position2D(-1.0f, 0.381835430861f); // 232
//  data.at(233)= AL::Math::Position2D(1.0f, -0.242213994265f); // 233
//  data.at(234)= AL::Math::Position2D(-0.0371092446148f, -1.0f); // 234
//  data.at(235)= AL::Math::Position2D(0.890082597733f, 1.0f); // 235
//  data.at(236)= AL::Math::Position2D(-1.0f, 0.324445039034f); // 236
//  data.at(237)= AL::Math::Position2D(1.0f, 0.141750186682f); // 237
//  data.at(238)= AL::Math::Position2D(0.977463424206f, -1.0f); // 238
//  data.at(239)= AL::Math::Position2D(-0.550112783909f, 1.0f); // 239
//  data.at(240)= AL::Math::Position2D(-1.0f, 0.443135172129f); // 240
//  data.at(241)= AL::Math::Position2D(1.0f, -0.599870681763f); // 241
//  data.at(242)= AL::Math::Position2D(-0.832208573818f, -1.0f); // 242
//  data.at(243)= AL::Math::Position2D(0.465709835291f, 1.0f); // 243
//  data.at(244)= AL::Math::Position2D(-1.0f, 0.401599287987f); // 244
//  data.at(245)= AL::Math::Position2D(1.0f, 0.433333963156f); // 245
//  data.at(246)= AL::Math::Position2D(-0.432613492012f, -1.0f); // 246
//  data.at(247)= AL::Math::Position2D(-0.609803557396f, 1.0f); // 247
//  data.at(248)= AL::Math::Position2D(-1.0f, 0.216377228498f); // 248
//  data.at(249)= AL::Math::Position2D(1.0f, -0.251437455416f); // 249
//  data.at(250)= AL::Math::Position2D(0.408884525299f, -1.0f); // 250
//  data.at(251)= AL::Math::Position2D(0.0578557215631f, 1.0f); // 251
//  data.at(252)= AL::Math::Position2D(-1.0f, 0.34742462635f); // 252
//  data.at(253)= AL::Math::Position2D(1.0f, 0.140886887908f); // 253
//  data.at(254)= AL::Math::Position2D(-0.610124647617f, -1.0f); // 254
//  data.at(255)= AL::Math::Position2D(0.95994412899f, 1.0f); // 255
//  data.at(256)= AL::Math::Position2D(-1.0f, -0.887269198895f); // 256
//  data.at(257)= AL::Math::Position2D(1.0f, -0.900830626488f); // 257
//  data.at(258)= AL::Math::Position2D(-0.780239701271f, -1.0f); // 258
//  data.at(259)= AL::Math::Position2D(-0.699071764946f, 1.0f); // 259
//  data.at(260)= AL::Math::Position2D(-1.0f, 0.903883159161f); // 260
//  data.at(261)= AL::Math::Position2D(1.0f, -0.352707117796f); // 261
//  data.at(262)= AL::Math::Position2D(-0.237847417593f, -1.0f); // 262
//  data.at(263)= AL::Math::Position2D(-0.78850799799f, 1.0f); // 263
//  data.at(264)= AL::Math::Position2D(-1.0f, -0.732275545597f); // 264
//  data.at(265)= AL::Math::Position2D(1.0f, 0.950554311275f); // 265
//  data.at(266)= AL::Math::Position2D(0.582848370075f, -1.0f); // 266
//  data.at(267)= AL::Math::Position2D(-0.603211760521f, 1.0f); // 267
//  data.at(268)= AL::Math::Position2D(-1.0f, 0.868995249271f); // 268
//  data.at(269)= AL::Math::Position2D(1.0f, -0.928301393986f); // 269
//  data.at(270)= AL::Math::Position2D(-0.589914679527f, -1.0f); // 270
//  data.at(271)= AL::Math::Position2D(-0.479908406734f, 1.0f); // 271
//  data.at(272)= AL::Math::Position2D(-1.0f, -0.0218683090061f); // 272
//  data.at(273)= AL::Math::Position2D(1.0f, -0.93544703722f); // 273
//  data.at(274)= AL::Math::Position2D(0.258973926306f, -1.0f); // 274
//  data.at(275)= AL::Math::Position2D(-0.755312025547f, 1.0f); // 275
//  data.at(276)= AL::Math::Position2D(-1.0f, 0.598847270012f); // 276
//  data.at(277)= AL::Math::Position2D(1.0f, 0.681121408939f); // 277
//  data.at(278)= AL::Math::Position2D(0.172919034958f, -1.0f); // 278
//  data.at(279)= AL::Math::Position2D(0.764219760895f, 1.0f); // 279
//  data.at(280)= AL::Math::Position2D(-1.0f, 0.656599164009f); // 280
//  data.at(281)= AL::Math::Position2D(1.0f, -0.271122515202f); // 281
//  data.at(282)= AL::Math::Position2D(-0.121350988746f, -1.0f); // 282
//  data.at(283)= AL::Math::Position2D(0.957195997238f, 1.0f); // 283
//  data.at(284)= AL::Math::Position2D(-1.0f, 0.327833026648f); // 284
//  data.at(285)= AL::Math::Position2D(1.0f, -0.841997623444f); // 285
//  data.at(286)= AL::Math::Position2D(0.824572503567f, -1.0f); // 286
//  data.at(287)= AL::Math::Position2D(0.454606682062f, 1.0f); // 287
//  data.at(288)= AL::Math::Position2D(-1.0f, 0.765862286091f); // 288
//  data.at(289)= AL::Math::Position2D(1.0f, 0.794291198254f); // 289
//  data.at(290)= AL::Math::Position2D(-0.423808813095f, -1.0f); // 290
//  data.at(291)= AL::Math::Position2D(-0.339033663273f, 1.0f); // 291
//  data.at(292)= AL::Math::Position2D(-1.0f, 0.298035055399f); // 292
//  data.at(293)= AL::Math::Position2D(1.0f, 0.670941948891f); // 293
//  data.at(294)= AL::Math::Position2D(-0.664223372936f, -1.0f); // 294
//  data.at(295)= AL::Math::Position2D(-0.492598891258f, 1.0f); // 295
//  data.at(296)= AL::Math::Position2D(-1.0f, -0.357796162367f); // 296
//  data.at(297)= AL::Math::Position2D(1.0f, -0.370494872332f); // 297
//  data.at(298)= AL::Math::Position2D(0.871116995811f, -1.0f); // 298
//  data.at(299)= AL::Math::Position2D(0.333215385675f, 1.0f); // 299
//  data.at(300)= AL::Math::Position2D(-1.0f, -0.668628573418f); // 300
//  data.at(301)= AL::Math::Position2D(1.0f, -0.503084659576f); // 301
//  data.at(302)= AL::Math::Position2D(-0.41406339407f, -1.0f); // 302
//  data.at(303)= AL::Math::Position2D(0.746048867702f, 1.0f); // 303
//  data.at(304)= AL::Math::Position2D(-1.0f, -0.191886022687f); // 304
//  data.at(305)= AL::Math::Position2D(1.0f, 0.791168630123f); // 305
//  data.at(306)= AL::Math::Position2D(0.493341356516f, -1.0f); // 306
//  data.at(307)= AL::Math::Position2D(-0.983940780163f, 1.0f); // 307
//  data.at(308)= AL::Math::Position2D(-1.0f, 0.969746351242f); // 308
//  data.at(309)= AL::Math::Position2D(1.0f, -0.10306148231f); // 309
//  data.at(310)= AL::Math::Position2D(0.618602275848f, -1.0f); // 310
//  data.at(311)= AL::Math::Position2D(0.900610089302f, 1.0f); // 311
//  data.at(312)= AL::Math::Position2D(-1.0f, -0.784377515316f); // 312
//  data.at(313)= AL::Math::Position2D(1.0f, 0.451298445463f); // 313
//  data.at(314)= AL::Math::Position2D(0.638666391373f, -1.0f); // 314
//  data.at(315)= AL::Math::Position2D(-0.285782158375f, 1.0f); // 315
//  data.at(316)= AL::Math::Position2D(-1.0f, -0.174402222037f); // 316
//  data.at(317)= AL::Math::Position2D(1.0f, 0.389386385679f); // 317
//  data.at(318)= AL::Math::Position2D(0.572149574757f, -1.0f); // 318
//  data.at(319)= AL::Math::Position2D(0.399224162102f, 1.0f); // 319
//  data.at(320)= AL::Math::Position2D(-1.0f, -0.909424066544f); // 320
//  data.at(321)= AL::Math::Position2D(1.0f, 0.500375986099f); // 321
//  data.at(322)= AL::Math::Position2D(-0.803614974022f, -1.0f); // 322
//  data.at(323)= AL::Math::Position2D(0.071489572525f, 1.0f); // 323
//  data.at(324)= AL::Math::Position2D(-1.0f, 0.0382244847715f); // 324
//  data.at(325)= AL::Math::Position2D(1.0f, 0.15051163733f); // 325
//  data.at(326)= AL::Math::Position2D(-0.108196355402f, -1.0f); // 326
//  data.at(327)= AL::Math::Position2D(0.30141338706f, 1.0f); // 327
//  data.at(328)= AL::Math::Position2D(-1.0f, -0.409922450781f); // 328
//  data.at(329)= AL::Math::Position2D(1.0f, -0.710368394852f); // 329
//  data.at(330)= AL::Math::Position2D(-0.836511552334f, -1.0f); // 330
//  data.at(331)= AL::Math::Position2D(0.780612945557f, 1.0f); // 331
//  data.at(332)= AL::Math::Position2D(-1.0f, -0.719608068466f); // 332
//  data.at(333)= AL::Math::Position2D(1.0f, 0.892438828945f); // 333
//  data.at(334)= AL::Math::Position2D(-0.236840128899f, -1.0f); // 334
//  data.at(335)= AL::Math::Position2D(-0.280815422535f, 1.0f); // 335
//  data.at(336)= AL::Math::Position2D(-1.0f, 0.1400629282f); // 336
//  data.at(337)= AL::Math::Position2D(1.0f, 0.426683604717f); // 337
//  data.at(338)= AL::Math::Position2D(-0.446151286364f, -1.0f); // 338
//  data.at(339)= AL::Math::Position2D(-0.295199006796f, 1.0f); // 339
//  data.at(340)= AL::Math::Position2D(-1.0f, 0.997077465057f); // 340
//  data.at(341)= AL::Math::Position2D(1.0f, 0.0180927608162f); // 341
//  data.at(342)= AL::Math::Position2D(-0.118684470654f, -1.0f); // 342
//  data.at(343)= AL::Math::Position2D(-0.896330177784f, 1.0f); // 343
//  data.at(344)= AL::Math::Position2D(-1.0f, 0.237224668264f); // 344
//  data.at(345)= AL::Math::Position2D(1.0f, -0.86908197403f); // 345
//  data.at(346)= AL::Math::Position2D(0.855087518692f, -1.0f); // 346
//  data.at(347)= AL::Math::Position2D(0.0292538926005f, 1.0f); // 347
//  data.at(348)= AL::Math::Position2D(-1.0f, 0.305737704039f); // 348
//  data.at(349)= AL::Math::Position2D(1.0f, 0.670632898808f); // 349
//  data.at(350)= AL::Math::Position2D(-0.319678455591f, -1.0f); // 350
//  data.at(351)= AL::Math::Position2D(0.778663873672f, 1.0f); // 351
//  data.at(352)= AL::Math::Position2D(-1.0f, -0.159202516079f); // 352
//  data.at(353)= AL::Math::Position2D(1.0f, 0.215545028448f); // 353
//  data.at(354)= AL::Math::Position2D(-0.835422217846f, -1.0f); // 354
//  data.at(355)= AL::Math::Position2D(-0.733416318893f, 1.0f); // 355
//  data.at(356)= AL::Math::Position2D(-1.0f, -0.852869093418f); // 356
//  data.at(357)= AL::Math::Position2D(1.0f, 0.419420868158f); // 357
//  data.at(358)= AL::Math::Position2D(-0.5707654953f, -1.0f); // 358
//  data.at(359)= AL::Math::Position2D(0.762228667736f, 1.0f); // 359
//  data.at(360)= AL::Math::Position2D(-1.0f, -0.924536883831f); // 360
//  data.at(361)= AL::Math::Position2D(1.0f, 0.797517836094f); // 361
//  data.at(362)= AL::Math::Position2D(0.718494772911f, -1.0f); // 362
//  data.at(363)= AL::Math::Position2D(0.977685809135f, 1.0f); // 363
//  data.at(364)= AL::Math::Position2D(-1.0f, 0.0121186915785f); // 364
//  data.at(365)= AL::Math::Position2D(1.0f, 0.487579137087f); // 365
//  data.at(366)= AL::Math::Position2D(0.493166893721f, -1.0f); // 366
//  data.at(367)= AL::Math::Position2D(0.816378772259f, 1.0f); // 367
//  data.at(368)= AL::Math::Position2D(-1.0f, 0.572971105576f); // 368
//  data.at(369)= AL::Math::Position2D(1.0f, -0.637290358543f); // 369
//  data.at(370)= AL::Math::Position2D(0.616978228092f, -1.0f); // 370
//  data.at(371)= AL::Math::Position2D(0.431884586811f, 1.0f); // 371
//  data.at(372)= AL::Math::Position2D(-1.0f, -0.313541322947f); // 372
//  data.at(373)= AL::Math::Position2D(1.0f, 0.265510469675f); // 373
//  data.at(374)= AL::Math::Position2D(-0.967637717724f, -1.0f); // 374
//  data.at(375)= AL::Math::Position2D(-0.851829230785f, 1.0f); // 375
//  data.at(376)= AL::Math::Position2D(-1.0f, -0.0240753572434f); // 376
//  data.at(377)= AL::Math::Position2D(1.0f, -0.62074804306f); // 377
//  data.at(378)= AL::Math::Position2D(-0.716103315353f, -1.0f); // 378
//  data.at(379)= AL::Math::Position2D(0.764843344688f, 1.0f); // 379
//  data.at(380)= AL::Math::Position2D(-1.0f, 0.237947180867f); // 380
//  data.at(381)= AL::Math::Position2D(1.0f, 0.970929265022f); // 381
//  data.at(382)= AL::Math::Position2D(0.175047710538f, -1.0f); // 382
//  data.at(383)= AL::Math::Position2D(0.865338563919f, 1.0f); // 383
//  data.at(384)= AL::Math::Position2D(-1.0f, -0.995214521885f); // 384
//  data.at(385)= AL::Math::Position2D(1.0f, -0.588854670525f); // 385
//  data.at(386)= AL::Math::Position2D(-0.599455475807f, -1.0f); // 386
//  data.at(387)= AL::Math::Position2D(-0.672585964203f, 1.0f); // 387
//  data.at(388)= AL::Math::Position2D(-1.0f, -0.440914571285f); // 388
//  data.at(389)= AL::Math::Position2D(1.0f, -0.113261073828f); // 389
//  data.at(390)= AL::Math::Position2D(-0.883921265602f, -1.0f); // 390
//  data.at(391)= AL::Math::Position2D(0.758514046669f, 1.0f); // 391
//  data.at(392)= AL::Math::Position2D(-1.0f, 0.0125746848062f); // 392
//  data.at(393)= AL::Math::Position2D(1.0f, 0.305404067039f); // 393
//  data.at(394)= AL::Math::Position2D(-0.402139306068f, -1.0f); // 394
//  data.at(395)= AL::Math::Position2D(0.325373709202f, 1.0f); // 395
//  data.at(396)= AL::Math::Position2D(-1.0f, 0.257173955441f); // 396
//  data.at(397)= AL::Math::Position2D(1.0f, -0.932736873627f); // 397
//  data.at(398)= AL::Math::Position2D(0.091488994658f, -1.0f); // 398
//  data.at(399)= AL::Math::Position2D(0.0367580987513f, 1.0f); // 399

//  // possible solution
//  //  data.at(396)= AL::Math::Position2D(-1.0f, -1.0f); // 396
//  //  data.at(397)= AL::Math::Position2D(-1.0f, 1.0f); // 397
//  //  data.at(398)= AL::Math::Position2D(1.0f, -1.0f); // 398
//  //  data.at(399)= AL::Math::Position2D(1.0f, 1.0f); // 399

//  std::vector<AL::Math::Position2D> pPoints;
//  AL::Math::removeAlignedPoint(data, pPoints);
//  EXPECT_EQ(4, (int)pPoints.size());

//  // Usefull for plot in python
//  std::cout << "result of removeAlignedPoint" << std::endl;
//  for (unsigned int i=0; i<pPoints.size(); i++)
//  {
//    std::cout << "data.append(almath.Position2D("
//              << pPoints.at(i).x << ", " << pPoints.at(i).y << "))" << std::endl;
//  }

//  AL::Math::deleteDoublesInNoneSortVector(data);
//  EXPECT_EQ(400, (int)data.size());

//  std::vector<AL::Math::Position2D> result = AL::Math::getConvexHull(data);
//  EXPECT_EQ(5, (int)result.size());

//  // Usefull for plot in python
//  std::cout << "result of getConvexHull" << std::endl;
//  for (unsigned int i=0; i<result.size(); i++)
//  {
//    std::cout << "data.append(almath.Position2D("
//              << result.at(i).x << ", " << result.at(i).y << "))" << std::endl;
//  }

////  std::vector<AL::Math::Position2D> solutionExpected;
////  solutionExpected.push_back(AL::Math::Position2D(-1.0f, -1.0f));
////  solutionExpected.push_back(AL::Math::Position2D(-1.0f, 1.0f));
////  solutionExpected.push_back(AL::Math::Position2D(1.0f, -1.0f));
////  solutionExpected.push_back(AL::Math::Position2D(1.0f, 1.0f));
////  bool isInSolution = false;
////  for (unsigned int i=0; i<solutionExpected.size(); i++)
////  {
////    isInSolution = false;
////    for (unsigned int j=0; j<result.size(); j++)
////    {
////      if (solutionExpected.at(i).isNear(result.at(j), 0.000001f))
////      {
////        isInSolution = true;
////      }
////    }

////    EXPECT_TRUE(isInSolution);
////  }


//  float angle = 45.0*TO_RAD;
//  AL::Math::Rotation2D rot = AL::Math::Rotation2D::fromAngle(angle);
//  for (unsigned int i=0; i<data.size(); i++)
//  {
//    data.at(i) = rot*data.at(i);
//  }

//  AL::Math::removeAlignedPoint(data, pPoints);
//  EXPECT_EQ(4, (int)pPoints.size());

//  AL::Math::deleteDoublesInNoneSortVector(data);
//  EXPECT_EQ(400, (int)data.size());
//}


TEST(isLeftBest, compare2)
{
  // -4: at least two points are equal
  // -3: points are nearly aligned but pB is not in the middle of pA, pC
  // -2: points are aligned and pB is in the middle of pA, pC
  // -1: pC is left
  // +1: pC is right
  AL::Math::Position2D pA;
  AL::Math::Position2D pB;
  AL::Math::Position2D pC;

  //i: 77 j: 8 k: 41
  pA = AL::Math::Position2D(0.0742930024862f, 0.215870693326f); // 77
  pB = AL::Math::Position2D(0.975853681564f, 0.0119071817026f); // 8
  pC = AL::Math::Position2D(0.826174616814f, 0.0483366213739f); // 41
  EXPECT_EQ(-3, AL::Math::isLeftBest(pA, pB, pC));
}

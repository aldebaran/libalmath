/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
//#include <almath/types/alquaternion.h>
#include <almath/tools/altransformhelpers.h>
#include <gtest/gtest.h>

#include <stdexcept>


TEST(ALQuaternionTest, creation)
{
  AL::Math::Quaternion pQua1 = AL::Math::Quaternion();
  AL::Math::Quaternion pQua2 = AL::Math::Quaternion();
  AL::Math::Quaternion pQua3 = AL::Math::Quaternion();

  // create Quaternion()
  pQua1 = AL::Math::Quaternion();
  EXPECT_TRUE(pQua1.w == 1.0f);
  EXPECT_TRUE(pQua1.x == 0.0f);
  EXPECT_TRUE(pQua1.y == 0.0f);
  EXPECT_TRUE(pQua1.z == 0.0f);

  // create Quaternion(w, x, y, z)
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_TRUE(pQua1.w == 0.1f);
  EXPECT_TRUE(pQua1.x == 0.2f);
  EXPECT_TRUE(pQua1.y == 0.3f);
  EXPECT_TRUE(pQua1.z == 0.4f);

  // create Quaternion(const std::vector<float>& pFloats)
  std::vector<float> listQuaternion;
  listQuaternion.push_back(0.1f);
  listQuaternion.push_back(0.2f);
  listQuaternion.push_back(0.3f);
  listQuaternion.push_back(0.4f);

  pQua1 = AL::Math::Quaternion(listQuaternion);
  EXPECT_TRUE(pQua1.w == 0.1f);
  EXPECT_TRUE(pQua1.x == 0.2f);
  EXPECT_TRUE(pQua1.y == 0.3f);
  EXPECT_TRUE(pQua1.z == 0.4f);

  // operator *= with Quaternion
  // operator *  with Quaternion

  // operator == with Quaternion
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_TRUE((pQua1==pQua2));

  // operator != with Quaternion
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.11f, 0.2f, 0.3f, 0.4f);
  EXPECT_TRUE((pQua1!=pQua2));

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.21f, 0.3f, 0.4f);
  EXPECT_TRUE((pQua1!=pQua2));

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.31f, 0.4f);
  EXPECT_TRUE((pQua1!=pQua2));

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.41f);
  EXPECT_TRUE((pQua1!=pQua2));

  // operator *= with float
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua1 *= 2.0f;
  pQua2 = AL::Math::Quaternion(0.2f, 0.4f, 0.6f, 0.8f);
  EXPECT_TRUE((pQua1==pQua2));

  // operator /= with float
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.2f, 0.4f, 0.6f, 0.8f);
  pQua2 /= 2.0f;
  EXPECT_TRUE((pQua1==pQua2));

  // isNear

  // norm

  // normalize

  // inverse

  // fromAngleAndAxisRotation

  // toVector

  // function quaternionFromAngleAndAxisRotation

  // function norm

  // function normalize

  // function quaternionInverse

  // function angleAndAxisRotationFromQuaternion

}

TEST(ALQuaternionTest, normalize)
{
  AL::Math::Quaternion pQuaD1 = AL::Math::Quaternion();
}


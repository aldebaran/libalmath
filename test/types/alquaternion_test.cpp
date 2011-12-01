/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
//#include <almath/types/alquaternion.h>
#include <almath/tools/altransformhelpers.h>
#include <almath/tools/altrigonometry.h>
#include <almath/tools/almathio.h>

#include <cmath>

#include <gtest/gtest.h>

#include <stdexcept>


TEST(ALQuaternionTest, creation)
{
  AL::Math::Transform  pT1   = AL::Math::Transform();
  AL::Math::Transform  pT2   = AL::Math::Transform();
  AL::Math::Transform  pT3   = AL::Math::Transform();
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
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.2f, 0.4f, 0.6f, 0.8f);
  EXPECT_FALSE(pQua1.isNear(pQua2, 0.0001f));

  pQua1 = AL::Math::Quaternion(-0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_FALSE(pQua1.isNear(pQua2, 0.0001f));

  pQua1 = AL::Math::Quaternion(0.1f, -0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_FALSE(pQua1.isNear(pQua2, 0.0001f));

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, -0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_FALSE(pQua1.isNear(pQua2, 0.0001f));

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, -0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_FALSE(pQua1.isNear(pQua2, 0.0001f));

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_TRUE(pQua1.isNear(pQua2, 0.0001f));

  pQua1 = AL::Math::Quaternion(-0.1f, -0.2f, -0.3f, -0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_TRUE(pQua1.isNear(pQua2, 0.0001f));

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(-0.1f, -0.2f, -0.3f, -0.4f);
  EXPECT_TRUE(pQua1.isNear(pQua2, 0.0001f));

  // norm
  pQua1 = AL::Math::Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
  float normQua = pQua1.norm();
  EXPECT_NEAR(normQua, sqrtf(30), 0.0001f);

  // normalize
  pQua1 = AL::Math::Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pQua1), std::runtime_error);

  // inverse

  // fromAngleAndAxisRotation

  // toVector

  // function quaternionFromAngleAndAxisRotation

  // function norm

  // function normalize

  // function quaternionInverse

  // function angleAndAxisRotationFromQuaternion


//  pT1
//  pT2
//  pQua1
//  pQua2
//  pQua3

  // operator *= with Quaternion
  // operator *  with Quaternion

  unsigned int nbX = 10;
  unsigned int nbY = 10;
  unsigned int nbZ = 10;

  for (unsigned int i=0; i<nbX; i++)
  {
    for (unsigned int j=0; j<nbY; j++)
    {
      for (unsigned int k=0; k<nbZ; k++)
      {
        float angleX = ((float)i)/(float(nbX))*AL::Math::_2_PI_;
        float angleY = ((float)j)/(float(nbY))*AL::Math::_2_PI_;
        float angleZ = ((float)k)/(float(nbZ))*AL::Math::_2_PI_;

        pT1 = AL::Math::Transform::fromRotX(angleX)*
            AL::Math::Transform::fromRotY(angleY)*
            AL::Math::Transform::fromRotZ(angleZ);

        pT2 = AL::Math::Transform::fromRotZ(angleY)*
            AL::Math::Transform::fromRotX(-angleZ)*
            AL::Math::Transform::fromRotY(-angleX);

        pT3 = pT1*pT2;

        pQua1 = AL::Math::quaternionFromTransform(pT1);
        pQua2 = AL::Math::quaternionFromTransform(pT2);
        pQua3 = pQua1*pQua2;

        EXPECT_TRUE(pQua3.isNear(AL::Math::quaternionFromTransform(pT3), 0.0001f));

        // if (!pQua3.isNear(AL::Math::quaternionFromTransform(pT3), 0.0001f))
        // {
        //   std::cout << pQua3.isNear(AL::Math::quaternionFromTransform(pT3)) << " "
        //             << angleX*AL::Math::TO_DEG << " "
        //             << angleY*AL::Math::TO_DEG << " "
        //             << angleZ*AL::Math::TO_DEG << std::endl;

        //   std::cout << "pQua3" << std::endl << pQua3 << std::endl;
        //   std::cout << "AL::Math::quaternionFromTransform(pT3)" << std::endl
        //             << AL::Math::quaternionFromTransform(pT3) << std::endl;
        //   std::cout << std::endl;
        // }

        // if (!pTIn.isNear(pTOut, 0.0001f))
        // {
        //   std::cout << "[angleX, angleY, angleZ, i, j, k]: "
        //             << angleX << " " << angleY << " " << angleZ
        //             << i << " " << j << " " << k << std::endl;
        // }

      }
    }
  }

}

TEST(ALQuaternionTest, normalize)
{
  AL::Math::Quaternion pQuaD1 = AL::Math::Quaternion();
}


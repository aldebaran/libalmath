/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/altransformhelpers.h>
#include <almath/tools/altrigonometry.h>
#include <almath/tools/almathio.h>

#include <cmath>

#include <gtest/gtest.h>

#include <stdexcept>

TEST(ALQuaternionTest, basicOperator)
{
  AL::Math::Quaternion pQua1 = AL::Math::Quaternion();
  AL::Math::Quaternion pQua2 = AL::Math::Quaternion();
  AL::Math::Quaternion pQua3 = AL::Math::Quaternion();
  AL::Math::Transform  pT1   = AL::Math::Transform();
  AL::Math::Transform  pT2   = AL::Math::Transform();
  AL::Math::Transform  pT3   = AL::Math::Transform();

  // operator == with Quaternion
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_TRUE(pQua1==pQua2);

  // operator != with Quaternion
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.11f, 0.2f, 0.3f, 0.4f);
  EXPECT_TRUE(pQua1!=pQua2);

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.21f, 0.3f, 0.4f);
  EXPECT_TRUE(pQua1!=pQua2);

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.31f, 0.4f);
  EXPECT_TRUE(pQua1!=pQua2);

  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.41f);
  EXPECT_TRUE(pQua1!=pQua2);

  // operator *= with float
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua1 *= 2.0f;
  pQua2 = AL::Math::Quaternion(0.2f, 0.4f, 0.6f, 0.8f);
  EXPECT_TRUE(pQua1==pQua2);

  // operator /= with float
  pQua1 = AL::Math::Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
  pQua2 = AL::Math::Quaternion(0.2f, 0.4f, 0.6f, 0.8f);
  pQua2 /= 2.0f;
  EXPECT_TRUE(pQua1==pQua2);

  // operator *= with Quaternion
  pQua1 = AL::Math::Quaternion();
  pQua2 = AL::Math::Quaternion();
  pQua2 = pQua1;
  pQua2 *= pQua1;
  pQua1 *= pQua1;
  EXPECT_TRUE(pQua2.isNear(pQua1));

  AL::Math::Transform tf = AL::Math::Transform::from3DRotation(0.2f, 0.1f, -0.3f);
  pQua1 = AL::Math::quaternionFromTransform(tf);
  pQua2 = pQua1;

  pQua2 *= pQua1;
  pQua1 *= pQua1;

  EXPECT_TRUE(pQua2.isNear(pQua1));

  // operator *= with Quaternion
  // operator *  with Quaternion
  unsigned int nbX = 101;
  unsigned int nbY = 101;
  unsigned int nbZ = 101;

  for (unsigned int i=0; i<nbX; ++i)
  {
    for (unsigned int j=0; j<nbY; ++j)
    {
      for (unsigned int k=0; k<nbZ; ++k)
      {
        float angleX = static_cast<float>(i+1)/(static_cast<float>(nbX+1))*AL::Math::_2_PI_ - AL::Math::PI;
        float angleY = static_cast<float>(j+1)/(static_cast<float>(nbY+1))*AL::Math::_2_PI_ - AL::Math::PI;
        float angleZ = static_cast<float>(k+1)/(static_cast<float>(nbZ+1))*AL::Math::_2_PI_ - AL::Math::PI;

        pT1 = AL::Math::Transform::fromRotX(angleX)*
            AL::Math::Transform::fromRotY(angleY)*
            AL::Math::Transform::fromRotZ(angleZ);

        pT2 = AL::Math::Transform::fromRotZ(angleY)*
            AL::Math::Transform::fromRotX(-angleZ)*
            AL::Math::Transform::fromRotY(-angleX);

        pT3 = pT1*pT2;
        // test A
        pQua1 = AL::Math::quaternionFromTransform(pT1);
        pQua2 = AL::Math::quaternionFromTransform(pT2);
        pQua3 = pQua1*pQua2;
        EXPECT_TRUE(pQua3.isNear(AL::Math::quaternionFromTransform(pT3), 0.001f));

        // test B
        pQua1 = AL::Math::quaternionFromTransform(pT1);
        pQua2 = AL::Math::quaternionFromTransform(pT2);
        pQua1 *= pQua2;
        EXPECT_TRUE(pQua1.isNear(AL::Math::quaternionFromTransform(pT3), 0.001f));

        // test C
        pQua1 = AL::Math::quaternionFromTransform(pT1);
        pQua2 = pQua1;
        pQua2 *= pQua1;
        pQua1 *= pQua1;
        EXPECT_TRUE(pQua2.isNear(pQua1));
      }
    }
  }

}

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
  // function norm
  pQua1 = AL::Math::Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
  float normQua = pQua1.norm();
  EXPECT_NEAR(normQua, std::sqrt(30.0f), 0.0001f);

  // normalize
  // function normalize
  pQua1 = AL::Math::Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
  ASSERT_THROW(AL::Math::normalize(pQua1), std::runtime_error);

  // toVector
  pQua1 = AL::Math::Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
  std::vector<float> pQua1Vect = pQua1.toVector();

  EXPECT_NEAR(pQua1Vect.at(0), 1.0f, 0.0001f);
  EXPECT_NEAR(pQua1Vect.at(1), 2.0f, 0.0001f);
  EXPECT_NEAR(pQua1Vect.at(2), 3.0f, 0.0001f);
  EXPECT_NEAR(pQua1Vect.at(3), 4.0f, 0.0001f);

  unsigned int nbX = 101;
  unsigned int nbY = 101;
  unsigned int nbZ = 101;

  // fromAngleAndAxisRotation
  // function quaternionFromAngleAndAxisRotation
  // angleAndAxisRotationFromQuaternion
  for (unsigned int i=0; i<nbX; ++i)
  {
    float angle = static_cast<float>(i+1)/(static_cast<float>(nbX+1))*AL::Math::_2_PI_ - AL::Math::PI;

    pQua1 = AL::Math::Quaternion::fromAngleAndAxisRotation(angle, 1.0f, 0.0f, 0.0f);
    pT1   = AL::Math::Transform::fromRotX(angle);
    EXPECT_TRUE(pQua1.isNear(AL::Math::quaternionFromTransform(pT1), 0.0001f));

    pQua1 = AL::Math::Quaternion::fromAngleAndAxisRotation(angle, 0.0f, 1.0f, 0.0f);
    pT1   = AL::Math::Transform::fromRotY(angle);
    EXPECT_TRUE(pQua1.isNear(AL::Math::quaternionFromTransform(pT1), 0.0001f));

    pQua1 = AL::Math::Quaternion::fromAngleAndAxisRotation(angle, 0.0f, 0.0f, 1.0f);
    pT1   = AL::Math::Transform::fromRotZ(angle);
    EXPECT_TRUE(pQua1.isNear(AL::Math::quaternionFromTransform(pT1), 0.0001f));

    float epsilon = 0.001f;

    float pAngleResult = 0.0f;
    float pAxeXResult  = 0.0f;
    float pAxeYResult  = 0.0f;
    float pAxeZResult  = 0.0f;

    pQua1 = AL::Math::quaternionFromTransform(AL::Math::Transform::fromRotX(angle));
    AL::Math::angleAndAxisRotationFromQuaternion(pQua1, pAngleResult, pAxeXResult, pAxeYResult, pAxeZResult);

    std::vector<float> result = AL::Math::angleAndAxisRotationFromQuaternion(pQua1);
    EXPECT_TRUE(result.size()==4);
    EXPECT_NEAR(result.at(0), pAngleResult, epsilon);
    EXPECT_NEAR(result.at(1), pAxeXResult, epsilon);
    EXPECT_NEAR(result.at(2), pAxeYResult, epsilon);
    EXPECT_NEAR(result.at(3), pAxeZResult, epsilon);

    bool isSuccess = false;
    if (((std::abs(angle-pAngleResult) < epsilon) &&
          (std::abs(pAxeXResult-1.0f) < epsilon) &&
          (std::abs(pAxeYResult-0.0f) < epsilon) &&
          (std::abs(pAxeZResult-0.0f) < epsilon)) ||
        ((std::abs(angle+pAngleResult) < epsilon) &&
          (std::abs(pAxeXResult+1.0f) < epsilon) &&
          (std::abs(pAxeYResult+0.0f) < epsilon) &&
          (std::abs(pAxeZResult+0.0f) < epsilon)))
    {
      isSuccess = true;
    }
    else
    {
      isSuccess = false;
    }
    EXPECT_TRUE(isSuccess);

    pQua1 = AL::Math::quaternionFromTransform(AL::Math::Transform::fromRotY(angle));
    AL::Math::angleAndAxisRotationFromQuaternion(pQua1, pAngleResult, pAxeXResult, pAxeYResult, pAxeZResult);

    result = AL::Math::angleAndAxisRotationFromQuaternion(pQua1);
    EXPECT_TRUE(result.size()==4);
    EXPECT_NEAR(result.at(0), pAngleResult, epsilon);
    EXPECT_NEAR(result.at(1), pAxeXResult, epsilon);
    EXPECT_NEAR(result.at(2), pAxeYResult, epsilon);
    EXPECT_NEAR(result.at(3), pAxeZResult, epsilon);

    if (((std::abs(angle-pAngleResult) < epsilon) &&
         (std::abs(pAxeXResult-0.0f) < epsilon) &&
         (std::abs(pAxeYResult-1.0f) < epsilon) &&
         (std::abs(pAxeZResult-0.0f) < epsilon)) ||
        ((std::abs(angle+pAngleResult) < epsilon) &&
         (std::abs(pAxeXResult+0.0f) < epsilon) &&
         (std::abs(pAxeYResult+1.0f) < epsilon) &&
         (std::abs(pAxeZResult+0.0f) < epsilon)
         )
        )
    {
      isSuccess = true;
    }
    else
    {
      isSuccess = false;
    }

    // Identity special case
    if (std::abs(angle) < epsilon)
    {
      isSuccess = true;
    }

    EXPECT_TRUE(isSuccess);

    pQua1 = AL::Math::quaternionFromTransform(AL::Math::Transform::fromRotZ(angle));
    AL::Math::angleAndAxisRotationFromQuaternion(pQua1, pAngleResult, pAxeXResult, pAxeYResult, pAxeZResult);

    result = AL::Math::angleAndAxisRotationFromQuaternion(pQua1);
    EXPECT_TRUE(result.size()==4);
    EXPECT_NEAR(result.at(0), pAngleResult, epsilon);
    EXPECT_NEAR(result.at(1), pAxeXResult, epsilon);
    EXPECT_NEAR(result.at(2), pAxeYResult, epsilon);
    EXPECT_NEAR(result.at(3), pAxeZResult, epsilon);

    if (((std::abs(angle-pAngleResult) < epsilon) &&
          (std::abs(pAxeXResult-0.0f) < epsilon) &&
          (std::abs(pAxeYResult-0.0f) < epsilon) &&
          (std::abs(pAxeZResult-1.0f) < epsilon)) ||
        ((std::abs(angle+pAngleResult) < epsilon) &&
          (std::abs(pAxeXResult+0.0f) < epsilon) &&
          (std::abs(pAxeYResult+0.0f) < epsilon) &&
          (std::abs(pAxeZResult+1.0f) < epsilon)))
    {
      isSuccess = true;
    }
    else
    {
      isSuccess = false;
    }
    // Identity special case
    if (std::abs(angle) < epsilon)
    {
      isSuccess = true;
    }
    EXPECT_TRUE(isSuccess);
  }

  // inverse
  // function quaternionInverse
  for (unsigned int i=0; i<nbX; ++i)
  {
    for (unsigned int j=0; j<nbY; ++j)
    {
      for (unsigned int k=0; k<nbZ; ++k)
      {
        float angleX = static_cast<float>(i+1)/(static_cast<float>(nbX+1))*AL::Math::_2_PI_ - AL::Math::PI;
        float angleY = static_cast<float>(j+1)/(static_cast<float>(nbY+1))*AL::Math::_2_PI_ - AL::Math::PI;
        float angleZ = static_cast<float>(k+1)/(static_cast<float>(nbZ+1))*AL::Math::_2_PI_ - AL::Math::PI;

        pT1 = AL::Math::Transform::fromRotX(angleX)*
            AL::Math::Transform::fromRotY(angleY)*
            AL::Math::Transform::fromRotZ(angleZ);

        pQua1 = AL::Math::quaternionFromTransform(pT1);
        pQua2 = pQua1.inverse();

        pT2 = AL::Math::pinv(pT1);
        pQua3 = AL::Math::quaternionFromTransform(pT2);

        EXPECT_TRUE(pQua2.isNear(pQua3, 0.001f));

      }
    }
  }

}

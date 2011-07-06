/**
* @author Cyrille Collette
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alrotation2d.h>

#include <cmath>

namespace AL {
  namespace Math {

    Rotation2D& Rotation2D::operator*= (const Rotation2D& pRot2)
    {
      float c1 = r1_c1;
      float c2 = r1_c2;

      r1_c1 = (c1 * pRot2.r1_c1) + (c2 * pRot2.r2_c1);
      r1_c2 = (c1 * pRot2.r1_c2) + (c2 * pRot2.r2_c2);

      c1 = r2_c1;
      c2 = r2_c2;

      r2_c1 = (c1 * pRot2.r1_c1) + (c2 * pRot2.r2_c1);
      r2_c2 = (c1 * pRot2.r1_c2) + (c2 * pRot2.r2_c2);

      return *this;
    }

    Rotation2D Rotation2D::operator* (const Rotation2D& pRot2) const
    {
      Rotation2D pOut = *this;
      pOut *= pRot2;
      return pOut;
    }


    bool Rotation2D::operator==(const Rotation2D& pRot2) const
    {
      if (
        (r1_c1 == pRot2.r1_c1) &&
        (r1_c2 == pRot2.r1_c2) &&
        (r2_c1 == pRot2.r2_c1) &&
        (r2_c2 == pRot2.r2_c2))
      {
        return true;
      }
      else
      {
        return false;
      }
    }


    bool Rotation2D::operator!=(const Rotation2D& pRot2) const
    {
      return !(*this==pRot2);
    }


    Rotation2D Rotation2D::transpose() const
    {
      return Math::transpose(*this);
    }

    float Rotation2D::determinant() const
    {
      return Math::determinant(*this);
    }

    bool Rotation2D::isNear(
      const Rotation2D& pRot2,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(r1_c1 - pRot2.r1_c1) > pEpsilon) ||
        (fabsf(r1_c2 - pRot2.r1_c2) > pEpsilon) ||
        (fabsf(r2_c1 - pRot2.r2_c1) > pEpsilon) ||
        (fabsf(r2_c2 - pRot2.r2_c2) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    Rotation2D Rotation2D::fromAngle(const float pTheta)
    {
      Rotation2D rot = Rotation2D();

      float c = cosf(pTheta);
      float s = sinf(pTheta);

      rot.r1_c1 = c;
      rot.r1_c2 = -s;
      rot.r2_c1 = s;
      rot.r2_c2 = c;
      return rot;
    }

    std::vector<float> Rotation2D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(4);
      returnVector[0] = r1_c1;
      returnVector[1] = r1_c2;

      returnVector[2] = r2_c1;
      returnVector[3] = r2_c2;

      return returnVector;
    }


    Rotation2D transpose(const Rotation2D& pRot)
    {
      Rotation2D pOut;
      pOut.r1_c1 = pRot.r1_c1;
      pOut.r1_c2 = pRot.r2_c1;
      pOut.r2_c1 = pRot.r1_c2;
      pOut.r2_c2 = pRot.r2_c2;
      return pOut;
    }

    float determinant(const Rotation2D& pRot)
    {
      float det;
      det = pRot.r1_c1*pRot.r2_c2 - pRot.r1_c2*pRot.r2_c1;
      return det;
    }

  }
}


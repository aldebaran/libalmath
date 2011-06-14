/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alposition6d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {

    Position6D Position6D::operator+ (const Position6D& pPos2) const
    {
      Position6D res;
      res.x  = x + pPos2.x;
      res.y  = y + pPos2.y;
      res.z  = z + pPos2.z;
      res.wx = wx + pPos2.wx;
      res.wy = wy + pPos2.wy;
      res.wz = wz + pPos2.wz;
      return res;
    }

    Position6D Position6D::operator- (const Position6D& pPos2) const
    {
      Position6D res;
      res.x  = x - pPos2.x;
      res.y  = y - pPos2.y;
      res.z  = z - pPos2.z;
      res.wx = wx - pPos2.wx;
      res.wy = wy - pPos2.wy;
      res.wz = wz - pPos2.wz;
      return res;
    }

    Position6D Position6D::operator+ () const
    {
      Position6D res;
      res.x  = x;
      res.y  = y;
      res.z  = z;
      res.wx = wx;
      res.wy = wy;
      res.wz = wz;
      return res;
    }

    Position6D Position6D::operator- () const
    {
      Position6D res;
      res.x  = -x;
      res.y  = -y;
      res.z  = -z;
      res.wx = -wx;
      res.wy = -wy;
      res.wz = -wz;
      return res;
    }


    Position6D& Position6D::operator+= (const Position6D& pPos2)
    {
      x  += pPos2.x;
      y  += pPos2.y;
      z  += pPos2.z;
      wx += pPos2.wx;
      wy += pPos2.wy;
      wz += pPos2.wz;
      return *this;
    }


    Position6D& Position6D::operator-= (const Position6D& pPos2)
    {
      x  -= pPos2.x;
      y  -= pPos2.y;
      z  -= pPos2.z;
      wx -= pPos2.wx;
      wy -= pPos2.wy;
      wz -= pPos2.wz;
      return *this;
    }


    bool Position6D::isNear(
      const Position6D& pPos,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(x - pPos.x) > pEpsilon) ||
        (fabsf(y - pPos.y) > pEpsilon) ||
        (fabsf(z - pPos.z) > pEpsilon) ||
        (fabsf(wx - pPos.wx) > pEpsilon) ||
        (fabsf(wy - pPos.wy) > pEpsilon) ||
        (fabsf(wz - pPos.wz) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    Position6D Position6D::operator* (float pM) const
    {
      Position6D res;
      res.x = x * pM;
      res.y = y * pM;
      res.z = z * pM;
      res.wx = wx * pM;
      res.wy = wy * pM;
      res.wz = wz * pM;
      return res;
    }

    Position6D Position6D::operator/ (float pM) const
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: operator/ Division by zeros.");
      }
      return *this * (1.0f/pM);
    }

    Position6D& Position6D::operator*= (const float pM)
    {
      x *=pM;
      y *=pM;
      z *=pM;
      wx *=pM;
      wy *=pM;
      wz *=pM;
      return *this;
    }

    Position6D& Position6D::operator/= (float pM)
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: operator/= Division by zeros.");
      }
      *this *= (1.0f/pM);
      return *this;
    }

    float Position6D::distance(const Position6D& pPos) const
    {
      return Math::distance(*this, pPos);
    }

    float Position6D::distanceSquared(const Position6D& pPos) const
    {
      return Math::distanceSquared(*this, pPos);
    }

    float Position6D::norm() const
    {
      return Math::norm(*this);
    }

    std::vector<float> Position6D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(6);
      returnVector[0] = x;
      returnVector[1] = y;
      returnVector[2] = z;
      returnVector[3] = wx;
      returnVector[4] = wy;
      returnVector[5] = wz;

      return returnVector;
    }

    float distanceSquared(
      const Position6D& pPos1,
      const Position6D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+
          (pPos1.y-pPos2.y)*(pPos1.y-pPos2.y)+
          (pPos1.z-pPos2.z)*(pPos1.z-pPos2.z);
    }

    float distance(
      const Position6D& pPos1,
      const Position6D& pPos2)
    {
      return sqrtf(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position6D& p)
    {
      return sqrtf( (p.x*p.x) + (p.y*p.y) + (p.z*p.z) +
                   (p.wx*p.wx) + (p.wy*p.wy) + (p.wz*p.wz) );
    }

    Position6D normalize(const Position6D& p)
    {
      Position6D ret;
      ret = p;
      float tmpNorm = norm(p);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: normalize Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }

  } // end namespace math
} // end namespace al


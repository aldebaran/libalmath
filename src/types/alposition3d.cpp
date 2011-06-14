/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alposition3d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {


    Position3D Position3D::operator+ (const Position3D& pPos2) const
    {
      Position3D res;
      res.x = x + pPos2.x;
      res.y = y + pPos2.y;
      res.z = z + pPos2.z;
      return res;
    }

    Position3D Position3D::operator- (const Position3D& pPos2) const
    {
      Position3D res;
      res.x = x - pPos2.x;
      res.y = y - pPos2.y;
      res.z = z - pPos2.z;
      return res;
    }

    Position3D Position3D::operator+ () const
    {
      Position3D res;
      res.x = x;
      res.y = y;
      res.z = z;
      return res;
    }

    Position3D Position3D::operator- () const
    {
      Position3D res;
      res.x = -x;
      res.y = -y;
      res.z = -z;
      return res;
    }


    Position3D& Position3D::operator+= (const Position3D& pPos2)
    {
      x += pPos2.x;
      y += pPos2.y;
      z += pPos2.z;
      return *this;
    }

    Position3D& Position3D::operator-= (const Position3D& pPos2)
    {
      x -= pPos2.x;
      y -= pPos2.y;
      z -= pPos2.z;
      return *this;
    }


    bool Position3D::isNear(
      const Position3D& pPos,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(x - pPos.x) > pEpsilon) ||
        (fabsf(y - pPos.y) > pEpsilon) ||
        (fabsf(z - pPos.z) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    Position3D Position3D::operator* (float pM) const
    {
      Position3D res;
      res.x = x * pM;
      res.y = y * pM;
      res.z = z * pM;
      return res;
    }

    Position3D operator* (
      const float       pM,
      const Position3D& pPos1)
    {
      return pPos1 * pM;
    }

    Position3D Position3D::operator/ (float pM) const
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: operator/ Division by zeros.");
      }
      return *this * (1.0f/pM);
    }

    Position3D operator/ (
      const float       pM,
      const Position3D& pPos1)
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: operator/ Division by zeros.");
      }
      return (1.0f/pM) * pPos1;
    }

    Position3D& Position3D::operator*= (float pM)
    {
      x *=pM;
      y *=pM;
      z *=pM;
      return *this;
    }

    Position3D& Position3D::operator/= (float pM)
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: operator/= Division by zeros.");
      }
      *this *= (1.0f/pM);
      return *this;
    }

    bool Position3D::operator== (const Position3D& pPos2) const
    {
      if(
        (x == pPos2.x) &&
        (y == pPos2.y) &&
        (z == pPos2.z))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool Position3D::operator!= (const Position3D& pPos2) const
    {
      return !(*this==pPos2);
    }

    float Position3D::distanceSquared(const Position3D& pPos) const
    {
      return Math::distanceSquared(*this, pPos);
    }

    float Position3D::distance(const Position3D& pPos2) const
    {
      return Math::distance(*this, pPos2);
    }

    float Position3D::norm() const
    {
      return Math::norm(*this);
    }

    Position3D Position3D::normalize() const
    {
      return Math::normalize(*this);
    }

    float Position3D::dotProduct(const Position3D& p) const
    {
      return Math::dotProduct(*this, p);
    }

    Position3D Position3D::crossProduct(const Position3D& p) const
    {
      return Math::crossProduct(*this, p);
    }

    std::vector<float> Position3D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(3);
      returnVector[0] = x;
      returnVector[1] = y;
      returnVector[2] = z;

      return returnVector;
    }


    float distanceSquared(
      const Position3D& pPos1,
      const Position3D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+
          (pPos1.y-pPos2.y)*(pPos1.y-pPos2.y)+
          (pPos1.z-pPos2.z)*(pPos1.z-pPos2.z);
    }


    float distance(
      const Position3D& pPos1,
      const Position3D& pPos2)
    {
      return sqrtf(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position3D& p)
    {
      return sqrtf( (p.x*p.x) + (p.y*p.y) + (p.z*p.z) );
    }

    Position3D normalize(const Position3D& p)
    {
      Position3D ret;
      ret = p;
      float tmpNorm = norm(p);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: normalize Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }

    float dotProduct(
      const Position3D& p1,
      const Position3D& p2)
    {
      return (p1.x * p2.x + p1.y * p2.y + p1.z * p2.z);
    }

    void crossProduct(
      const Position3D& p1,
      const Position3D& p2,
      Position3D&       pRes)
    {
      pRes.x = p1.y*p2.z - p1.z*p2.y;
      pRes.y = p1.z*p2.x - p1.x*p2.z;
      pRes.z = p1.x*p2.y - p1.y*p2.x;
    }

    Position3D crossProduct(
      const Position3D& p1,
      const Position3D& p2)
    {
      Position3D res;
      crossProduct(p1, p2, res);
      return res;
    }

  } // end namespace math
} // end namespace al


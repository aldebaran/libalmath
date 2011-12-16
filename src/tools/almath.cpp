/*
 * Copyright (c) 2012, Aldebaran Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Aldebaran Robotics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Aldebaran Robotics BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <almath/tools/almath.h>

#include <cmath>

namespace AL
{
  namespace Math
  {

    bool clipData(
      const float& pMin,
      const float& pMax,
      float& pData)
    {
      if (pData < pMin)
      {
        pData = pMin;
        return true;
      }
      else if (pData > pMax)
      {
        pData = pMax;
        return true;
      }
      else
      {
        return false;
      }
    }


    Position6D position6DFromVelocity6D(const Velocity6D& pVel)
    {
      Position6D pos;
      pos.x  = pVel.xd;
      pos.y  = pVel.yd;
      pos.z  = pVel.zd;
      pos.wx = pVel.wxd;
      pos.wy = pVel.wyd;
      pos.wz = pVel.wzd;
      return pos;
    }

    Position3D operator*(
      const Rotation&   pRot,
      const Position3D& pPos)
    {
      Position3D result;
      result.x = (pRot.r1_c1 * pPos.x) + (pRot.r1_c2 * pPos.y) + (pRot.r1_c3 * pPos.z);
      result.y = (pRot.r2_c1 * pPos.x) + (pRot.r2_c2 * pPos.y) + (pRot.r2_c3 * pPos.z);
      result.z = (pRot.r3_c1 * pPos.x) + (pRot.r3_c2 * pPos.y) + (pRot.r3_c3 * pPos.z);
      return result;
    }

    Velocity6D operator*(
      const float       pVal,
      const Position6D& pPos)
    {
      /** cyrille 27/04/2009 static ? **/

      Velocity6D pVel;
      pVel.xd  = pVal * pPos.x;
      pVel.yd  = pVal * pPos.y;
      pVel.zd  = pVal * pPos.z;
      pVel.wxd = pVal * pPos.wx;
      pVel.wyd = pVal * pPos.wy;
      pVel.wzd = pVal * pPos.wz;
      return pVel;
    }


    AL::Math::Rotation rotationFromAngleDirection(
      const float&                pTheta,
      const AL::Math::Position3D& pPos)
    {
      return AL::Math::rotationFromAngleDirection(
            pTheta,
            pPos.x,
            pPos.y,
            pPos.z);
    }

  } // namespace Math
} // namespace AL


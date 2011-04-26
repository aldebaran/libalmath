/**
* @author David GOUAILLIER
* Aldebaran Robotics (c) 2009 All Rights Reserved
*
* Inspired by paper : Algorithms for Cubic Spline Interpolation.pdf
* \\sirius\documents\Biblio Motion\david\Interpolation\
*/
#include <almath/interpolation/alinterpolationcubicspline.h>

namespace AL
{
  namespace Math
  {
    namespace Interpolation
    {

#define DEL_ARRAY_IF_NOT_NULL(x) if (x!=NULL){delete[] x; x = NULL;}

      // Constructeur Default
      ALInterpolationCubicSpline::ALInterpolationCubicSpline(
        const unsigned int pNbPoints,
        const unsigned int pSampleDuration):
          fNbPoints(pNbPoints),
          fSampleDuration(pSampleDuration),
          fTime(NULL),
          fa(NULL),
          fb(NULL),
          fc(NULL),
          fd(NULL)
      {
        fTime           = new float[fNbPoints];
        fa              = new float[fNbPoints-1];
        fb              = new float[fNbPoints-1];
        fc              = new float[fNbPoints-1];
        fd              = new float[fNbPoints-1];
      }


      void ALInterpolationCubicSpline::init(
        const float* pPoints,
        unsigned int pQpRatio)
      {
        // 1 :  Compute Time Array
        for (unsigned int i=0; i<fNbPoints; i++)
        {
          fTime[i] = (float)(i*pQpRatio*fSampleDuration);
        }

        // 2 : Compute h and b
        float* h  = new float[fNbPoints-1];   // Array diff time control points
        float* b  = new float[fNbPoints-1];   // Array vitesse at control points
        for(unsigned int i=0; i<(fNbPoints-1); i++)
        {
          h[i] = fTime[i+1]-fTime[i];
          b[i] = ( pPoints[i+1]-pPoints[i] )/h[i];
        }

        // 3 : Gaussian Elimination
        float* u = new float[fNbPoints-1];    // for gaussian elimination
        float* v = new float[fNbPoints-1];    // for gaussian elimination
        u[0] = 2.0f * (h[0] + h[1]);
        v[0] = 6.0f * (b[1] - b[0]);

        for (unsigned int i=1; i<(fNbPoints-1); i++)
        {
          u[i] = 2.0f * (h[i-1] + h[i]) - h[i-1]*h[i-1]/u[i-1];
          v[i] = 6.0f * (b[i] - b[i-1]) - h[i-1]*v[i-1]/u[i-1];
        }

        // 4 : Back Substitution
        float* z= new float[fNbPoints];       // back Substitution
        z[fNbPoints-1] = 0.0f;
        for (unsigned int i=(fNbPoints-2); i>0; i--)
        {
          z[i] = (v[i]-h[i]*z[i+1])/u[i];
        }
        z[0] = 0.0f;

        // 5 : Compute spline Coefficient
        for (unsigned int i=0; i<(fNbPoints-1); i++)
        {
          fa[i] = pPoints[i];
          fb[i] = -h[i]*z[i+1]/6.0f - z[i]*h[i]/3.0f + (pPoints[i+1]-pPoints[i])/h[i];
          fc[i] = z[i]/2.0f;
          fd[i] = (z[i+1]-z[i]) / (6.0f*h[i]);
        }

        // 6 : clean array
        DEL_ARRAY_IF_NOT_NULL(h);
        DEL_ARRAY_IF_NOT_NULL(b);
        DEL_ARRAY_IF_NOT_NULL(u);
        DEL_ARRAY_IF_NOT_NULL(v);
        DEL_ARRAY_IF_NOT_NULL(z);
      }


      float ALInterpolationCubicSpline::getInterpolation(
          const unsigned int pDesiredTime)
      {
        unsigned int i;
        for (i=0; i<(fNbPoints-1); i++)
        {
          if ((float)pDesiredTime <= fTime[i+1])
          {
            break;
          }
        }
        float t = ( (float)pDesiredTime - fTime[i] );
        float t2 = t*t;
        float t3 = t2*t;

        return fa[i] + fb[i]*t + fc[i]*t2 + fd[i]*t3;
      }


      ALInterpolationCubicSpline::~ALInterpolationCubicSpline()
      {
        DEL_ARRAY_IF_NOT_NULL(fTime);
        DEL_ARRAY_IF_NOT_NULL(fa);
        DEL_ARRAY_IF_NOT_NULL(fb);
        DEL_ARRAY_IF_NOT_NULL(fc);
        DEL_ARRAY_IF_NOT_NULL(fd);
      }

    }
  }
}


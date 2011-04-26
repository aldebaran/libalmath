/**
*
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/
#include <almath/tools/altrigonometry.h>

namespace AL {
  namespace Math {


    bool inlineCosSinInitialized = false;
    bool inlineACosASinInitialized = false;
    float inlineCosArray[inlineCosSinArraySize];
    float inlineSinArray[inlineCosSinArraySize];
    float inlineACosArray[inlineACosASinArraySize];
    float inlineASinArray[inlineACosASinArraySize];

    /*
    * initialize the tables
    */
    void initInlineCosSin()
    {
      int i = -1;
      float *p_cos = inlineCosArray, *p_sin = inlineSinArray;

      while ( ++i < inlineCosSinArraySize)
      {
        *p_cos++ = cosf(0.0f + (float)i * stepForInlineCosSin);
        *p_sin++ = sinf(0.0f + (float)i * stepForInlineCosSin);
      }
      inlineCosSinInitialized = true;
    }

    void initInlineACosASin()
    {
      int i = -1;
      float *p_acos = inlineACosArray, *p_asin = inlineASinArray;

      while ( ++i < inlineACosASinArraySize-1)
      {
        *p_acos++ = acosf(-1.0f + (float)i * stepForInlineACosASin);
        *p_asin++ = asinf(-1.0f + (float)i * stepForInlineACosASin);
      }

      inlineACosArray[inlineACosASinArraySize-1] = 0.0f;
      inlineASinArray[inlineACosASinArraySize-1] = PI_2;

      inlineACosASinInitialized = true;
    }

    /*
    * return cos(n) in rad
    */
    float inlineCos(float n)
    {
      if(!inlineCosSinInitialized)
        initInlineCosSin();

      if(n < 0.0f)
        n = -n;
      //a - (a >> 14)<<14];// % inlineCosSinArraySize ];
      return inlineCosArray[ (int)(n*freqForInlineCosSin) % inlineCosSinArraySize ];
    }

    /*
    * return sin(n) in rad
    */
    float inlineSin(float n)
    {
      if(!inlineCosSinInitialized)
        initInlineCosSin();

      if(n < 0.0f)
        return -inlineSinArray[ (int)(-n*freqForInlineCosSin) % inlineCosSinArraySize ];

      return inlineSinArray[ (int)(n*freqForInlineCosSin) % inlineCosSinArraySize ];
    }


    /*
    * return ArcCos(n) if -1 <= n <= 1
    * return value between 0 and PI
    * TODO : Exceptions
    */
    float inlineACos(float n)
    {
      if(!inlineACosASinInitialized)
        initInlineACosASin();

      if (n == 1.0f)
        return 0.0f;
      // May cause a segmentation fault if n<(-1) OR n>1
      return inlineACosArray[(int)( (n + 1.0f) *freqForInlineACosASin)];
    }

    /*
    * return ArcSin(n) if -1 <= n <= 1
    * return value between -PI_2 and +PI_2
    * TODO : Exceptions
    */
    float inlineASin(float n)
    {
      if(!inlineACosASinInitialized)
        initInlineACosASin();

      if (n == 1.0f)
        return PI_2;
      // May cause a segmentation fault if n<(-1) OR n>1
      return inlineASinArray[(int)( (n + 1.0f) *freqForInlineACosASin)];
    }

  } // namespace Math
} // namespace AL


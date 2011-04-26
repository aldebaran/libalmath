/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOLYNOMIALSOLVER_H_
#define _LIB_ALMATH_ALMATH_ALPOLYNOMIALSOLVER_H_

#include <math.h>
#include <stdlib.h>

#include <almath/tools/almath.h>

#define PI 3.14159265358979f

namespace AL
{
  namespace Math
  {

    /// <summary> Radical cubique du nombre. </summary>
    /// <param name="nb"> Nombre dont on veut la racine cubique. </param>
    /// <returns> Solution. </returns>
    float radcubique(float nb);

    /// <summary> Radical cubique du nombre. </summary>
    /// <param name="nb"> Nombre dont on veut la racine cubique. </param>
    /// <returns> Solution. </returns>
    double radcubique(double nb);

    /// <summary> Resolution d'une equation de polynome du premier degre (ax+b=0). </summary>
    /// <param name="a"> Polynomial coefficient. </param>
    /// <param name="b"> Polynomial coefficient. </param>
    /// <param name="buffer"> Complex solutions. </param>
    /// <returns> Number of solution. </returns>
    unsigned int Premierdegre(
      float     a,
      float     b,
      Complex buffer[1]);


    /// <summary> Resolution d'une equation de polynome du deuxieme degre (ax^2+bx^2+c=0). </summary>
    /// <param name="a"> Polynomial coefficient. </param>
    /// <param name="b"> Polynomial coefficient. </param>
    /// <param name="c"> Polynomial coefficient. </param>
    /// <param name="buffer"> Complex solutions. </param>
    /// <returns> Number of solution. </returns>
    unsigned int Deuxiemedegre(
      float     a,
      float     b,
      float     c,
      Complex buffer[2]);

    /// <summary> Resolution d'une equation de polynome du troisieme degre type (ax^3+bx^2+cx+d=0) (ce type d'equation a toujours au moins une racine reelle). </summary>
    /// <param name="a"> Polynomial coefficient. </param>
    /// <param name="b"> Polynomial coefficient. </param>
    /// <param name="c"> Polynomial coefficient. </param>
    /// <param name="d"> Polynomial coefficient. </param>
    /// <param name="buffer"> Complex solutions. </param>
    /// <returns> Number of solution. </returns>
    unsigned int Troisiemedegre(
      float     a,
      float     b,
      float     c,
      float     d,
      Complex buffer[3]);


    ////ATTENTION : FONCTION BUGGER
    // /// <summary> Resolution d'une equation de polynome du quatrieme degre type (ax^4+bx^3+cx^2+dx+e=0). </summary>
    // /// <param name="a"> Polynomial coefficient. </param>
    // /// <param name="b"> Polynomial coefficient. </param>
    // /// <param name="c"> Polynomial coefficient. </param>
    // /// <param name="d"> Polynomial coefficient. </param>
    // /// <param name="e"> Polynomial coefficient. </param>
    // /// <param name="buffer"> Complex solutions. </param>
    // /// <returns> Number of solution. </returns>
    // ////Resoud une equation du quatrieme degre, renvoye le nombre de racines
    //unsigned int Quatriemedegre(
    //  float     a,
    //  float     b,
    //  float     c,
    //  float     d,
    //  float     e,
    //  Complex buffer[4]);


  }
}

#endif  // _LIB_ALMATH_ALMATH_ALPOLYNOMIALSOLVER_H_


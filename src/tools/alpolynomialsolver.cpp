/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*/

#include <almath/tools/alpolynomialsolver.h>

#include <vector>
#include <algorithm>

namespace AL
{
  namespace Math
  {

    // Complex Comparator, used to sort solutions at the end.
    struct ComplexComparator
    {
      bool operator() (
        const Complex& a,
        const Complex& b)
      {
        if (a.re < b.re)
          return true;
        if (a.re == b.re)
          return a.im < b.im;
        return false;
      }
    } complexComparator;


    // Sort complex array in place, using complexSorter comparison.
    void sortComplexArray(
      Complex* array,
      int num)
    {
      // Sort solutions.
      std::vector<Complex> tmpVect(array, array + num);
      std::sort(tmpVect.begin(), tmpVect.end(), complexComparator);
      for (int i = 0; i < num; i++)
      {
        array[i] = tmpVect[i];
      }
    }


    //retourne le radical cubique du nombre
    float radcubique(float nb)
    {
      if (nb > 0.0f)
      {
        //pow ne marche pas bien avec des nombre negatifs
        return powf(nb, 1.0f/3.0f);
      }
      else
      {
        return -powf(-nb, 1.0f/3.0f);
      }
    } // end radcubique


    //retourne le radical cubique du nombre
    double radcubique(double nb)
    {
      if (nb > 0.0)
      {
        //pow ne marche pas bien avec des nombre negatifs
        return pow(nb, 1.0/3.0);
      }
      else
      {
        return -pow(-nb, 1.0/3.0);
      }
    } // end radcubique


    //Donne la racine d un polynome de degre 1 retourne le nombre de racines
    unsigned int Premierdegre(
      float     a,
      float     b,
      AL::Math::Complex buffer[1])
    {
      if (a == 0.0f)
      {
        return 0;
      }

      buffer[0].re = -b/a;
      buffer[0].im = 0.0f;
      return 1;
    }


    //Resolution d une equation du deuxieme degre retourne le nombre de racines
    unsigned int Deuxiemedegre(
      float     a,
      float     b,
      float     c,
      AL::Math::Complex buffer[2]) //complexe
    {
      float delta;

      if (a == 0.0f)
      {
        return Premierdegre(b, c, buffer);
      }

      delta = b*b - 4.0f*a*c;

      if (delta >= 0.0f)    //racines reelles
      {
        buffer[0].re = -0.5f*(b+sqrt(delta))/a; // was (-b-sqrt(delta))/(2.0f*a)
        buffer[0].im = 0.0f;
        buffer[1].re = 0.5f*(-b+sqrt(delta))/a; // was (-b+sqrt(delta))/(2.0f*a)
        buffer[1].im = 0.0f;
      }
      else      //racines complexes conjugees
      {
        buffer[0].re = -0.5f*b/a; // was -b/(2.0f*a)
        buffer[0].im = 0.5f*(-sqrt(-delta))/a; // was (-sqrt(-delta))/(2.0f*a)
        buffer[1].re = buffer[0].re;
        buffer[1].im =  -buffer[0].im;
      }
      sortComplexArray(buffer, 2);
      return 2;
    }


    //Resolution d'une equation du troisieme degre type (ax^3+bx^2+cx+d=0)
    //(ce type d'equation a toujours au moins une racine reelle)
    //Retourne le nombre de racines
    unsigned int Troisiemedegre(
      float     a,
      float     b,
      float     c,
      float     d,
      AL::Math::Complex buffer[3]) //complexe
    {

      if (a == 0.0f)
      {
        return Deuxiemedegre(b, c, d, buffer);
      }

      // on divise tout par a et on pose x=X-b/(3a)
      // on arrive a une equation du type X^3+pX+q = 0 avec

      // facile a retrouver en posant le calcul

      //relations entre les racines
      //X1+X2+X3 = 0
      //X1*X2 + X2*X3 + X1*X3 = p
      //X1*X2*X3 = -q

      //on pose X = u+v en imposant 3u*v + p = 0
      //en remplacant on tombe sur l'equation u^3+v^3 = q
      //or on a uv = -p/3 ie u^3*v^3 = -p^3/27

      //On a la somme et le produit, on peut trouver u^3 et v^3
      //u^3 et v^3 solutions de Y^2 + qY -p^3/27 = 0


      // avec des floats
      // test 9 avec delta == 0 marche sous windows
      // test 9 avec delta == 0 marche pas sous linux
      // sous windows, avec delta>= 0.0, tous les tests marchent
      // si on fait
      float p = 0.0f;
      float q = 0.0f;
      float delta = 0.0f;
      p = -b*b/(3.0f*a*a) + c/a;
      q = 2.0f*b*b*b/(27.0f*a*a*a) - b*c/(3.0f*a*a) + d/a;
      delta = 0.25f*q*q + p*p*p/27.0f;

      //// avec des doubles
      // test 9 marche pas sous windows
      // test 9 marche sous linux
      //double p = 0.0;
      //double q = 0.0;
      //double delta = 0.0;
      //p = -b*b/(3.0*a*a) + c/a;
      //q = 2.0*b*b*b/(27.0*a*a*a) - b*c/(3.0*a*a) + d/a;
      //delta = 0.25*q*q + p*p*p/27.0;


      //X1 = u+v, c'est la somme des racines cubiques des solutions
      //Et avec les relations coefficients-racines on a X2+X3 = -X1  et X2*X3 = -q/X1
      //donc X2 et X3 solutions de Z^2+X1*Z-q/X1
      if (delta >= 0.0f)
      {
        buffer[0].re = radcubique(-0.5f*q + sqrt(delta)) + radcubique(-0.5f*q - sqrt(delta));
        buffer[0].im = 0.0f;
        if (buffer[0].re != 0.0f)
        {
          Deuxiemedegre(1.0f, buffer[0].re, -q/buffer[0].re, &buffer[1]);
        }
        else  //sinon on peut factoriser par X (ca donne X^2+p=0)
        {
          Deuxiemedegre(1.0f, 0.0f, p, &buffer[1]);
        }

        //on a trois solutions de l'equation en X mais on a pose x=X-b/3a
        //il suffit donc de faire -b/3a sur les parties reelles des solutions en X

        buffer[0].re += -b/(3.0f*a);
        buffer[1].re += -b/(3.0f*a);
        buffer[2].re += -b/(3.0f*a);
      }
      else if ( delta == 0.0 ) // n'y rentre jamais maintenant
      {
        // On a ici q^2/4 = - p^3/27 et, necessairement p <= 0 :
        // on est enclin a envisager l'existence d'une solution
        // double voire triple. Si tel est le cas, cette solution
        // annule 3*X^2 + p, expression derivee de l'equation.
        // On verifie qu'il en est ainsi et que la 3eme solution est alors :

        if (p == 0.0f)
        {
          buffer[0].re = -b/(3.0f*a);
          buffer[1].re = -b/(3.0f*a);
          buffer[2].re = -b/(3.0f*a);
        }
        else
        {
          buffer[0].re = -sqrt(-p/3.0f) - b/(3.0f*a);
          buffer[1].re = -sqrt(-p/3.0f) - b/(3.0f*a);
          buffer[2].re = (3.0f*q/p) - b/(3.0f*a);
        }

        buffer[0].im = 0.0f;
        buffer[1].im = 0.0f;
        buffer[2].im = 0.0f;


        //// **** BEGIN SOLUTION FAUSSE ****
        //// on ne peut pas rallier ce cas a celui du dessus car le radical cubique d'un complexe
        //// est difficile a exprimer (on peut le faire en fonction d'arctan et de sin)
        //// Mais si delta == 0 on a une solution double donc en cette solution la derivee
        //// du polynome s'annule aussi en ce point ie 3X^2+p=0
        //buffer[0].re = sqrt(-p/3.0f);
        //buffer[0].im = 0.0f;
        //if (buffer[0].re != 0.0f)
        //{
        //  Deuxiemedegre(1.0f, buffer[0].re, -q/buffer[0].re, &buffer[1]);
        //}
        //else  //sinon on peut factoriser par X
        //{
        //  Deuxiemedegre(1.0f, 0.0f, p, &buffer[1]);
        //}

        ////pareil que si delta > 0
        //buffer[0].re = buffer[0].re -b/(3.0f*a);
        //buffer[1].re = buffer[1].re -b/(3.0f*a);
        //buffer[2].re = buffer[2].re -b/(3.0f*a);

        ////buffer[0].re += -b/(3.0f*a);
        ////buffer[1].re += -b/(3.0f*a);
        ////buffer[2].re += -b/(3.0f*a);
        //// **** END SOLUTION FAUSSE ****
      }
      else      //trois racines reelles
      {
        //si delta < 0 on fait une autre methode
        //u^3 et v^3 solutions de l'equation sont des complexes conjuges (delta<0)
        //ecrivons u^3 sous forme exponentielle
        //u^3 = r*e^(it) = -q/2+i*sqrt(delta) (solution de l'eq du deuxieme deg)
        //r est le module egal a la racine carree de la somme des carrees de la partie
        //imaginaire et reelle, en reprenant l'expression de delta en trouve r=sqrt(-p^3/27)
        //(p<0  car delta < 0)
        //e^(it) = cos(t)+i*sint(t), en identifiant partie imaginaire et reelle
        //on trouve cos(t) = -q/(2r) (t =acos(-q/(2r)))
        //u^3 et v^3 conjugees donc les solutions sont reelles
        //les trois solutions sont les uk+vk avec uk = sqrt(-p/3)*e((t+2*k*PI)/3)
        //                      vk = uk(barre)
        //Xk = 2*sqrt(-p/3)*cos((t+2k*PI)/3)
        //xk = Xk - b/(3a)

        float t = 0.0f;
        float r = 0.0f;
        float arg = 0.0f;

        r = sqrt(-p/3.0f);
        arg = -q/(2.0f*sqrt(-p*p*p/27.0f));

        t = acos(arg);

        for (unsigned i=0; i<3; i++)
        {
          buffer[i].re = 2.0f*sqrt(-p/3.0f)*cos((t + 2.0f*i*PI)/3.0f) - b/(3.0f*a);
          buffer[i].im = 0.0f;
        }
      }

      sortComplexArray(buffer, 3);

      return 3;
    }


    //// ATTENTION : c'est bugger, je la commente
    ////Resoudre une equation du quatrieme degre, renvoye le nombre de racines
    //unsigned int Quatriemedegre(
    //  float     a,
    //  float     b,
    //  float     c,
    //  float     d,
    //  float     e,
    //  AL::Math::Complex buffer[4]) //complexe
    //{
    //  float A,B,C;
    //  float Z;
    //  float u;
    //  float x,y;

    //  AL::Math::Complex sol3[3];
    //  AL::Math::Complex sol2[2];

    //  if (a == 0.0f)
    //  {
    //    return Troisiemedegre(b, c, d, e, buffer);
    //  }

    //  //on divise par a puis on remplace x=X-b/4a
    //  //on tombe sur un polynome du type X^4+AX^2+BX+C avec
    //  A = (-3.0f*b*b)/(8.0f*a*a) + c/a;
    //  B = (b*b*b)/(8.0f*a*a*a) - (b*c)/(2.0f*a*a) + d/a;
    //  C = -3.0f*(b*b*b*b)/(256.0f*a*a*a*a) + (c*b*b)/(16.0f*a*a*a) - (d*b)/(4.0f*a*a) +e/a;

    //  //Si B=0 on sait resoudre ! en prenant Z = X^2
    //  if (B == 0.0f)
    //  {
    //    //on calcule les Z
    //    Deuxiemedegre(1.0f, A, C, sol2);

    //    //On fait ensuite + ou - la racine des sols pour avoir les X
    //    //Attention !! sqrt(a+i*b) != sqrt(a)+i*sqrt(b)
    //    //Formule generale :
    //    // sqrt (x+iy) =  sqrt(2*(sqrt(x^2+y^2)+x))/2 + i*sqrt(2*(sqrt(x^2+y^2)-x))/2 * signe de y
    //    //on fait -b/4a et on a les x

    //    for (unsigned int i=0; i<4; i++)
    //    {
    //      x = sol2[i % 2].re;
    //      y = sol2[i % 2].im;

    //      buffer[i].re = sqrt(2.0f*(sqrt(x*x + y*y)+x))/2.0f*(1.0f - (i/2.0f)*2.0f) - b/(4.0f*a);
    //      buffer[i].im = sqrt(2.0f*(sqrt(x*x + y*y)-x))/2.0f*(1.0f - (i/2.0f)*2.0f);
    //      if (y < 0.0f)
    //      {
    //        buffer[i].im = -buffer[i].im;
    //      }
    //    }
    //  }
    //  else
    //  {

    //    //On suppose X racine  : on essaye de factoriser X^4+AX^2 en (X^2+u/2)^2
    //    //or (X^2+u/2)^2 = X^4 + uX^2 + u^2/4
    //    //et X^4 = -AX^2 - BX - C (car X racine du polynome)
    //    //Donc au final (X^2+u/2)^2 = (u-A)X^2 - BX + (u^2)/4 - C
    //    //On a une equation du second degre, on cherche u tel que
    //    //  * delta = 0
    //    //  * u != A

    //    //On calcule le discriminant et on veut u = 0
    //    //on tombe sur  u^3 - A*u^2 -4 *C*u + 4*A*C - B^2 = 0
    //    //si u = A alors B serait egal a 0 ce qui n'est pas le cas (distinction faite plus haut)
    //    //on calcule les valeurs possibles de u
    //    Troisiemedegre(1.0f, -A, -4.0f*C, 4.0f*A*C-B*B, sol3);

    //    //on prend le u reel le plus grand (voir pour la suite)
    //    u=sol3[0].re;
    //    //Par convention, ma fonction troisieme construit sol3 tq sol3[0] est reel

    //    for (unsigned int i=0; i<3; i++)
    //    {
    //      if (sol3[i].im == 0.0f && sol3[i].re > u)
    //      {
    //        u = sol3[i].re;
    //      }
    //    }

    //    //delta = 0 donc Z est solution double de (u-A)X^2 - bX + (u^2)/4 - C
    //    // avec Z = B/(2*(u-A))
    //    //on peut factoriser

    //    Z = B/(2.0f*(u-A));

    //    //(X^2+u/2)^2 = (u-A)(X-Z)^2
    //    //Il existe au moins un u tel que u>A d'apres l'equation dont u est racine
    //    //les X solutions sont donc les sols de
    //    // X^2+u/2 = sqrt (u-A)*(X-Z)
    //    // X^2+u/2 = -sqrt (u-A)*(X-Z)
    //    Deuxiemedegre(1.0f,-sqrt(u-A), u/2.0f + Z*sqrt(u-A), buffer);
    //    Deuxiemedegre(1.0f, sqrt(u-A), u/2.0f - Z*sqrt(u-A), &buffer[2]);

    //    //on fait -4b/a pour chaque solution pour avoir les x
    //    for (unsigned int i=0; i<4; i++)
    //      buffer[i].re -= b/(4.0f*a);
    //  }
    //  return 4;
    //}


  } // namespace Math
} // namespace AL


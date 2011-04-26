/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2009 All Rights Reserved
 *
 */
#include <almath/tools/alpolynomialsolver.h>
#include <almath/tools/almath.h>
#include "../almathtestutils.h"

AL::Math::Complex pSol[4];
float a = 0.0f;
float b = 0.0f;
float c = 0.0f;
float d = 0.0f;
float e = 0.0f;
int nbSol = 0;

TEST(ALPolynomialSolverTest, radcubique0)
{
  EXPECT_NEAR(AL::Math::radcubique(1.0f), 1.0f, kEpsilon);
}


TEST(ALPolynomialSolverTest, order2)
{
  a = 0.0f;
  b = 0.0f;
  c = 0.0f;

  //std::cout << "***** Exemple 0 *****" << std::endl;
  a = 0.0f;
  b = 0.0f;
  c = 0.0f;
  nbSol = AL::Math::Deuxiemedegre(a, b, c, pSol);
  EXPECT_EQ(nbSol, 0);

  //std::cout << "***** Exemple 1 *****" << std::endl;
  a = 1.0f;
  b = -6.0f;
  c = 5.0f;
  nbSol = AL::Math::Deuxiemedegre(a, b, c, pSol);
  EXPECT_EQ(nbSol, 2);
  compareComplex(pSol[0], AL::Math::Complex(1.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(5.0f, 0.0f));

  //std::cout << "***** Exemple 2 *****" << std::endl;
  a = 4.0f;
  b = -8.0f;
  c = 0.0f;
  nbSol = AL::Math::Deuxiemedegre(a, b, c, pSol);
  EXPECT_EQ(nbSol, 2);
  compareComplex(pSol[0], AL::Math::Complex(0.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(2.0f, 0.0f));

  //std::cout << "***** Exemple 3 *****" << std::endl;
  a = 1.0f;
  b = 0.0f;
  c = 1.0f;
  nbSol = AL::Math::Deuxiemedegre(a, b, c, pSol);
  EXPECT_EQ(nbSol, 2);
  compareComplex(pSol[0], AL::Math::Complex(0.0f, -1.0f));
  compareComplex(pSol[1], AL::Math::Complex(0.0f, 1.0f));
} // end order2


TEST(ALPolynomialSolverTest, order3)
{
  //std::cout << "***** Exemple 0 *****" << std::endl;
  a = 0.0f;
  b = 0.0f;
  c = 0.0f;
  d = 0.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 0);

  //std::cout << "***** Exemple 1 *****" << std::endl;
  a = 1.0f;
  b = 3.0f;
  c = 5.0f;
  d = 6.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-2.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(-0.5f, -1.6583122f));
  compareComplex(pSol[2], AL::Math::Complex(-0.5f, 1.6583122f));

  //std::cout << "***** Exemple 2 *****" << std::endl;
  a = 1.0f;
  b = 3.0f;
  c = 3.0f;
  d = 1.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-1.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(-1.0f, 0.0f));
  compareComplex(pSol[2], AL::Math::Complex(-1.0f, 0.0f));

  //std::cout << "***** Exemple 3 *****" << std::endl;
  a = 2.0f;
  b = 15.0f;
  c = 24.0f;
  d = -16.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-4.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(-4.0f, 0.0f));
  compareComplex(pSol[2], AL::Math::Complex(0.5f, 0.0f));

  //std::cout << "***** Exemple 4 *****" << std::endl;
  a = 4.0f;
  b = -5.0f;
  c = -23.0f;
  d = 6.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-2.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(0.25f, 0.0f));
  compareComplex(pSol[2], AL::Math::Complex(3.0f, 0.0f));

  //std::cout << "***** Exemple 5 *****" << std::endl;
  a = 1.0f;
  b = 0.0f;
  c = 1.0f;
  d = 1.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-0.68232781f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(0.34116390f, -1.1615415f));
  compareComplex(pSol[2], AL::Math::Complex(0.34116390f, 1.1615415f));

  //std::cout << "***** Exemple 6 *****" << std::endl;
  a = 1.0f;
  b = 0.0f;
  c = 0.0f;
  d = 1.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-1.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(0.5f, -0.86602539f));
  compareComplex(pSol[2], AL::Math::Complex(0.5f, +0.86602539f));

  // http://tanopah.jo.free.fr/ADS/bloc1/polyfct3.html
  //std::cout << "***** Exemple 7 *****" << std::endl;
  a = 8.0f;
  b = -4.0f;
  c = 8.0f;
  d = -12.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-0.25f, -1.1989580f));
  compareComplex(pSol[1], AL::Math::Complex(-0.25f, +1.1989580f));
  compareComplex(pSol[2], AL::Math::Complex(1.0f, 0.0f));

  //std::cout << "***** Exemple 8 *****" << std::endl;
  a = 3.0f;
  b = -9.0f;
  c = -18.0f;
  d = +24.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-2.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(1.0f, 0.0f));
  compareComplex(pSol[2], AL::Math::Complex(4.0f, 0.0f));

  //std::cout << "***** Exemple 9 *****" << std::endl;
  a = 1.0f;
  b = -1.0f;
  c = -8.0f;
  d = +12.0f;
  nbSol = AL::Math::Troisiemedegre(a, b, c, d, pSol);
  EXPECT_EQ(nbSol, 3);
  compareComplex(pSol[0], AL::Math::Complex(-3.0f, 0.0f));
  compareComplex(pSol[1], AL::Math::Complex(2.0f, 0.0f), 1.0e-3f);
  compareComplex(pSol[2], AL::Math::Complex(2.0f, 0.0f), 1.0e-3f);


} // end order3

////ATTENTION : ORDER4 EST BUGGER
//TEST(ALPolynomialSolverTest, order4)
//{
//  ////std::cout << "***** Exemple 0 *****" << std::endl;
//  //a = 0.0f;
//  //b = 0.0f;
//  //c = 0.0f;
//  //d = 0.0f;
//  //e = 0.0f;
//  //nbSol = AL::Math::Quatriemedegre(a, b, c, d, e, pSol);
//  //EXPECT_EQ(nbSol, 0);
//
//  ////std::cout << "***** Exemple 1 *****" << std::endl;
//  //a = 1.0f;
//  //b = 0.0f;
//  //c = 0.0f;
//  //d = 0.0f;
//  //e = -1.0f;
//  //nbSol = AL::Math::Quatriemedegre(a, b, c, d, e, pSol);
//  //EXPECT_EQ(nbSol, 4);
//  //compareComplex(pSol[0], AL::Math::Complex(0.0f, 1.0f));
//  //compareComplex(pSol[1], AL::Math::Complex(1.0f, 0.0f));
//  //compareComplex(pSol[2], AL::Math::Complex(0.0f, -1.0f));
//  //compareComplex(pSol[3], AL::Math::Complex(-1.0f, 0.0f));
//
//  ////std::cout << "***** Exemple 2 *****" << std::endl;
//  //a = 1.0f;
//  //b = 0.0f;
//  //c = -4.0f;
//  //d = 0.0f;
//  //e = -45.0f;
//  //nbSol = AL::Math::Quatriemedegre(a, b, c, d, e, pSol);
//  //EXPECT_EQ(nbSol, 4);
//  //compareComplex(pSol[0], AL::Math::Complex(0.0f, 2.23606797749979f));
//  //compareComplex(pSol[1], AL::Math::Complex(3.0f, 0.0f));
//  //compareComplex(pSol[2], AL::Math::Complex(0.0f, -2.23606797749979f));
//  //compareComplex(pSol[3], AL::Math::Complex(-3.0f, 0.0f));
//
//  ////std::cout << "***** Exemple 3 *****" << std::endl;
//  //a = 1.0f;
//  //b = 0.0f;
//  //c = -5.0f;
//  //d = 0.0f;
//  //e = +4.0f;
//  //nbSol = AL::Math::Quatriemedegre(a, b, c, d, e, pSol);
//  //EXPECT_EQ(nbSol, 4);
//  //compareComplex(pSol[0], AL::Math::Complex(1.0f, 0.0f));
//  //compareComplex(pSol[1], AL::Math::Complex(2.0f, 0.0f));
//  //compareComplex(pSol[2], AL::Math::Complex(-1.0f, 0.0f));
//  //compareComplex(pSol[3], AL::Math::Complex(-2.0f, 0.0f));
//
//  ////std::cout << "***** Exemple 4 *****" << std::endl;
//  //a = 1.0f;
//  //b = -10.0f;
//  //c = +35.0f;
//  //d = -50.0f;
//  //e = +24.0f;
//  //nbSol = AL::Math::Quatriemedegre(a, b, c, d, e, pSol);
//  //EXPECT_EQ(nbSol, 4);
//  //compareComplex(pSol[0], AL::Math::Complex(4.0f, 0.0f));
//  //compareComplex(pSol[1], AL::Math::Complex(3.0f, 0.0f));
//  //compareComplex(pSol[2], AL::Math::Complex(2.0f, 0.0f));
//  //compareComplex(pSol[3], AL::Math::Complex(1.0f, 0.0f));
//
//} // end order4


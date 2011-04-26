/**
* @author Aldebaran Robotics
* Copyright (c) Aldebaran Robotics 2007 All Rights Reserved
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_MERSENNETWISTER_H_
#define _LIB_ALMATH_ALMATH_MERSENNETWISTER_H_


namespace mersennetwister
{
  /* initializes state[N] with a seed */
  void init_genrand(unsigned long s);

  /* initialize by an array with array-length */
  /* init_key is the array for initializing keys */
  /* key_length is its length */
  //unsigned long init_key[], key_length;
  void init_by_array(unsigned long init_key[], unsigned long key_length);



  void next_state(void);

  /* generates a random number on [0,0xffffffff]-interval */
  unsigned long genrand_int32(void);

  /* generates a random number on [0,0x7fffffff]-interval */
  long genrand_int31(void);

  /* generates a random number on [0,1]-real-interval */
  double genrand_real1(void);

  /* generates a random number on [0,1)-real-interval */
  double genrand_real2(void);

  /* generates a random number on (0,1)-real-interval */
  double genrand_real3(void);

  /* generates a random number on [0,1) with 53-bit resolution*/
  double genrand_res53(void);
}

#endif  // _LIB_ALMATH_ALMATH_MERSENNETWISTER_H_

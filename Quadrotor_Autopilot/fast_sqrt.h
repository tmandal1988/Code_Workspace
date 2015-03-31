/*
 * fast_sqrt.h
 *
 *  Created on: Mar 23, 2015
 *      Author: Tanmay
 */

#ifndef FAST_SQRT_H_
#define FAST_SQRT_H_

//Log base 2 approximation and Newton's Method
//ref:http://ilab.usc.edu/wiki/index.php/Fast_Square_Root

float fast_sqrt(const float x)
{
  union
  {
    int i;
    float x;
  } u;

  u.x = x;
  u.i = (1<<29) + (u.i >> 1) - (1<<22);
  return u.x;
}

#endif /* FAST_SQRT_H_ */

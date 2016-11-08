#ifndef PSEUDO_RANDOM_HPP_
#define PSEUDO_RANDOM_HPP_

#include <cstdlib>
#include <cmath>

#if defined (WINDOWS) || defined (_WIN32) || defined (__WIN32__)
  #define OS_WINDOWS
  #define srandom srand
  #define random rand
  #define RANDOM_MAX RAND_MAX
#else
  #define RANDOM_MAX 2147483647
#endif

unsigned int PseudoRandomSeed(void);

static inline double PseudoRandomUniformReal(void)
{
    return ((double) random()) / ((double) RANDOM_MAX);
}

static inline double PseudoRandomUniformReal(const double min, const double max)
{
    return min + (max - min) * PseudoRandomUniformReal();
}

#endif












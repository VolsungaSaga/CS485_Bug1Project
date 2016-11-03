#ifndef MY_TIMER_HPP_
#define MY_TIMER_HPP_


#if defined (WINDOWS) || defined (_WIN32) || defined (__WIN32__)
#include <Windows.h>
typedef long long Clock;	
static inline void StartTime(Clock * const c)
{
    QueryPerformanceCounter((LARGE_INTEGER *) c);
}
static inline double ElapsedTime(Clock * const c)
{
    long long end;
    long long freq;
    QueryPerformanceCounter((LARGE_INTEGER *) &end);
    QueryPerformanceFrequency((LARGE_INTEGER *) &freq);    
    return ((double) (end - (*c))) / freq; 
}	
#else
#include <sys/time.h>
#include <sys/resource.h>
typedef struct timeval Clock;	
static inline void StartTime(Clock * const c)
{
    gettimeofday(c, NULL);  
}
static inline double ElapsedTime(Clock * const c)
{
    struct timeval end;
    gettimeofday(&end, NULL);      
    return  (end.tv_sec  - c->tv_sec ) +  0.000001 * (end.tv_usec - c->tv_usec);    
}

#endif

#endif

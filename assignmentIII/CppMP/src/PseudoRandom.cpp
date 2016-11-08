#include "PseudoRandom.hpp"
#include <cstdio>
#include <ctime>

unsigned int PseudoRandomSeed(void)
{
    FILE        *fp = fopen("/dev/urandom", "r");    
    unsigned int s;
    
    if(fp != NULL)
    {
	fread(&s, sizeof(unsigned int), 1, fp);    
	fclose(fp);    	        
    }
    else
	s = (unsigned int) time(NULL);
    
    printf("using seed: %u\n", s);
    
    
    srandom(s);
    
    return s;
}    
    
    
    

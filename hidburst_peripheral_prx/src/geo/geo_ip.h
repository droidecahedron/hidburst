#ifndef _CIRLE_TEST_H
#define _CIRLE_TEST_H 

#include <stdint.h>

#define CORD_SIZE 25

typedef enum
{
  X_POS,
  Y_POS
} circle_test_pos;

/**
Function for generating relative mouse positions drawing a circle.

@param ret_data returns the relative movements. X movement is returned in *(ret_data)
and Y movement is returned in *(ret_data+1). The drawn cicle will consist of 100
points.  
*/
void circle_test_get(int8_t *ret_data);

/**
Function for generating relative mouse positions drawing a square.

@param ret_data returns the relative movements. X movement is returned in *(ret_data)
and Y movement is returned in *(ret_data+1).
*/
void square_test_get(int8_t *ret_data);

#endif

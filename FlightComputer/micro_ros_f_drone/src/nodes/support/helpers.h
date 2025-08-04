#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#define RCCHECK(fn) { 
  rcl_ret_t temp_rc = fn; 
  if((temp_rc != RCL_RET_OK)){
    printf("Error"); // TODO: Add Error Handling
  }
}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

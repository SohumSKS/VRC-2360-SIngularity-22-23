/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Sohum Suthar                                              */
/*    Created:      Oct 10 2021                                               */
/*    Description:  Competition Program 2360S                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
/* -----------------------------------------------------------------------------
    lcm_handle_thread: handles all incoming LCM messages

    Modification History:
    DATE         AUTHOR   COMMENT
    2012-06-01   SS       Created and written
   -------------------------------------------------------------------------- */

#ifndef LCM_HANDLE_THREAD_INC
#define LCM_HANDLE_THREAD_INC


#define LCMHANDLE_SUCCESS 1
#define LCMHANDLE_FAIL    0


/* standard ansi C header files */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <poll.h>

/* posix header files */
#define  POSIX_SOURCE 1
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <map>

#include <lcm/lcm-cpp.hpp>

#include <../../dsvimlib/include/imageTalk.h>
#include <../../dsvimlib/include/msg_util.h>

#include "../../habcam-lcmtypes/image/image/image_t.hpp"
#include "../../habcam-lcmtypes/image/image/image_parameter_t.hpp"

#include "../../dsvimlib/include/imageTalk.h"

typedef struct stereo_event_t
{
   double			  heading;
   double			  pitch;
   double			  roll;
   double			  latitude;
   double			  longitude;
   double			  altitude0;
   double			  altitude1;
   double         fathometer;
   double         conductivity;
   double         temperature;
   double         depth;
   double         salinity;
   double         dO;
   int            cdom;
   int            scatter;
   int            fluor;
   int            therm;

   double            ph1;
   double            ph2;
   int            imageState;


} stereo_event_t;

extern void *lcmHandleThread (void *);

#endif


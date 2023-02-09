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

#include <mesobot-lcmtypes/image/image_t.hpp>
#include <mesobot-lcmtypes/image/image_parameter_t.hpp>
#include <mesobot-lcmtypes/raw/bytes_t.hpp>

extern void *lcmHandleThread (void *);

#endif


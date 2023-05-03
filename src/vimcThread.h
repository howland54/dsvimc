/* ----------------------------------------------------------------------

   code for talkinig to the USB bus for flycapture

   Modification History:
   DATE        AUTHOR   COMMENT
   13-Apr-217  jch      creation, derive from jason crossbow prompt thread
   2018        jch      modify for use with vimba/allied vision cameras

   ---------------------------------------------------------------------- */

#ifndef FLY_THREAD_INC
#define FLY_THREAD_INC



#include <iostream>
#include <sstream>

#include <opencv4/opencv2/opencv.hpp>

#include <VimbaCPP/Include/VimbaCPP.h>
#include "FrameObserver.h"

#include <../../dsvimlib/include/ErrorCodeToMessage.h>

#include "../../habcam-lcmtypes/image/image/image_t.hpp"
#include "../../habcam-lcmtypes/image/image/image_parameter_t.hpp"

using namespace std;

/* ---------------------------------------------------------------------
   fly thread external def
   --------------------------------------------------------------------- */
extern void *vimcThread (void *);

#define CAMERA_TRIGGER_PERIOD 0.2
#define CAMERA_QUERY_PERIOD   0.25
#define NUM_FRAMES            8

#define MAX_CHARACTER_COUNT   4095

typedef enum {
   CAMERA_SHUTTER,
   CAMERA_GAIN,
   MAX_N_OF_PARAMETER_QUERIES
} e_cameraParameterT;




#endif

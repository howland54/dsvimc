/* ----------------------------------------------------------------------


9 April 2025   jch     former rov code used as shell for habcam sensor thread, then jpeg thread
---------------------------------------------------------------------- */
#ifndef JPEG_PROCESS_INC
#define JPEG_PROCESS_INC

#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include <lcm/lcm-cpp.hpp>

typedef struct
{
   cv::Mat  theImage;
   char imageDescription[2048];

   int   year;
   int   month;
   int   day;
   int   hour;
   int   minute;
   int secs;
   int milliseconds;
   //int   trialTenMinute;
   int   cameraNumber;
   int   dayOfYear;
} jpegSave_t;




// ----------------------------------------------------------------------
// DEBUG FLAG:  Uncomment these and recompile to get verbose debugging 
// ----------------------------------------------------------------------
//#define DEBUG_CONSTANCY

// ----------------------------------------------------------------------

extern void *jpegThread (void *);


#endif

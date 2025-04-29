/* ----------------------------------------------------------------------


20 Feb 2023   jch     former rov code used as shell for habcam sensor thread
---------------------------------------------------------------------- */
#ifndef CONSTANCY_PROCESS_INC
#define CONSTANCY_PROCESS_INC

#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include <lcm/lcm-cpp.hpp>

// ----------------------------------------------------------------------
// DEBUG FLAG:  Uncomment these and recompile to get verbose debugging 
// ----------------------------------------------------------------------
//#define DEBUG_CONSTANCY

// ----------------------------------------------------------------------

extern void *constancyThread (void *);
typedef struct {
   float ml;
   float ma;
   float mb;
   bool  validData;
} constancyIlluminationVector_t;

extern constancyIlluminationVector_t theIlluminationVector;

#define DEFAULT_CONSTANCY_COUNT  10

#endif

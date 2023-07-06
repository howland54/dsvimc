/* ----------------------------------------------------------------------


   ---------------------------------------------------------------------- */

#ifndef STEREO_LOGGING_THREAD_INC
#define STEREO_LOGGING_THREAD_INC

#include <time.h>
#include <sys/time.h>
#include <lcm/lcm-cpp.hpp>

using namespace std;


#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xphoto.hpp>
#include "tiff.h"
#include <tiffio.h>

#include "../../dsvimlib/include/IniFile.h"
#include "../../dsvimlib/include/mkdir_p.h"

#include "../../habcam-lcmtypes/image/image/image_t.hpp"
#include "../../habcam-lcmtypes/image/image/image_parameter_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorGPS_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorCtd_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorFathometer_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/marineSensorAltimeter_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorAttitudeSensor_t.hpp"


#define MAX_LOGGING_LENGTH 2048
#define INTERIMAGE_CRITERIA   80000 // microseconds
extern void *stereoLoggingThread (void *);


extern int readIniStereoProcess ();

extern char *flyIniFile;



/* ---------------------------------------------------------------------
   logging thread external def
   --------------------------------------------------------------------- */
extern void *stereoLoggingThread (void *);



#endif

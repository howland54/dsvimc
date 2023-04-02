/* ----------------------------------------------------------------------

   fly main() header

   Modification History:
   DATE 	AUTHOR 	COMMENT
   11-Apr-217  jch      creation

---------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>

#include <VimbaCPP/Include/VimbaSystem.h>

#include "../../dsvimlib/include/imageTalk.h"
#include "../../dsvimlib/include/nio_thread.h"

#ifndef FLY_PROCESS_INC
#define FLY_PROCESS_INC

using AVT::VmbAPI::VimbaSystem;

#define DEFAULT_INI_FILE  "vimc.ini"

#define NULL_EXTRA_ARG_VALUE  -9999


#define MAX_NUMBER_OF_THREADS 256
#define MAX_N_OF_CAMERAS        8
#define MAX_N_OF_LIGHTS         8
#define NO_LIGHT_POSITION       "UNKNOWN_POSITION"

#define NO_SERIAL_NUMBER    "NO_SERIAL_NUMBER"
#define DEFAULT_AUTO_GAIN   1
#define DEFAULT_GAIN        7.5
#define DEFAULT_BINNING     0
#define DEFAULT_AUTO_EXPOSURE 0
#define DEFAULT_EXPOSURE   100.0
#define DEFAULT_CHANNEL_NAME  "NO_CHANNEL_NAME"
#define NO_INITIAL_FILE_NAME  "NO_INITIAL_FILE_NAME"
#define DEFAULT_CAMERA_FREQUENCY 1000000000



#define SYNC_UNKNOWN       5
#define SYNC_SLAVE         6
#define SYNC_LISTENING     7
#define SYNC_UNCALIBRATED  8
#define SYNC_INITIALIZING  9

#define SOFTWARE           10
#define LINE1              11
#define LINE2              12
#define LINE3              13
#define LINE4              14
#define FIXED_RATE         15
#define FREE_RUN           16
#define UNKNOWN_TRIGGER_SOURCE   17


typedef struct
{
   bool     autoGain;
   double   theGain;
   bool     autoShutter;
   double   theShutter;
   bool     binning;
   int      cameraSynced;
   int      triggerSource;
} cameraSettings_t;

typedef struct
{
   char                    *cameraSerialNumber;
   pthread_t               cameraSubThread;
   int                     sendingThread;
   int                     subThread;
   cameraSettings_t        desiredSettings;
   cameraSettings_t        actualSettings;
   char                    *xmlFileName;
   char                    *parameterFileName;
   char                    *lcmChannelName;
   bool                    plugged;
   char                    *subscriptionName;
   char                    *filenamePrefix;
   char                    *saveDirectoryRoot;
   VmbInt64_t              cameraFrequency;
   double                  startCameraTime;
   bool                    saveImages;
   bool                    doNotUseInStereoLogging;
   bool                    saveJPG;
   char                    *jpgPrefix;
   char                    *jpgSaveRoot;

} avtCameraT;




extern int make_thread_table_entry (int thread_num, const char *thread_name, void *(*thread_function) (void *), void *thread_launch_arg, int extra_arg_1, int extra_arg_2, int extra_arg_3, int extra_arg_4);

extern thread_table_entry_t global_thread_table[MAX_NUMBER_OF_THREADS];


extern char *flyIniFile;

#define	NOT_APPLICABLE	0
#endif

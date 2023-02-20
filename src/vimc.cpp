
/* ----------------------------------------------------------------------

    multi-threaded driver for Allied Vision cameras using the Vimba API
    derived from the Jason2 multithreaded architecture by Jonathan Howland
    beginning on 6 april 2017
    originally written for Pt Grey cameas using the flycapture API; hence some of the references to fly
    ---------------------------------------------------------------------- */

/* standard ansi C header files */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>


//or other similar situations
#include <unistd.h>
#include <sys/time.h>
/* posix header files */
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>

/* jason header files */
#include "../../dsvimlib/include/imageTalk.h"
#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/IniFile.h"
#include "../../dsvimlib/include//msg_util.h"
#include "../../dsvimlib/include/global.h"


#include "vimc.h"		/* main() header file */
#include "vimcBusThread.h"
#include "vimcThread.h"
#include "vimcLoggingThread.h"
#include "sensorThread.h"
#include "msThread.h"
#include "msNetThread.h"

#include "lcmHandleThread.h"

#include "stereoLoggingThread.h"

//----------------------------------------------------------------
//  this is the filename which is read as the argument to the main
//  program and stored as a double so everyvody can read it
//----------------------------------------------------------------

char *flyIniFile;




startup_t startup;
avtCameraT  avtCameras[MAX_N_OF_CAMERAS];
VimbaSystem          *vSystem;


int   nOfAvtCameras;


lcm::LCM myLcm("udpm://239.255.76.67:7667?ttl=0");

//lcm::LCM myLcm;

// ----------------------------------------------------------------------
// This is a structure that is initialized in rov.cpp to contain
// default thread attributes for launching threads. 
// It is provided as global variable as a convenience to other threads.
// ----------------------------------------------------------------------
pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

// ----------------------------------------------------------------------
// This is a table of channel pairs that contains all the communication
// channels used in the entire system.  
// Each channel pair connects a single thread to the router.
// the router itself is process 0.
// This array is initialized in rov.cpp
// ----------------------------------------------------------------------


// the global thread table

//thread_table_entry_t global_thread_table[MAX_NUMBER_OF_THREADS];


#define ROV_THREAD_DEFAULT_STACKSIZE 2000000

/* the main program--launch all the necessary threads */

int
main (int argc, char *argv[])
{
   int i;
   int status;


   size_t size;

   nOfAvtCameras = 0;

   fprintf (stderr, "File %s compiled on %s at %s by Jonathan C. Howland\n", __FILE__, __DATE__, __TIME__);

   if(!myLcm.good())
      {
         return 1;
      }

   fprintf (stderr, "lcm initialized good\n");

   
   // set priority higher than default
   status = nice (-10);
   fprintf (stderr, "Setting priority to -10, status = %d (%s) \n", status, (status == 0) ? "SUCCESS" : "FAILED");

   vSystem = &AVT::VmbAPI::VimbaSystem::GetInstance();

   // initialize the thread attributes
   pthread_attr_init (&DEFAULT_ROV_THREAD_ATTR);

   // set for detached threads
   pthread_attr_setdetachstate (&DEFAULT_ROV_THREAD_ATTR, PTHREAD_CREATE_DETACHED),
         // read and set the default stack size
   pthread_attr_getstacksize (&DEFAULT_ROV_THREAD_ATTR, &size), fprintf (stderr, "Default PTHREAD stack size was %ld (0x%08lX)\n", size, size);

   pthread_attr_setstacksize (&DEFAULT_ROV_THREAD_ATTR, ROV_THREAD_DEFAULT_STACKSIZE), pthread_attr_getstacksize (&DEFAULT_ROV_THREAD_ATTR, &size), fprintf (stderr, "New PTHREAD stack size was %ld (0x%08lX)\n", size, size);


   // time to read the startup file
   // first check for an argument

   if (argc > 1)
      {
         flyIniFile = strdup (argv[1]);
         fprintf (stderr, "ini file argument is %s\n", flyIniFile);
      }
   else
      {
         flyIniFile = strdup (DEFAULT_INI_FILE);
         fprintf (stderr, "default ini file: %s\n", flyIniFile);
      }

   // now try to open the file
   IniFile  *iniFile = new IniFile();
   int okINI = iniFile->openIni(flyIniFile);
   bool stereoLogging;
   if(GOOD_INI_FILE_READ == okINI)
      {
         for(int cameraNumber = 0; cameraNumber < MAX_N_OF_CAMERAS; cameraNumber++)
            {
               char cameraLabel[32];
               snprintf(cameraLabel,32,"CAMERA_%d",cameraNumber+1);
               char *thisSN = iniFile->readString(cameraLabel,"SERIAL_NUMBER", NO_SERIAL_NUMBER);
               if(!strcmp(thisSN,NO_SERIAL_NUMBER))
                  {
                     free(thisSN );
                  }
               else
                  {
                     avtCameras[nOfAvtCameras].cameraSerialNumber =  strdup(thisSN);
                     char *scratchPrefix = iniFile->readString(cameraLabel,"FILENAME_PREFIX",cameraLabel);
                     avtCameras[nOfAvtCameras].filenamePrefix = strdup(scratchPrefix);
                     free( scratchPrefix);
                     avtCameras[nOfAvtCameras].saveImages = (bool)iniFile->readInt(cameraLabel,"SAVE_IMAGES",true);
                     scratchPrefix = iniFile->readString(cameraLabel,"SAVE_DIRECTORY_ROOT","./");
                     avtCameras[nOfAvtCameras].saveDirectoryRoot = strdup(scratchPrefix);
                     free(scratchPrefix);
                     avtCameras[nOfAvtCameras].subscriptionName = iniFile->readString(cameraLabel,"SUBSCRIPTION_NAME","DEFAULT_SUBSCRIPTION");

                     bool autoGain = (bool)iniFile->readInt(cameraLabel,"AUTO_GAIN",DEFAULT_AUTO_GAIN);
                     avtCameras[nOfAvtCameras].desiredSettings.autoGain = autoGain;
                     double theGain = iniFile->readInt(cameraLabel,"GAIN",DEFAULT_GAIN);
                     avtCameras[nOfAvtCameras].desiredSettings.theGain = theGain;

                     bool autoExposure = (bool)iniFile->readInt(cameraLabel,"AUTO_EXPOSURE",DEFAULT_AUTO_EXPOSURE);
                     avtCameras[nOfAvtCameras].desiredSettings.autoShutter = autoExposure;
                     double theExposure = iniFile->readInt(cameraLabel,"EXPOSURE",DEFAULT_EXPOSURE);
                     avtCameras[nOfAvtCameras].desiredSettings.theShutter = theExposure;

                     bool binning = (bool)iniFile->readInt(cameraLabel,"BINNING",DEFAULT_BINNING);
                     avtCameras[nOfAvtCameras].desiredSettings.binning = binning;

                     char *scratch = iniFile->readString(cameraLabel,"CHANNEL_NAME",DEFAULT_CHANNEL_NAME);
                     if(!strncmp(scratch,DEFAULT_CHANNEL_NAME,15))
                        {
                           scratch = strdup(cameraLabel);
                        }
                     avtCameras[nOfAvtCameras].lcmChannelName = strdup(scratch);
                     free(scratch);


                     scratch = iniFile->readString(cameraLabel,"CONFIG_FILE_NAME",NO_INITIAL_FILE_NAME);
                     avtCameras[nOfAvtCameras].xmlFileName = strdup(scratch);
                     free(scratch);

                     scratch = iniFile->readString(cameraLabel,"PARAMETER_FILE_NAME",NO_INITIAL_FILE_NAME);
                     avtCameras[nOfAvtCameras].parameterFileName = strdup(scratch);
                     free(scratch);


                     avtCameras[nOfAvtCameras].cameraFrequency = iniFile->readInt(cameraLabel,"CAMERA_FREQUENCY",DEFAULT_CAMERA_FREQUENCY );

                     avtCameras[nOfAvtCameras].plugged = false;

                     nOfAvtCameras++;

                     free(thisSN);
                  }
            }
         stereoLogging = (bool)iniFile->readInt("GENERAL","LOG_STEREO",true);
         iniFile->closeIni();
      }

   else
      {
         fprintf (stderr, "%s ini file does not exist...exiting!\n", flyIniFile);
         return -1;
      }


   msg_queue_initialize();



   // fill the thread table.  first mark all entries so only full entries will be launched

   for (i = 0; i < MAX_NUMBER_OF_THREADS; i++)
      {
         global_thread_table[i].thread_function = NULL;
      }


   // separate thread creation .  First the ones which are always there
   make_thread_table_entry (BUS_THREAD, "BUS_THREAD", busThread, (void *)NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (LCM_RECEIVE_THREAD, "LCM_RECEIVE_THREAD", lcmHandleThread, (void *)NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (LOGGING_THREAD, "LOGGING_THREAD", loggingThread, (void *)NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (SENSOR_THREAD, "SENSOR_THREAD", sensorThread, (void *)NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (CTD_THREAD, "CTD_THREAD", nio_thread, (void *)sensorThread, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (FATHOMETER_THREAD, "FATHOMETER_THREAD", nio_thread, (void *)sensorThread, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (GPS_THREAD, "GPS_THREAD", nio_thread, (void *)sensorThread, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (ALTIMETER_THREAD, "ALTIMETER_THREAD", nio_thread, (void *)sensorThread, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);

   make_thread_table_entry (MS_NET_THREAD,"MS_NET_THREAD",msNetThread, (void *) MS_NET_THREAD, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
   make_thread_table_entry (MS_NIO_THREAD, "MS_NIO_THREAD", nio_thread, (void *) MS_NIO_THREAD, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);

   if(stereoLogging)
       {
         make_thread_table_entry (STEREO_LOGGING_THREAD, "STEREO_LOGGING_THREAD", stereoLoggingThread, (void *)NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
       }

   for(int flyNumber = 0; flyNumber < nOfAvtCameras; flyNumber++)
      {
         char flyThreadName[32];
         snprintf(flyThreadName,31,"vimThread%0d",flyNumber+1);
         long int theThreadNumber = FLY_THREAD_BASE + flyNumber;
         printf(" creating thread %ld %s\n",theThreadNumber,flyThreadName);
         make_thread_table_entry (FLY_THREAD_BASE + flyNumber, flyThreadName, vimcThread, (void *)theThreadNumber, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE, NULL_EXTRA_ARG_VALUE);
      }
   
   // launch all the system threads

   for (i = 0; i < MAX_NUMBER_OF_THREADS; i++)
      {
         if (global_thread_table[i].thread_function != NULL)
            {
               usleep(100000);
               status = pthread_create (&global_thread_table[i].thread, &DEFAULT_ROV_THREAD_ATTR, global_thread_table[i].thread_function, global_thread_table[i].thread_launch_arg);
            }
      }
   // sleep for a bit to give the threads time to start up
   usleep(700000);

   struct tm *tm;
   time_t current_time;

   // call time() and localtime for hour and date
   current_time = time (NULL);
   tm = localtime (&current_time);



   int c;

   do{
         c = getchar();


         if('q' == c)
            {
               printf("\nBYE....\n");
               return 0;
            }
         else if(EOF == c)
            {
               continue;
            }
         else
            {   
               // set priority higher than default
               status = nice (-10);
               fprintf (stderr, "Setting priority to -10, status = %d (%s) \n", status, (status == 0) ? "SUCCESS" : "FAILED");
            }
      } while (c != 'q');

}


/* ----------------------------------------------------------------------

   this function derived from
   JHU ROV and Jason 2 prototype


   Modification History:
   DATE 	AUTHOR 	COMMENT
   21-MAR-2000  LLW     Created and written

---------------------------------------------------------------------- */
int make_thread_table_entry (int thread_num, 
                             const char *thread_name,
                             void *(*thread_function) (void *),
                             void *thread_launch_arg,
                             int extra_arg_1,
                             int extra_arg_2,
                             int extra_arg_3,
                             int extra_arg_4)
{


   global_thread_table[thread_num].thread_num = thread_num;
   global_thread_table[thread_num].thread_name = strdup(thread_name);
   global_thread_table[thread_num].thread_function = thread_function;
   global_thread_table[thread_num].thread_launch_arg = thread_launch_arg;

   if (extra_arg_1 != NULL_EXTRA_ARG_VALUE)
      {
         global_thread_table[thread_num].extra_arg_1 = extra_arg_1;
      }
   if (extra_arg_2 != NULL_EXTRA_ARG_VALUE)
      {
         global_thread_table[thread_num].extra_arg_2 = extra_arg_2;
      }
   if (extra_arg_3 != NULL_EXTRA_ARG_VALUE)
      {
         global_thread_table[thread_num].extra_arg_3 = extra_arg_3;
      }
   if (extra_arg_4 != NULL_EXTRA_ARG_VALUE)
      {
         global_thread_table[thread_num].extra_arg_4 = extra_arg_4;
      }


   return 1;
}







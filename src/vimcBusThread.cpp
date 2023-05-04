/* ----------------------------------------------------------------------

   code for talkinig to the USB bus for flycapture
   
   Modification History:
   DATE        AUTHOR   COMMENT
   13-Apr-2017  jch      creation, derive from jason crossbow prompt thread
          2018  jch      modify to work with vimba cameras on an ethernet
   ---------------------------------------------------------------------- */

/* ansii c headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <unistd.h>
#include <ctime>
#include <iostream>
#include <sstream>


/* local includes */
#include "vimcBusThread.h"
#include "../../dsvimlib/include/time_util.h"

#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageTalk.h"		/* jasontalk protocol and structures */
#include "../../dsvimlib/include/msg_util.h"		/* utility functions for messaging */

#include "vimc.h"


/* posix header files */
#define  POSIX_SOURCE 1


#include <pthread.h>
extern avtCameraT  avtCameras[MAX_N_OF_CAMERAS];
extern int   nOfAvtCameras;
extern VimbaSystem          *vSystem;


pthread_t   busSubThread;
extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

int threadFromSN(int nOfAvtCameras,avtCameraT *theCameras,std::string theSN )
{
   int returnThreadNumber = NO_THREAD_FOUND;
   for(int threadNumber = 0; threadNumber < nOfAvtCameras; threadNumber++ )
      {
         std::string testSerial(theCameras[threadNumber].cameraSerialNumber);
         if(testSerial == theSN)
            {
               returnThreadNumber = FLY_THREAD_BASE + threadNumber;
               break;
            }
      }
   return returnThreadNumber;
}

std::string GetCurrentTimeString()
{
   time_t rawtime;
   struct tm * timeinfo;
   time( &rawtime );
   timeinfo = localtime( &rawtime );

   std::ostringstream formatTime;
   formatTime << (timeinfo->tm_year + 1900) << "-"
      << (timeinfo->tm_mon + 1) << "-"
      << (timeinfo->tm_mday) << " "
      << (timeinfo->tm_hour) << ":"
      << (timeinfo->tm_min) << ":"
      << (timeinfo->tm_sec);

   return formatTime.str();
}









/* ----------------------------------------------------------------------
   bus Thread


   Modification History:
   DATE        AUTHOR   COMMENT
   13-Apr-2017  jch      creation, derive from jason crossbow prompt thread

---------------------------------------------------------------------- */
void *busThread (void *)
{

   msg_hdr_t hdr = { 0 };
   unsigned char data[MSG_DATA_LEN_MAX] = { 0 };

   unsigned int xbs_count = 0;

   // wakeup message
   printf ("BUS_THREAD (thread %d) initialized \n", BUS_THREAD);
   printf ("BUS_THREAD File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);
   // sleep a little so the camera threads can be started up and be waiting for a message if a camera is already on the bus
   usleep(1400000);
   int msg_success = msg_queue_new(BUS_THREAD, "fly bus thread");

   if(msg_success != MSG_OK)
      {
         fprintf(stderr, "vimc bus thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
         fflush (stdout);
         fflush (stderr);
         abort ();
      }
   CameraObserver *camObserver = new CameraObserver();
   AVT::VmbAPI::ICameraListObserverPtr pDeviceObs(camObserver);
   VmbErrorType err = VmbErrorSuccess;
   err = vSystem->Startup();
   if ( VmbErrorSuccess == err )
      {
         std::cout << "Vimba StartUp OK" << std::endl;
         err = vSystem->RegisterCameraListObserver(pDeviceObs);
      }

   // loop forever
   while (1)
      {

         int msg_get_success = msg_get(BUS_THREAD,&hdr, data, MSG_DATA_LEN_MAX);
         if(msg_get_success != MSG_OK)
            {
               fprintf(stderr, "vimc bus thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
            }
         else
            {
               // process new message
               switch (hdr.type)
                  {
                  case CAMA:
                     {
                        std::string theMsg = (char *)data;
                        int theCameraNumber = threadFromSN(nOfAvtCameras,avtCameras, theMsg);
                        printf("camera number is %d\n",theCameraNumber);
                        msg_send(theCameraNumber,BUS_THREAD,CAMA,hdr.length, data );
                        break;
                     }
                  case CAMR:
                     {
                        std::string theMsg = (char *)data;
                        int theCameraNumber = threadFromSN(nOfAvtCameras,avtCameras, theMsg);
                        printf("camera number is %d\n",theCameraNumber);
                        if(theCameraNumber < 0)
                           {
                              //printf("removal:  camera not found in system, SN = %ld",serialNumber);
                           }
                        else
                           {
                              msg_send(theCameraNumber,BUS_THREAD, CAMR,hdr.length, data);
                           }
                        break;
                     }
                  case PNG:
                     {			// recieved a PNG (Ping) message

                        const char *msg = "vimc bus thread is Alive!";
                        // respond with a SPI (Status Ping) message
                        msg_send( hdr.from, BUS_THREAD, SPI, strlen (msg), msg);
                        break;
                     }
                  case BYE:  // received a bye message--time to give up the ghost--
                     {

                        return(NULL);
                        break;
                     }
                  case SPI:
                     {			// recieved a SPI (Status Ping) message
                        break;
                     }
                  default:
                     {			// recieved an unknown message type

                        printf ("fly bus thread: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", BUS_THREAD, hdr.type, hdr.from, hdr.to, hdr.length);
                        break;
                     }

                  }			// switch
            } // end els msg get ok
      }

}

/* ----------------------------------------------------------------------

** spartan Thread
** spartanThread.cpp
**
** Description:
**
**
**
** History:
**       Date       Who      Description
**      ------      ---      --------------------------------------------
**      1/2024      jch      Create from DATA_THREAD.cpp

---------------------------------------------------------------------- */
/* standard ansi C header files */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

/* posix header files */
#define  POSIX_SOURCE 1
#include <pthread.h>

/* jason header files */
/* jason header files */
#include "../../dsvimlib/include/imageTalk.h"
#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/IniFile.h"
#include "../../dsvimlib/include//msg_util.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/launch_timer.h"
#include "sensorThread.h"
#include "spartanThread.h"

extern lcm::LCM myLcm;


/* ----------------------------------------------------------------------

   spartan Thread

   Modification History:
   DATE         AUTHOR  COMMENT
   25 Jan 2024  jch     Created and written.to talk to sparton M2 AHRS
   
   

---------------------------------------------------------------------- */
void *
spartanThread (void *)
{

   gps_t currentGPS;
   double headingOffset;
   char    *attitudeChannelName;

   msg_hdr_t hdr = { 0, 0, 0, 0};
   unsigned char data[MSG_DATA_LEN_MAX] = { 0 };

   // wakeup message
   printf ("SPARTAN thread (thread %d) initialized \n", SPARTON_THREAD);
   printf ("spartan thread File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);

   int msg_success = msg_queue_new(SPARTON_THREAD, "sparton thread");
   if(msg_success != MSG_OK)
      {
         fprintf(stderr, "data thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
         fflush (stdout);
         fflush (stderr);
         abort ();
      }
   IniFile  *iniFile = new IniFile();

   int okINI = iniFile->openIni(flyIniFile);
   double spartanQueryInterval = SPARTAN_DEFAULT_QUERY_INTERVAL;

   if(GOOD_INI_FILE_READ == okINI)
       {

           spartanQueryInterval = iniFile->readDouble( "SPARTON_M2","QUERY_INTERVAL",SPARTAN_DEFAULT_QUERY_INTERVAL);
           if(spartanQueryInterval < 0.0)
               {
                   spartanQueryInterval =  SPARTAN_DEFAULT_QUERY_INTERVAL;
               }
           double defaultLatitude = iniFile->readDouble( "SPARTON_M2","DEFAULT_LATITUDE", SPARTON_DEFAULT_LATITUDE);
           double defaultLongitude = iniFile->readDouble( "SPARTON_M2","DEFAULT_LONGITUDE", SPARTON_DEFAULT_LONGITUDE);
           currentGPS.latitude = defaultLatitude;
           currentGPS.longitude = defaultLongitude;
           headingOffset = iniFile->readDouble( "SPARTON_M2","HEADING_ROTATION",DEFAULT_MICROSTRAIN_HDG_OFFSET);
           if(headingOffset < 0.0)
               {
                   headingOffset += 360.0;
               }
           attitudeChannelName = iniFile->readString("MICROSTRAIN","CHANNEL_NAME","ATTITUDE");




      }
   launched_timer_data_t * spartanQueryTimer = launch_timer_new(spartanQueryInterval, -1, SPARTON_THREAD,  SPQR);
   launched_timer_data_t * spartanWMMTimer = launch_timer_new(WMM_QUERY_INTERVAL, -1, SPARTON_THREAD,  SPMM);
   // loop forever
   while (1)
      {

         // wait forever on the input channel
#ifdef DEBUG_DATA_THREAD
         printf ("DATA_THREAD calling get_message.\n");
#endif

         int msg_get_success = msg_get(SPARTON_THREAD,&hdr, data, MSG_DATA_LEN_MAX);
         if(msg_get_success != MSG_OK)
            {
               fprintf(stderr, "spartan thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
            }
         else{
#ifdef DEBUG_RECIEVED_MESSAGES
               // print on stderr
               printf ("sparton thread got msg type %d from proc %d to proc %d, %d bytes data\n", hdr.type, hdr.from, hdr.to, hdr.length);
#endif

               // process new message
               switch (hdr.type)
                  {
                  case SPQR:
                     { // time to send a prompt
                        char  promptString[256];
                        int len = snprintf(promptString,255,"$xxXDR\r\n");
                        msg_send(SPARTON_NIO_THREAD,SPARTON_THREAD, WAS, len,promptString );
                        break;
                     }
                  case SPMM:  /* time to set the position for a variation calculation */
                     {
                        char  promptString[256];
                        time_t current_time;
                        struct tm *tm;
                        current_time = time (NULL);
                        tm = localtime (&current_time);
                        double decimalYear = 1900.0 + tm->tm_year + tm->tm_yday/365.0;

                        int len = snprintf(promptString, 255, "$PSPA,AUTOVAR,%.2f,%.2f,0.0,%.1f\r\n",currentGPS.latitude, currentGPS.longitude,decimalYear);
                        msg_send(SPARTON_NIO_THREAD,SPARTON_THREAD, WAS, len,promptString );
                        break;
                     }
                  case WPOS: // got a position message
                     {
                        gps_t *theGPS = (gps_t  *) data;
                        currentGPS.latitude = theGPS->latitude;
                        currentGPS.longitude = theGPS->longitude;
                        break;
                     }
                  case SAS:
                     {
                        // parse the incoming string
                        char dateString[256];
                        char loggingRecord[2048];
                        int logLen = 0;
                        rov_time_t sensorTime = rov_get_time();
                        rov_sprintf_dsl_time_string(dateString);
                        unsigned short	computed_checksum = compute_nmea_checksum((char *)data);
                        char *checksum_place = strstr((char *)data,"*");
                        if(checksum_place)
                           {
                              unsigned int read_checksum;
                              int items = sscanf(checksum_place+1,"%02x",&read_checksum);

                              if(items)
                                 {

                                    if(read_checksum == computed_checksum)
                                       {
                                          *checksum_place = '\0'; // nul terminat to get rid of the checksum
                                          if(!strncmp((char *)data,"$PSPA",5))
                                             {

                                                double autoVar;
                                                int autovarItems = sscanf( (char *)data,"$PSPA,AutoVar=%lf",&autoVar);
                                                if(autovarItems)
                                                   {

                                                   }
                                                else
                                                   {

                                                   }

                                             }
                                          else if(!strncmp((char *)data, "$HCXDR",6))
                                             {
                                                double magHeading, trueHeading, pitch, roll, spartonTemp, magError;
                                                marine_sensor::MarineSensorAttitudeSensor_t  myAttitude;

                                                int HCXitems = sscanf( (char *)data,"$HCXDR,A,%lf,D,A,%lf,D,A,%lf,D,A,%lf,D,C,%lf,C,G,%lf",&magHeading,&trueHeading, &pitch, &roll, &spartonTemp, &magError);
                                                if(6 == HCXitems)
                                                   {
                                                      double rawHeading = trueHeading;
                                                      double cookedHeading = rawHeading - headingOffset;
                                                      double cookedPitch, cookedRoll;
                                                      if(cookedHeading < 0.0)
                                                          {
                                                              cookedHeading += 360.0;
                                                          }
                                                      if(headingOffset < 10.0)
                                                          {
                                                              cookedPitch =  pitch ;
                                                              cookedRoll = roll;
                                                          }
                                                      else if(headingOffset  > 350.0)
                                                          {
                                                              cookedPitch =  pitch ;
                                                              cookedRoll = roll;
                                                          }
                                                      else if((headingOffset > 80.0) && (headingOffset <100.0))
                                                          {
                                                              cookedPitch =  -roll ;
                                                              cookedRoll = pitch;
                                                          }
                                                      else if((headingOffset > 170.0) && (headingOffset <190.0))
                                                          {
                                                              cookedPitch =  -pitch ;
                                                              cookedRoll = -roll;
                                                          }
                                                      else if((headingOffset > 260.0) && (headingOffset <280.0))
                                                          {
                                                              cookedPitch =  roll ;
                                                              cookedRoll = -pitch;
                                                          }
                                                      myAttitude.heading = cookedHeading;
                                                      myAttitude.pitch = cookedPitch;
                                                      myAttitude.roll = cookedRoll;
                                                      myAttitude.utime = sensorTime;
                                                      int success = myLcm.publish(attitudeChannelName,&myAttitude);
                                                      int logLen = snprintf(loggingRecord,2047,"ATT %s HABCAM %0.2f %0.2f %0.2f", dateString, cookedHeading, cookedPitch, cookedRoll);
                                                      if(logLen)
                                                          {
                                                              msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                                                          }

                                                   }
                                                else
                                                   {

                                                   }
                                             }
                                       }
                                 }
                           }

                        break;
                     }

                  case PNG:		// recieved a PNG (Ping) message
                     {
                        const char *msg = "SPARTON_THREAD is Alive!";

                        // respond with a SPI (Status Ping) message
                        msg_send( hdr.from, hdr.to, SPI, strlen (msg), msg);

                        break;
                     }
                  case SPI:		// recieved a SPI (Status Ping) message
                     break;

                  case BYE:
                     {
                        launch_timer_disable(spartanQueryTimer);
                        launch_timer_disable(spartanWMMTimer);
                        return(NULL);

                        break;
                     }

                  default:		/* recieved an unknown message type */
                     {
                        printf ("SPARTON_THREAD: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", SPARTON_THREAD, hdr.type, hdr.from, hdr.to, hdr.length);
                        break;
                     }

                  }			// switch
            } // end else msg_get OK

      }


}


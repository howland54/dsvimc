/* ----------------------------------------------------------------------

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
//#define DEBUG_LOGGING

/* local includes */
#include "../../dsvimlib/include/time_util.h"

#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageTalk.h"		/* jasontalk protocol and structures */
#include "../../dsvimlib/include/msg_util.h"		/* utility functions for messaging */

#include "vimcLoggingThread.h"

/* posix header files */
#define  POSIX_SOURCE 1


#include <pthread.h>


extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

FILE     *logAsciiFilePointer;
char     asciiLogFileName[256];
char     *logging_directory;



/* ----------------------------------------------------------------------
   logging Thread


   Modification History:
   DATE        AUTHOR   COMMENT
   19-July-2018  jch      creation, derive from vimba bus thread

---------------------------------------------------------------------- */
void *loggingThread (void *)
{

   msg_hdr_t hdr = { 0 };
   unsigned char data[MSG_DATA_LEN_MAX] = { 0 };
   FILE *iniFd;

   logging_t   myLog;


   // wakeup message
   printf ("LOGGING_THREAD (thread %d) initialized \n", LOGGING_THREAD);
   printf ("LOGGING_THREAD File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);
   // sleep a little so the camera threads can be started up and be waiting for a message if a camera is already on the bus
   usleep(700000);
   int msg_success = msg_queue_new(LOGGING_THREAD, "logging thread");

   if(msg_success != MSG_OK)
      {
         fprintf(stderr, "logging: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
         fflush (stdout);
         fflush (stderr);
         abort ();
      }

   iniFd = fopen (flyIniFile, "r");
   if (!iniFd)
      {
         printf ("%s ini file does not exist...exiting!\n", flyIniFile);
         fflush (stdout);
         fflush (stderr);
         abort ();
      }
   int success = readIniLoggingProcess (&myLog);

   // loop forever
   while (1)
      {

         int msg_get_success = msg_get(LOGGING_THREAD,&hdr, data, MSG_DATA_LEN_MAX);
         if(msg_get_success != MSG_OK)
            {
               fprintf(stderr, "logging thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
            }
         else
            {
               // process new message
               switch (hdr.type)
                  {

                  case PNG:
                     {			// recieved a PNG (Ping) message

                        const char *msg = "logging thread is Alive!";

                        // respond with a SPI (Status Ping) message
                        msg_send( hdr.from, BUS_THREAD, SPI, strlen (msg), msg);
                        break;
                     }
                  case BYE:  // received a bye message--time to give up the ghost--
                     {

                        return(NULL);
                        break;
                     }
                  case LOG: // should log this to the file
                     {
                        data[hdr.length] = '\0';
                        logThisNow(&myLog,(char *)data);
                        break;
                     }
                  case SPI:
                     {			// recieved a SPI (Status Ping) message
                        break;
                     }
                  default:
                     {			// recieved an unknown message type

                        printf ("logging thread: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", BUS_THREAD, hdr.type, hdr.from, hdr.to, hdr.length);
                        break;
                     }

                  }			// switch
            } // end eles msg get ok
      }

}

int
readIniLoggingProcess(logging_t *theLog)
{
   // each section is demarked by a a string which is contained
   // in the global table
   IniFile  *iniFile = new IniFile();
   int okINI = iniFile->openIni(flyIniFile);
   if(GOOD_INI_FILE_READ == okINI)

   theLog->logging_directory = iniFile->readString ( "GENERAL", "LOGGING_DIRECTORY", "/data");
}

static int logOpenAsciiLog_file (logging_t * log)
/* ----------------------------------------------------------------------
     Maintains the binary log file pointer. Opens a new file each hour.

     MODIFICATION HISTORY
     DATE         WHO             WHAT
     -----------  --------------  ----------------------------
     18 Apr 1999  Louis Whitcomb  Created and Written based on Dana's original write_dvl
     15 Oct 2001  Jon Howland     modified to work with data structure, not globals
     2008-06-24    mvj            Changed timestamp to use rov_get_time instead of
                                  direct call to OS.  Useful for simulations.
     2009-03-09    mvj            Change to use realtime for log file names
                                  in fasttime.
     ---------------------------------------------------------------------- */
{

   // for date and hours
   static char filename[512];
   int status = 0;

   // Get (real)time.  2009-Mar-09  mvj  Create realtime log names regardless of time mode.
   // for date and hours
   struct tm *tm;
   time_t current_time;


   // fflush the data file
   //   if(log.ascii_log_file_pointer != NULL)
   //     fflush(log.ascii_log_file_pointer);

   // call time() and localtime for time
   current_time = time (NULL);
   tm = localtime (&current_time);
 #ifdef DEBUG_LOGGING
   printf ("in open log file, last hour = %d\n", log->last_hour);
   printf ("current hour = %d\n", tm->tm_hour);
   printf ("ascii file pointer = %x\n", (long int) log->logAsciiFilePointer);
 #endif
   /* if ascii_log_flag is true */
   /* open a new file on first call, and at the top of the hour thereafter */
   if (((tm->tm_hour != log->last_hour) || (log->logAsciiFilePointer == NULL)))
     {
       log->last_hour = tm->tm_hour;

         /* close existing log file */
         if (log->logAsciiFilePointer != NULL)
            {
               if (0 == fclose (log->logAsciiFilePointer))
                  printf ("LOGGING_THREAD: Closed log file %s OK\n", asciiLogFileName);
               else
                  printf ("LOGGING_THREAD: ERROR closing log file %s\n", asciiLogFileName);
            }
         /* create the new log file name */
         char suffix[32];

         snprintf(suffix,32,"VIMDAT");
         snprintf(filename,511,"%s/%04d%02d%02d_%02d%02d.%s",log->logging_directory,tm->tm_year + 1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min,suffix);

         /* open the new file */
#ifdef DEBUG_LOGGING
         printf ("LOGGING_THREAD: in open log file, opening a new file %s\n", filename);
#endif
         log->logAsciiFilePointer = fopen(filename, "wa");

         //logAsciiFilePointer = fopen("/tmp/foo", "wa");
         /* check results of fopen operation */
         if (log->logAsciiFilePointer == NULL)
            {
               printf ("LOGGING_THREAD: error opening %s\n", filename);
               status = 1;
            }
         else
            {
#ifdef DEBUG_LOGGING
               printf ("LOGGING_THREAD: log->logAsciiFilePointer was false\n");
               printf ("ascii file pointer = %x\n", (long int) log->logAsciiFilePointer);
#endif
               strcpy (asciiLogFileName, filename);
               printf ("LOGGING_THREAD: Opened log file %s OK\n", asciiLogFileName);
            }
      }
   return (status);
}
int logThisNow (logging_t * log, char *record_data)
/*


     MODIFICATION HISTORY
     DATE         WHO             WHAT
     -----------  --------------  ----------------------------
     18 Apr 1999  Louis Whitcomb  Created and Written based on Dana's original write_dvl
     15 OCT 2001  Jon Howland     hack to use with rov_54.1, remove timestamping
     19 Jul 2018  Jon Howland     hack to use in dsvim

     ---------------------------------------------------------------------- */
{


#ifdef DEBUG_LOGGING
   printf ("LOGGING_THREAD:  just entered log this now!\n");
#endif

   // open a log file if required
   //printf(" the logging record recieved: %s  %s\n",record_type,record_data);
   logOpenAsciiLog_file (log);


   if (log->logAsciiFilePointer != NULL)
      {
         fprintf(log->logAsciiFilePointer, "%s\n", record_data);
      }

#ifdef DEBUG_LOGGING
   printf ("LOGGING_THREAD:  just left log this now!\n");
#endif

   return 0;

}

/* ----------------------------------------------------------------------

Ms net thread

Modification History:
DATE         AUTHOR   COMMENT
26-OCT-2006  LLW      Created and written.
25 May 2017 jch modify from ms thread

---------------------------------------------------------------------- */
/* standard ansi C header files */
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* posix header files */
#define  POSIX_SOURCE 1

#include <pthread.h>
#include <termios.h>
#include <unistd.h>

/* jason header files */
#include "../../dsvimlib/include/imageTalk.h"

#include "../../dsvimlib/include//msg_util.h"
#include "../../dsvimlib/include//launch_timer.h"
#include "../../dsvimlib/include//time_util.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageA2b.h"
#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/IniFile.h"
#include "msNetThread.h"	/* this thread */
#include "microstrain.h"

/* changed 4/2/09 by jch, put in prompt to sample microstrain, it will only run at max rate.
   this is way too fast, so sample every (for now) .2 seconds
   should make this an ini file  entry*/

extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

micro_strain_t 	nms = {0};
#define START_OF_DATA_CHAR 0x31
#define MS_MESSAGE_LENGTH 23


/* ----------------------------------------------------------------------

ms net Thread

Modification History:
DATE         AUTHOR   COMMENT
26-OCT-2006  LLW      Created and written.

---------------------------------------------------------------------- */
void * msNetThread (void *thread_num)
{

   // declare local variables for message handling
   msg_hdr_t hdr = { 0, 0, 0, 0};
   unsigned char data[MSG_DATA_LEN_MAX+1] = { 0 };
   FILE *ini_fd;
   int ms_parse_sync = 0;

   // declare working local variables
   int my_thread_num;
   int msg_success;
   //launched_timer_data_t * timer_stuff;

   /* get my thread number */
   my_thread_num = (long int) thread_num;

   launched_timer_data_t * sync_timer = launch_timer_new(MICROSTRAIN_POLL_INTERVAL, -1, MS_NET_THREAD, MSPI);

   // ------------------------------------------------------------
   // wakeup message
   // ------------------------------------------------------------
   printf ("MS_NET_THREAD (thread %d, %s) initialized \n", MS_NET_THREAD, IMAGETALK_MESSAGE_NAME(my_thread_num));
   printf ("MS_NET_THREAD File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);

   // ------------------------------------------------------------
   // initialize a message queue for me
   // ------------------------------------------------------------
   msg_success = msg_queue_new(my_thread_num, "ms_thread");
   char *device;
   if(msg_success != MSG_OK)
      {
         printf ("MS THREAD: Could not initialize queue, error is %s\n",
                        MSG_ERROR_CODE_NAME(msg_success));
         fflush (stdout);
         abort ();
      }


   /* Launch a timer to send me messages at specified interval */
   /*  timer_stuff = launch_timer_new( 1.0,             // send a message at this dt
                 -1,              // this many times (-1 = infinite)
                 my_thread_num,      // send the messages to this thread (me)
                 TIMER_UPDATE_RESPONSE);  // send this type of message (header only, no data)
  */

   IniFile  *iniFile = new IniFile();
   int okINI = iniFile->openIni(flyIniFile);
   if(GOOD_INI_FILE_READ == okINI)
      {
        // device = iniFile->readString("MS_THREAD", "DEVICE", "null");
         iniFile->closeIni();
      }
   else
       {
           printf ("%s ini file does not exist...microstrain thread exiting!\n", flyIniFile);
           fflush (stdout);
           fflush (stderr);
           abort ();

       }

   // ------------------------------------------------------------
   // loop forever
   // ------------------------------------------------------------
   while (1)
      {

         // #define DEBUG_MS 1

         /* wait forever on the input channel */
         msg_get(my_thread_num, &hdr, data, MSG_DATA_LEN_MAX);

         // ------------------------------------------------------------
         /* process new message */
         // ------------------------------------------------------------
         switch (hdr.type)
            {

            /* ----------------------------------------
        received PNG (Ping) message
        ---------------------------------------- */
            case PNG:
               {
                  const char *msg = "MS_NET_THREAD is Alive!";


                  /* respond with a SPI (Status Ping) message */
                  // be careful emulating this - this sends a message BACK
                  // to the originator of the message I just received
                  msg_send(hdr.from, hdr.to, SPI, strlen (msg), msg);

                  // print newsy message
                  printf ("MS_NET_THREAD: (thread %d) got msg type %s (%d)  from proc %s (%d) to proc %s (%d), %d bytes data\n",
                                 MS_NET_THREAD,
                                 IMAGETALK_MESSAGE_NAME(hdr.type),
                                 hdr.type,
                                 IMAGETALK_THREAD_NAME(hdr.from),
                                 hdr.from,
                                 IMAGETALK_THREAD_NAME(hdr.to),
                                 hdr.to,
                                 hdr.length);


                  break;
               }
            case SAS:
               {
                  char msBuffer[1024];
                  if(MS_NIO_THREAD == hdr.from)
                     {
                        //printf(" received %d bytes\n",hdr.length);
                        if(hdr.length < 1024)
                           {
                              // look for the sync character
                              for(int cn = 0; cn <  hdr.length; cn++)
                                 {
                                    if(0 == ms_parse_sync)
                                       {
                                          if( data[cn] ==  START_OF_DATA_CHAR)
                                             {
                                                msBuffer[0] =  data[cn];
                                                //printf(" got a sync byte %d\n",cn);
                                                ms_parse_sync++;
                                             }
                                       }
                                    else
                                       {
                                          msBuffer[ms_parse_sync] = data[cn];
                                          if(ms_parse_sync == MS_MESSAGE_LENGTH)
                                             {
                                                int retcode = microstrain_parse_binary_string(&nms, msBuffer, MS_MESSAGE_LENGTH );
                                                //printf(" bytes %d char %x parsed, retcode = %d\n",cn,msBuffer[0],retcode);
                                                if(MICROSTRAIN_STRING_OK == retcode)
                                                   {
                                                      ms_parse_sync = 0;
                                                      msg_send(SENSOR_THREAD,
                                                               MS_NET_THREAD,
                                                               MSD,
                                                               sizeof(nms),
                                                               &nms);

                                                   }
                                                ms_parse_sync = 0;
                                                continue;

                                             }
                                          ms_parse_sync++;
                                       }
                                 }
                           }
                     }
                  break;
               }
            case MSPI:
               {
                  //printf(" sending a prompt\n");
                  unsigned char data = GYRO_STAB_EULER_W_INST_VECT;
                  // write the bytes
                  //write (sio.sio_port_fid, &data, 1);
                  msg_send(MS_NIO_THREAD,MS_NET_THREAD, WAS,1,&data );

                  break;
               }


               /* ----------------------------------------
        received SPI (Status Ping) message
        ---------------------------------------- */
            case SPI:
               break;

               /* ----------------------------------------
        received an unknown message type
        ---------------------------------------- */
            default:
               {
                  printf ("MS_NET_THREAD: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE:\n"
                                 "MS_NET_THREAD: got msg type %s (%d)  from proc %s (%d) to proc %s (%d), %d bytes data\n",
                                 MS_NET_THREAD,
                                 IMAGETALK_MESSAGE_NAME(hdr.type),
                                 hdr.type,
                                 IMAGETALK_THREAD_NAME(hdr.from),
                                 hdr.from,
                                 IMAGETALK_THREAD_NAME(hdr.to),
                                 hdr.to,
                                 hdr.length);
                  break;
               }

            }			// switch


      }


}

/* ----------------------------------------------------------------------

Sensor thread for ROV control system

Modification History:
DATE         AUTHOR COMMENT
23-JUL-2000  LLW    Created and written.
01-DEC-00    LLW    Added DEPth and KVH jasontalk strings
Bug fixes to WRV, PNS, and PAS  
02-DEC-00   	LLW&DAS  Addekvhd revised PNS, PAS sensor input strings with 
02-DEC-00   	LLW&DAS  Added new DEP and KVH sensor input strings
02-DEC-00   	LLW&DAS  Timestamp all incoming PNS, PAS, KVH, DEP, WJS sensor data, store previous values
09 MAY 2001 LLW&DAS Fixed heading velocity wrap bug in gyro and mag
14 May 2001  DAS Modified DEP string parsing to accept DEP string using Paro data
15 May 2001  DAS Modified PNS string to include shaarps hdg from nav program in dbl[25]
16 May 2001  DAS Only update nav_last with current nav data if it was a good solution 
this is based on what nav mode you are in and the number of reported good ranges
to the aft (ROV0) ducer
1  November 2001  JCH   began adding messages for J2
1  november 2001  JCH       paroscientific added 
6  December 2001  JCH   added honeywell maggie sbe37 

17 December 2001  JCH   added altimeter
18 December 2001  JCH   added husk for medea PAS (place holder)
31 December 2001  JCH   put in WT1, WT0 messages
13 JAN 2002  LLW&DAS  Added Octans NMEA parsing
18 Jan 2002  LLW     Added mutex status flags to sensor.cpp, control.cpp, rov.cpp 
to help debug possible mutex problem
31 Jan 2002    JCH  did merge with Louis' new code (see 13,18 Jan)
15 Feb 2002   JCH   mrg in new msging utilities
18 Feb 2002   JCH  fix logging--most notable, log doppler via my calls vice LLW's
24 Feb 2002    JCH  put in medea parse and MPS logging string
11 March 2002 JCH  put in  logging calls
13 Mar 2002   JCH  chnages for proper use of magnetic declination
04 Mar 2002 LLW temporarily borrow paro port to test altimeter
04 MAR 02 LLW   Commisioned altimeter in new controller, added error and temp to altimeter data struct. 
12 May 2002 jch  take out timer update from system timer, start my own
15 may 2002  jch  put in dvlnav as source
24 May 2002  jch  put in ini file reading
02 JUL 2002  LLW  Fixed format bug in depth logging.
07 JUL 02    LLW     Removed filtered depth from depth sensor data struc
09 July 2002  JCH fixed xbow
18 JUL 2002  LLW  Changed DVL to DVX string from dvlnav
14 Nov 2002 jch   change code so as to allow data from display nav computer
12 OCT 2003 LLW   Fixed seg fault on program termination
13 OCT 2003 LLW   replaced scary code from parse_parosci with sscanf
18 OCT 2003 LLW Added KVH ADGC gyro used on JHU ROV
13 MAY 2004 LLW Cleaned up TCM2 parsing code and assignment in sensor_thread.cpp
16 Sep 2004 jch  have to clean up altimeter.  need to make both Jason and medea respond correctly
                  to either a benthos or a simrad altimeter.  NOT DONE YET

13 Oct 2004 LLW Enabled DVZ processing for Auto-Altitude
13 OCT 2004  LLW      Created dvlnav_data and dvlnav_data_last in sensor_t.
                      Retired dvlnav_state and dvlnav_state_last from sensor_t,
                      the dvlnav state data is now in dvlnav_data.state and
                      dvlnav_data_last.state
18 Nov 2006   jch    allowed incoming MED data instead of just FSH
20 Feb 2023   jch     used as shell for habcam sensor thread

---------------------------------------------------------------------- */
/* standard ansi C header files */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* posix header files */
#define  POSIX_SOURCE 1
#include <pthread.h>

#include <string.h>

/* jason header files */
#include "../../dsvimlib/include/imageTalk.h"

#include "../../dsvimlib/include//msg_util.h"
#include "../../dsvimlib/include//launch_timer.h"
#include "../../dsvimlib/include//time_util.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageA2b.h"
#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/IniFile.h"



#include "sensorThread.h"	/* sensor thread */
extern thread_table_entry_t global_thread_table[MAX_NUMBER_OF_THREADS];


// ----------------------------------------------------------------------
// sensor data structure
// ----------------------------------------------------------------------
static sensor_t sensor = { PTHREAD_MUTEX_INITIALIZER, 0 };

// void *global_sensor = (void *) &sensor;

launched_timer_data_t * update_timer = NULL;



/* ----------------------------------------------------------------------
   Function to get sensor data

   ---------------------------------------------------------------------- */
void sensor_thread_get_data (sensor_t * data)
{

   // get a copy of the sensor data structure
   pthread_mutex_lock(&sensor.mutex);

   *data = sensor;

   pthread_mutex_unlock(&sensor.mutex);

}



/* =====================================================================
   The network message processing fuction
   ====================================================================== */
static void
process_net_msg (sensor_t * sensor, msg_hdr_t * in_hdr, char *in_data)
{

   sensor->msgs_received++;
   rov_time_t time_now = rov_get_time();
   switch (in_hdr->type)
      {

      case PNG:			/* respond to ping request */
         {


            // respond with a SPI (Status Ping) message
            msg_send(  in_hdr->from, in_hdr->to, SPI, 0,NULL);



            break;
         }

      case SPI:			// recieved a SPI (Status Ping) message
         break;



      case BYE:  // received a bye message--time to give up the ghost--
         {

            if( update_timer != NULL)
               launch_timer_disable(update_timer);
            return;


            break;
         }
      default:			// recieved an unknown message type
         {
            printf ("SENSOR_THREAD: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", SENSOR_THREAD, in_hdr->type, in_hdr->from, in_hdr->to, in_hdr->length);
            break;
         }

      }


}



/* ----------------------------------------------------------------------

Sensor Thread

Modification History:
DATE         AUTHOR  COMMENT
23-JUL-2000  LLW      Created and written.
18 Jan 2002  LLW     Added mutex status flags to sensor.cpp, control.cpp, rov.cpp 
to help debug possible mutex problem
18 Jan 2002  LLW     Added mutex status flags to sensor.cpp, control.cpp, rov.cpp 
to help debug possible mutex problem

---------------------------------------------------------------------- */


void *sensorThread (void *)
{

   //  sensor_t sensor = { 0 };
   msg_hdr_t in_hdr = { 0 };
   unsigned char in_data[MSG_DATA_LEN_MAX] = { 0 };

   int msg_success = msg_queue_new(SENSOR_THREAD, "sensor thread");
   if(msg_success != MSG_OK)
      {
         fprintf(stderr, "sensor thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
         fflush (stdout);
         fflush (stderr);
         abort ();
      }

   // wakeup message
   printf ("SENSOR (thread %d) initialized \n", SENSOR_THREAD);
   printf ("SENSOR File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);
   IniFile  *iniFile = new IniFile();
   int okINI = iniFile->openIni(flyIniFile);
   bool stereoLogging;
   if(GOOD_INI_FILE_READ == okINI)
       {
           double rollOffset, pitchOffset, headingOffset;
           rollOffset =    iniFile->readDouble("MICROSTRAIN","ROLL", DEFAULT_MICROSTRAIN_ROLL_OFFSET);
           pitchOffset =   iniFile->readDouble( "MICROSTRAIN","PITCH",DEFAULT_MICROSTRAIN_PITCH_OFFSET);
           headingOffset = iniFile->readDouble( "MICROSTRAIN","HEADING",DEFAULT_MICROSTRAIN_HDG_OFFSET);
           iniFile->closeIni();
       }
   else
       {
           printf ("%s ini file does not exist...sensorThread exiting!\n", flyIniFile);
           fflush (stdout);
           fflush (stderr);
           abort ();

       }


   // loop forever
   while (1)
      {

         // wait forever on the input channel
#ifdef DEBUG_SENSOR
         printf ("SENSOR calling get_message.\n");
#endif

         int msg_get_success = msg_get(SENSOR_THREAD,&in_hdr, in_data, MSG_DATA_LEN_MAX);
         if(msg_get_success != MSG_OK)
            {
               fprintf(stderr, "sensor thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
            }
         else{
#ifdef DEBUG_SENSOR
               // print on stderr
               printf ("SENSOR got msg type %d from proc %d to proc %d, %d bytes data\n", in_hdr.type, in_hdr.from, in_hdr.to, in_hdr.length);
#endif

               // lock the mutex
               pthread_mutex_lock (&sensor.mutex);

               // process the message
               process_net_msg (&sensor, &in_hdr, (char *) in_data);

               // unlock the mutex
               pthread_mutex_unlock (&sensor.mutex);


            }// end else
      }


}


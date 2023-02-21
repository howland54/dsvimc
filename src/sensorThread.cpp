/* ----------------------------------------------------------------------


20 Feb 2023   jch    former rov control code used as shell for habcam sensor thread

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
        case SAS:
            {
                switch (in_hdr->from)
                    {
                    case CTD_THREAD:
                        {
                            double   c,t,p,s,ss;
                            int items = sscanf( in_data, "%lf,%lf,%lf,%lf,%lf",&t,&c,&p,&s,&ss);
                            if(5 == items)
                                {
                                    sensor->ctd.conductivity = c;
                                    sensor->ctd.temperature = t;
                                    sensor->ctd.pressure = p;
                                    sensor->ctd.depth = p*PRESSURE_TO_DEPTH;
                                    sensor->ctd.salinity = s;
                                    sensor->ctd.sound_velocity= ss;
                                }
                            else if(4 == items)
                                {
                                    sensor->ctd.conductivity = c;
                                    sensor->ctd.temperature = t;
                                    sensor->ctd.pressure = p;
                                    sensor->ctd.depth = p*PRESSURE_TO_DEPTH;
                                    sensor->ctd.salinity = s;
                                    sensor->ctd.sound_velocity = UNKNOWN_SOUND_SPEED;
                                }
                            else if(3 == items)
                                { // this is a seabird CTD
                                    sensor->ctd.conductivity = c;
                                    sensor->ctd.temperature = t;
                                    sensor->ctd.pressure = p;
                                    sensor->ctd.depth = p*PRESSURE_TO_DEPTH;
                                    sensor->ctd.sound_velocity = UNKNOWN_SOUND_SPEED;
                                    sensor->ctd.salinity = UNKNOWN_SALINITY;
                                }
                            break;
                        }
                    case FATHOMETER_THREAD:
                    case GPS_THREAD:
                        {
                            char  command[MAX_MESSAGE_LENGTH];
                            int items = sscanf(in_data,"%s", command);
                            if(!items)
                                {
                                    break;
                                }
                            if(!strncmp(command, "$GPGGA",6))
                                {
                                    int ignoreGPSChecksum = FALSE;
                                    gpgga_t  gpg = parse_gpgga(command,ignoreGPSChecksum);
                                    if(gpg.valid)
                                        {
                                            sensor->vesselPosition.latitude = RAD_TO_DEGREES(gpg.latitude);
                                            sensor->vesselPosition.longitude = RAD_TO_DEGREES(gpg.longitude);
                                        }

                                }
                            else if(!strncmp(command, "$LCGGA",6)) // this must be the Kathy Marie
                                {
                                    gpgga_t  gpg = parse_lcgga(command);
                                    if(gpg.valid)
                                        {
                                            sensor->vesselPosition.latitude = RAD_TO_DEGREES(gpg.latitude);
                                            sensor->vesselPosition.longitude = RAD_TO_DEGREES(gpg.longitude);
                                        }

                                }
                            else if(!strncmp(command, "$GPVTG",6))  // course, speed
                                {
                                    double   inTrueCOG;
                                    double   inMagCOG;
                                    double   inSogKnots;
                                    double   sogKm;
                                    int items = sscanf(in_data,"$GPVTG,%lf,T,%lf,M,%lf,N,%lf,K",&inTrueCOG,&inMagCOG,&inSogKnots,&sogKm);
                                    if(4 == items)
                                        {
                                        }
                                    else
                                        {
                                            items = sscanf(in_data,"$GPVTG,,T,%lf,M,%lf,N,%lf,K",&inMagCOG,&inSogKnots,&sogKm);
                                            if(3 == items)
                                                {
                                                }
                                        }

                                }

                            else if (!strncmp(command, "$SDDBT",6))
                                {
                                    double   myWaterDepth = 0.0;
                                    int items = sscanf( in_data, "$SDDBT,%lf,f,",&myWaterDepth);
                                    if(items)
                                        {
                                            sensor->fathometer.pos = myWaterDepth * sensor->fathometerMultiplier;
                                        }
                                }

                            break;
                        }
                    case ALTIMETER_THREAD:
                        {
                            double	altimeterTemp,range;
                            int items = sscanf( in_data, "T%lf R%lf",&altimeterTemp,&range);
                            if(2 == items)
                                {
                                    sensor->altimeter.temp =  altimeterTemp;
                                    sensor->altimeter.pos = range;
                                }
                            else
                                {
                                    items = sscanf( in_data, "R%lf",&range);
                                    if(items)
                                        {
                                            sensor->altimeter.pos = range;
                                        }
                                }
                            break;
                        }
                    default:
                        {
                            break;
                        }
                    }
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

    sensor_t sensor = { 0 };
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


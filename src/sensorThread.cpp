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
#include "microstrain.h"


char    *altimeterChannelName;
char    *ctdChannelName;
char    *gpsChannelName;
char    *fathometerChannelName;
char    *attitudeChannelName;

double  headingOffset;
double fathometerMultiplier;
double fathometerBias;

#include "sensorThread.h"	/* sensor thread */

extern lcm::LCM myLcm;

// ----------------------------------------------------------------------
// sensor data structure
// ----------------------------------------------------------------------
static sensor_t sensor = { 0 };

launched_timer_data_t * update_timer = NULL;


/* =====================================================================
   The network message processing fuction
   ====================================================================== */
static void
process_net_msg (sensor_t * sensor, msg_hdr_t * in_hdr, char *in_data)
{

    char loggingRecord[2048];
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
        case MSD:  // received a complete microstrain string in the ms net thread
            {
                char dateString[256];
                micro_strain_t *theMicrostrain = (micro_strain_t  *) in_data;
                marine_sensor::MarineSensorAttitudeSensor_t  myAttitude;
                rov_sprintf_dsl_time_string(dateString);
                double rawHeading = theMicrostrain->raw_state.pos[DOF_HDG];
                double cookedHeading = rawHeading - headingOffset;
                double cookedPitch, cookedRoll;
                if(cookedHeading < 0.0)
                    {
                        cookedHeading += 360.0;
                    }
                if(headingOffset < 10.0)
                    {
                        cookedPitch =  theMicrostrain->raw_state.pos[DOF_PITCH] ;
                        cookedRoll = theMicrostrain->raw_state.pos[DOF_ROLL];
                    }
                else if(headingOffset  > 350.0)
                    {
                        cookedPitch =  theMicrostrain->raw_state.pos[DOF_PITCH] ;
                        cookedRoll = theMicrostrain->raw_state.pos[DOF_ROLL];
                    }
                else if((headingOffset > 80.0) && (headingOffset <100.0))
                    {
                        cookedPitch =  theMicrostrain->raw_state.pos[DOF_ROLL] ;
                        cookedRoll = theMicrostrain->raw_state.pos[DOF_PITCH];
                    }
                else if((headingOffset > 170.0) && (headingOffset <190.0))
                    {
                        cookedPitch =  -theMicrostrain->raw_state.pos[DOF_PITCH] ;
                        cookedRoll = -theMicrostrain->raw_state.pos[DOF_ROLL];
                    }
                else if((headingOffset > 260.0) && (headingOffset <280.0))
                    {
                        cookedPitch =  theMicrostrain->raw_state.pos[DOF_ROLL] ;
                        cookedRoll = -theMicrostrain->raw_state.pos[DOF_PITCH];
                    }
                myAttitude.heading = cookedHeading;
                myAttitude.pitch = cookedPitch;
                myAttitude.roll = cookedRoll;
                myAttitude.utime = theMicrostrain->most_recent_string_time;
                int success = myLcm.publish(attitudeChannelName,&myAttitude);
                int logLen = snprintf(loggingRecord,2047,"ATT %s HABCAM %0.2f %0.2f %0.2f", dateString, cookedHeading, cookedPitch, cookedRoll);
                if(logLen)
                    {
                        msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                    }
                break;
            }
        case SAS:
            {
                char dateString[256];
                int logLen = 0;
                rov_time_t sensorTime = rov_get_time();
                switch (in_hdr->from)
                    {
                    case CTD_THREAD:
                        {
                            double   c,t,p,s,ss;

                            rov_sprintf_dsl_time_string(dateString);
                            int items = sscanf( in_data, "%lf,%lf,%lf,%lf,%lf",&t,&c,&p,&s,&ss);
                            if(5 == items)
                                {
                                    sensor->ctd.conductivity = c;
                                    sensor->ctd.temperature = t;
                                    sensor->ctd.pressure = p;
                                    sensor->ctd.depth = p*PRESSURE_TO_DEPTH;
                                    sensor->ctd.salinity = s;
                                    sensor->ctd.sound_velocity= ss;
                                    logLen = snprintf(loggingRecord,2047,"CTD %s HABCAM %0.5f %0.3f %0.2f %0.2f %0.3f %0.5f", dateString,c,t,sensor->ctd.depth,p,s,ss);
                                }
                            else if(4 == items)
                                {
                                    sensor->ctd.conductivity = c;
                                    sensor->ctd.temperature = t;
                                    sensor->ctd.pressure = p;
                                    sensor->ctd.depth = p*PRESSURE_TO_DEPTH;
                                    sensor->ctd.salinity = s;
                                    sensor->ctd.sound_velocity = UNKNOWN_SOUND_SPEED;
                                    logLen = snprintf(loggingRecord,2047,"CTD %s HABCAM %0.5f %0.3f %0.2f %0.2f %0.3f %0.5f", dateString,c,t,sensor->ctd.depth,p,s);
                                }
                            else if(3 == items)
                                { // this is a seabird CTD
                                    sensor->ctd.conductivity = c;
                                    sensor->ctd.temperature = t;
                                    sensor->ctd.pressure = p;
                                    sensor->ctd.depth = p*PRESSURE_TO_DEPTH;
                                    sensor->ctd.sound_velocity = UNKNOWN_SOUND_SPEED;
                                    sensor->ctd.salinity = UNKNOWN_SALINITY;
                                    logLen = snprintf(loggingRecord,2047,"CTD %s HABCAM %0.5f %0.3f %0.2f %0.2f %0.3f", dateString,c,t,sensor->ctd.depth,p,s,ss);
                                }
                            else
                                {
                                    break;
                                }
                            marine_sensor::MarineSensorCtd_t myCtd;
                            myCtd.depth = sensor->ctd.depth;
                            myCtd.sea_water_electrical_conductivity = c;
                            myCtd.sea_water_pressure = p;
                            myCtd.sea_water_temperature = t;
                            myCtd.sea_water_salinity = sensor->ctd.salinity;
                            myCtd.utime =(long int)( 1000.0 * sensorTime);
                            int success = myLcm.publish(ctdChannelName,&myCtd);

                            if(logLen)
                                {
                                    msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                                }

                            break;
                        }
                    case FATHOMETER_THREAD:
                    case GPS_THREAD:
                        {
                            char  command[MAX_MESSAGE_LENGTH];
                            int logLen = 0;
                            int items = sscanf(in_data,"%s", command);
                            if(!items)
                                {
                                    break;
                                }
                            rov_sprintf_dsl_time_string(dateString);
                            if(!strncmp(command, "$GPGGA",6))
                                {
                                    int ignoreGPSChecksum = FALSE;
                                    gpgga_t  gpg = parse_gpgga(command,ignoreGPSChecksum);
                                    if(gpg.valid)
                                        {
                                            sensor->vesselPosition.latitude = RAD_TO_DEGREES(gpg.latitude);
                                            sensor->vesselPosition.longitude = RAD_TO_DEGREES(gpg.longitude);

                                            marine_sensor::MarineSensorGPS_t myGPS;
                                            myGPS.latitude = sensor->vesselPosition.latitude;
                                            myGPS.longitude = sensor->vesselPosition.longitude;
                                            myGPS.utime =(long int)( 1000.0 * sensorTime);
                                            logLen = snprintf(loggingRecord,2047,"GPS %s HABCAM %0.6f %0.6f", dateString,sensor->vesselPosition.latitude,sensor->vesselPosition.longitude);
                                            int success = myLcm.publish(gpsChannelName,&myGPS);
                                        }

                                }
                            else if(!strncmp(command, "$LCGGA",6)) // this must be the Kathy Marie
                                {
                                    gpgga_t  gpg = parse_lcgga(command);
                                    if(gpg.valid)
                                        {
                                            sensor->vesselPosition.latitude = RAD_TO_DEGREES(gpg.latitude);
                                            sensor->vesselPosition.longitude = RAD_TO_DEGREES(gpg.longitude);
                                            marine_sensor::MarineSensorGPS_t myGPS;
                                            myGPS.latitude = sensor->vesselPosition.latitude;
                                            myGPS.longitude = sensor->vesselPosition.longitude;
                                            myGPS.utime =(long int)( 1000.0 * sensorTime);
                                            logLen = snprintf(loggingRecord,2047,"GPS %s HABCAM %0.6f %0.6f", dateString,sensor->vesselPosition.latitude,sensor->vesselPosition.longitude);
                                            int success = myLcm.publish(gpsChannelName,&myGPS);
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
                                            sensor->fathometer.pos = myWaterDepth * fathometerMultiplier + fathometerBias;
                                            marine_sensor::MarineSensorFathometer_t myFathometer;
                                            myFathometer.depth = sensor->fathometer.pos;
                                            myFathometer.utime =(long int)( 1000.0 * sensorTime);
                                            logLen = snprintf(loggingRecord,2047,"FATH %s HABCAM %0.2f", dateString,myFathometer.depth);
                                            int success = myLcm.publish(fathometerChannelName,&myFathometer);
                                        }
                                }
                            if(logLen)
                                {
                                    msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                                }

                            break;
                        }

                    case ALTIMETER_THREAD:
                        {
                            double	altimeterTemp,range;
                            rov_sprintf_dsl_time_string(dateString);
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
                            if(items)
                                {
                                    marine_sensor::marineSensorAltimeter_t myAltimeter;
                                    myAltimeter.altitude = range;
                                    myAltimeter.utime =(long int)( 1000.0 * sensorTime);
                                    int success = myLcm.publish(altimeterChannelName,&myAltimeter);
                                    logLen = snprintf(loggingRecord,2047,"ALT %s HABCAM %0.3f ", dateString,myAltimeter.altitude);

                                }
                            if(logLen)
                                {
                                    msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
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
01-Apr-2023  jch      modified from sensor thread in rov code
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

            headingOffset = iniFile->readDouble( "MICROSTRAIN","HEADING_ROTATION",DEFAULT_MICROSTRAIN_HDG_OFFSET);
            if(headingOffset < 0.0)
                {
                    headingOffset += 360.0;
                }

            altimeterChannelName = iniFile->readString("ALTIMETER","CHANNEL_NAME", "ALTIMETER");
            ctdChannelName = iniFile->readString("CTD","CHANNEL_NAME","CTD");
            gpsChannelName = iniFile->readString("GPS","CHANNEL_NAME","GPS");
            fathometerChannelName = iniFile->readString("FATHOMETER","CHANNEL_NAME","FATHOMETER");
            attitudeChannelName = iniFile->readString("MICROSTRAIN","CHANNEL_NAME","ATTITUDE");
            fathometerMultiplier = iniFile->readDouble("FATHOMETER", "MULTIPLIER", 1.0);
            fathometerBias = iniFile->readDouble("FATHOMETER", "FATHOMETER_BIAS", 0.0);
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


                    // process the message
                    process_net_msg (&sensor, &in_hdr, (char *) in_data);



                }// end else
        }


}


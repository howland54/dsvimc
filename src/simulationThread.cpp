/* ----------------------------------------------------------------------

   code for tsimulating a pair of avt cameras and some other sensors on Habcam

   Modification History:
   DATE        AUTHOR   COMMENT
   1-April-2023  jch      creation, derive from bus thread
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

#include <string.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>


/* local includes */
#include "simulationThread.h"
#include "../../dsvimlib/include/time_util.h"
#include "../../dsvimlib/include/IniFile.h"


#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageTalk.h"		/* jasontalk protocol and structures */
#include "../../dsvimlib/include/msg_util.h"		/* utility functions for messaging */
#include "../../dsvimlib/include//launch_timer.h"

#include "../../habcam-lcmtypes/image/image/image_t.hpp"

#include "vimc.h"

#include "sensorThread.h"

/* posix header files */
#define  POSIX_SOURCE 1


#include <pthread.h>
extern avtCameraT  avtCameras[MAX_N_OF_CAMERAS];
extern int   nOfAvtCameras;

extern lcm::LCM myLcm;


extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

#define DEFAULT_IMAGE_FRAME_RATE    1.0
#define DEFAULT_CTD_RATE            2.0
#define DEFAULT_ALTIMETER_RATE      2.1
#define DEFAULT_ATTITUDE_RATE       10.0
#define DEFAULT_GPS_RATE            1.0
#define DEFAULT_FATHOMETER_RATE     1.5



/* ----------------------------------------------------------------------
   code for tsimulating a pair of avt cameras and some other sensors on Habcam

   Modification History:
   DATE        AUTHOR   COMMENT
   1-April-2023  jch      creation, derive from bus thread

---------------------------------------------------------------------- */
void *simulationThread (void *)
{

    msg_hdr_t hdr = { 0 };
    unsigned char data[MSG_DATA_LEN_MAX] = { 0 };
    double imageFrameRate = 1.0;
    double ctdRate = DEFAULT_CTD_RATE;
    double altimeterRate = DEFAULT_ALTIMETER_RATE;
    double attitudeRate = DEFAULT_ATTITUDE_RATE;
    double gpsRate = DEFAULT_GPS_RATE;
    double fathometerRate = DEFAULT_FATHOMETER_RATE;

    char *imageFileName = NULL;
    char *ctdChannelName = NULL;
    char *altimeterChannelName = NULL;
    char *attitudeChannelName = NULL;
    char *gpsChannelName = NULL;
    char *fathometerChannelName = NULL;
    // wakeup message
    printf ("SIMULATION_THREAD (thread %d) initialized \n", SIMULATION_THREAD);
    printf ("SIMULATION_THREAD File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);


    IniFile  *iniFile = new IniFile();
    int okINI = iniFile->openIni(flyIniFile);
    bool stereoLogging;
    if(GOOD_INI_FILE_READ == okINI)
        {
            char *scratchString = iniFile->readString("SIMULATION", "IMAGE_FILE", "NOFILE");
            if(!strncmp(scratchString,"NOFILE", 6))
                {
                    fprintf(stderr, "simulation thread couldnt find image descriptor, exiting\n");
                    fflush (stdout);
                    fflush (stderr);
                    abort ();
                }
            else
                {
                    imageFileName = strdup(scratchString);
                    free(scratchString);
                }
            imageFrameRate = iniFile->readDouble("SIMULATION", "IMAGE_FRAME_RATE", DEFAULT_IMAGE_FRAME_RATE);
            ctdRate = iniFile->readDouble("SIMULATION", "CTD_RATE", DEFAULT_CTD_RATE);
            altimeterRate = iniFile->readDouble("SIMULATION", "ALTIMETER_RATE", DEFAULT_ALTIMETER_RATE);
            attitudeRate = iniFile->readDouble("SIMULATION", "ATTITUDE_RATE", DEFAULT_ATTITUDE_RATE);
            gpsRate = iniFile->readDouble("SIMULATION", "GPS_RATE", DEFAULT_GPS_RATE);
            fathometerRate = iniFile->readDouble("SIMULATION", "FATHOMETER_RATE", DEFAULT_FATHOMETER_RATE);

            ctdChannelName = iniFile->readString("CTD","CHANNEL_NAME","CTD");
            altimeterChannelName = iniFile->readString("ALTIMETER","CHANNEL_NAME", "ALTIMETER");
            attitudeChannelName = iniFile->readString("MICROSTRAIN","CHANNEL_NAME", "ATTITUDE");
            gpsChannelName = iniFile->readString("GPS","CHANNEL_NAME","GPS");
            fathometerChannelName = iniFile->readString("FATHOMETER","CHANNEL_NAME","FATHOMETER");

        }
    else
        {
            printf ("%s ini file does not exist...simulation thread exiting!\n", flyIniFile);
            fflush (stdout);
            fflush (stderr);
            abort ();

        }

    // now load the simulated image;

    cv::Mat  image = cv::imread(imageFileName,-1);
    cv::Mat  leftImage;
    cv::Mat  rightImage;

    leftImage.create(image.rows, image.cols/2, CV_16UC1);
    rightImage.create(image.rows, image.cols/2, CV_16UC1);

    for(int j = 0; j < image.rows; j++)
        {
            for (int k = 0; k < image.cols/2; k++)
            {
                leftImage.at<unsigned short>(j,k) = image.at<unsigned short>(j,k)/16.0;
                rightImage.at<unsigned short>(j,k) = image.at<unsigned short>(j,k + image.cols/2)/16.0;
            }
        }

    image::image_t leftImageToPublish;
    image::image_t rightImageToPublish;
    leftImageToPublish.width = (int32_t)leftImage.cols;
    leftImageToPublish.height = (int32_t)leftImage.rows;
    leftImageToPublish.size = (int32_t)(leftImageToPublish.width * leftImageToPublish.height * 2);
    leftImageToPublish.pixelformat = image::image_t::PIXEL_FORMAT_GRAY;
    leftImageToPublish.data.resize((uint32_t)leftImageToPublish.size);

    unsigned char *dataPointer = reinterpret_cast<unsigned char *>(leftImage.data);

    std::copy(dataPointer, dataPointer + leftImageToPublish.size, leftImageToPublish.data.begin() );

    //memcpy(leftImageToPublish.data.front(),dataPointer,leftImageToPublish.size);

    rightImageToPublish.width = (int32_t)rightImage.cols;
    rightImageToPublish.height = (int32_t)rightImage.rows;
    rightImageToPublish.size = (int32_t)(rightImageToPublish.width * rightImageToPublish.height * 2);
    rightImageToPublish.pixelformat = image::image_t::PIXEL_FORMAT_GRAY;
    rightImageToPublish.data.resize((uint32_t)rightImageToPublish.size);

    dataPointer = reinterpret_cast<unsigned char *>(rightImage.data);

    std::copy(dataPointer, dataPointer + rightImageToPublish.size, rightImageToPublish.data.begin() );

    // now start the image timer
    double timerInterval = 1.0/imageFrameRate;

    // sleep for a while to give the system time to get ready

    usleep(900000);


    launched_timer_data_t * imageTimer = launch_timer_new(timerInterval, -1, SIMULATION_THREAD, SIMULATION_TICK1);
    launched_timer_data_t * ctdTimer = launch_timer_new(1.0/ctdRate, -1,SIMULATION_THREAD, SIMULATION_TICK2);
    launched_timer_data_t * altimeterTimer = launch_timer_new(1.0/altimeterRate, -1, SIMULATION_THREAD, SIMULATION_TICK3);
    launched_timer_data_t * attitudeTimer = launch_timer_new(1.0/attitudeRate, -1, SIMULATION_THREAD, SIMULATION_TICK4);
    launched_timer_data_t * gpsTimer = launch_timer_new(1.0/gpsRate, -1, SIMULATION_THREAD, SIMULATION_TICK5);
    launched_timer_data_t * fathometerTimer = launch_timer_new(1.0/fathometerRate, -1, SIMULATION_THREAD, SIMULATION_TICK6);


    int msg_success = msg_queue_new(SIMULATION_THREAD, "simulation thread");

    if(msg_success != MSG_OK)
        {
            fprintf(stderr, "simulation thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
            fflush (stdout);
            fflush (stderr);
            abort ();
        }
    // initialize some sensor data

    double simDepth = 50.0;
    double simT = 10.0;

    double simAlt = 3.1;
    double simPitch = 0.0;
    double simRoll = 0.0;
    double simHead = 30.0;
    double simLatitude = 41.55;
    double simLon = -70.61;
    double simFathometer = simDepth + simAlt;

    double depthNoise = 0.3;
    double tNoise = 0.2;
    double altNoise = 0.8;
    double attitudeNoise = 0.8;
    double lonNoise = 0.002;
    double latNoise = 0.0025;
    double fathometerNoise = 0.9;
    double conductivityNoise = 0.003;
    double defaultConductivity = 0.0034567;



    // loop forever
    while (1)
        {

            int msg_get_success = msg_get(SIMULATION_THREAD,&hdr, data, MSG_DATA_LEN_MAX);
            if(msg_get_success != MSG_OK)
                {
                    fprintf(stderr, "simulation thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
                }
            else
                {
                    // process new message
                    switch (hdr.type)
                        {

                        case SIMULATION_TICK1:
                            {
                                rov_time_t leftImageTime = rov_get_time();
                                leftImageToPublish.utime =(long int)( 1000.0 * leftImageTime);

                                int success = myLcm.publish(avtCameras[0].lcmChannelName,&leftImageToPublish);
                                printf(".");
                                rov_time_t rightImageTime = rov_get_time();
                                rightImageToPublish.utime =(long int)( 1000.0 * rightImageTime);
                                success = myLcm.publish(avtCameras[1].lcmChannelName,&rightImageToPublish);
                                printf(".");
                                fflush(stdout);


                                break;
                            }
                        case SIMULATION_TICK2:
                            {
                                double depthBias = (((double)rand() / RAND_MAX) - 0.5) * depthNoise;
                                double tBias = (((double)rand() / RAND_MAX) - 0.5) * tNoise;
                                double conductivityBias = (((double)rand() / RAND_MAX) - 0.5) * conductivityNoise;
                                rov_time_t ctdTime = rov_get_time();
                                marine_sensor::MarineSensorCtd_t myCtd;
                                myCtd.depth = simDepth + depthBias;
                                myCtd.sea_water_electrical_conductivity = defaultConductivity + conductivityBias;
                                myCtd.sea_water_pressure = myCtd.depth/PRESSURE_TO_DEPTH;
                                myCtd.sea_water_temperature = simT + tBias;
                                myCtd.sea_water_salinity = UNKNOWN_SALINITY;

                                int success = myLcm.publish(ctdChannelName,&myCtd);
                                break;
                            }
                        case SIMULATION_TICK3:
                            {
                                double altBias = (((double)rand() / RAND_MAX) - 0.5) * altNoise;
                                rov_time_t altimeterTime = rov_get_time();
                                marine_sensor::marineSensorAltimeter_t myAltimeter;
                                myAltimeter.altitude = simAlt + altBias;
                                int success = myLcm.publish(altimeterChannelName,&myAltimeter);

                                break;
                            }
                        case SIMULATION_TICK4:
                            {
                                double attitudeBias = (((double)rand() / RAND_MAX) - 0.5) * attitudeNoise;
                                rov_time_t attitudeTime = rov_get_time();
                                marine_sensor::MarineSensorAttitudeSensor_t myAttitude;
                                myAttitude.heading = simHead + attitudeBias;  // check units
                                myAttitude.pitch = simPitch + attitudeBias;
                                myAttitude.roll = simRoll + attitudeBias;

                                int success = myLcm.publish(attitudeChannelName,&myAttitude);
                                break;
                            }
                        case SIMULATION_TICK5:
                            {
                                double latitudeBias = (((double)rand() / RAND_MAX) - 0.5) * latNoise;
                                double longitudeBias = (((double)rand() / RAND_MAX) - 0.5) * lonNoise;
                                marine_sensor::MarineSensorGPS_t myGPS;
                                myGPS.latitude = simLatitude + latitudeBias;
                                myGPS.longitude = simLon + longitudeBias;
                                rov_time_t gpsTime = rov_get_time();
                                int success = myLcm.publish(gpsChannelName,&myGPS);
                                break;
                            }
                        case SIMULATION_TICK6:
                            {
                                double fathometerBias = (((double)rand() / RAND_MAX) - 0.5) * fathometerNoise;
                                rov_time_t fathometerTime = rov_get_time();
                                marine_sensor::MarineSensorFathometer_t myFathometer;
                                myFathometer.depth = simFathometer + fathometerBias;
                                int success = myLcm.publish(fathometerChannelName,&myFathometer);
                                break;
                            }
                        case PNG:
                            {			// recieved a PNG (Ping) message

                                const char *msg = "vimc bus thread is Alive!";
                                // respond with a SPI (Status Ping) message
                                msg_send( hdr.from, SIMULATION_THREAD, SPI, strlen (msg), msg);
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

                                printf ("simulation: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", BUS_THREAD, hdr.type, hdr.from, hdr.to, hdr.length);
                                break;
                            }

                        }			// switch
                } // end els msg get ok
        }

}

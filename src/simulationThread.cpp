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


/* posix header files */
#define  POSIX_SOURCE 1


#include <pthread.h>
extern avtCameraT  avtCameras[MAX_N_OF_CAMERAS];
extern int   nOfAvtCameras;

extern lcm::LCM myLcm;


extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

#define DEFAULT_IMAGE_FRAME_RATE    1.0



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

    char *imageFileName = NULL;
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
                leftImage.at<unsigned short>(j,k) = image.at<unsigned short>(j,k);
                rightImage.at<unsigned short>(j,k) = image.at<unsigned short>(j,k + image.cols/2);
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


    int msg_success = msg_queue_new(SIMULATION_THREAD, "simulation thread");

    if(msg_success != MSG_OK)
        {
            fprintf(stderr, "simulation thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
            fflush (stdout);
            fflush (stderr);
            abort ();
        }


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

                                rov_time_t rightImageTime = rov_get_time();
                                rightImageToPublish.utime =(long int)( 1000.0 * rightImageTime);
                                success = myLcm.publish(avtCameras[1].lcmChannelName,&rightImageToPublish);


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

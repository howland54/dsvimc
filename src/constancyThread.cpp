/* ----------------------------------------------------------------------


20 Feb 2023   jch    former rov control code used as shell for habcam color constancy thread

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
#include "color_constancy.hpp"

#include "constancyThread.h"

extern cv::Mat  constancyImage;

constancyIlluminationVector_t theIlluminationVector;
extern pthread_mutex_t constancyMutex;
extern pthread_mutex_t illuminationMutex;


/* ----------------------------------------------------------------------

Sensor Thread

Modification History:
DATE         AUTHOR  COMMENT
26-Mar-2025  jch      modified from sensor thread in habcam code
---------------------------------------------------------------------- */


void *constancyThread (void *)
{

    msg_hdr_t in_hdr = { 0 };
    unsigned char in_data[MSG_DATA_LEN_MAX] = { 0 };


    int msg_success = msg_queue_new(CONSTANCY_THREAD, "constancy thread");
    if(msg_success != MSG_OK)
        {
            fprintf(stderr, "constancy thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
            fflush (stdout);
            fflush (stderr);
            abort ();
        }

    // wakeup message
    printf ("CONSTANCY (thread %d) initialized \n", CONSTANCY_THREAD);
    printf ("CONSTANCY File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);

    theIlluminationVector.validData = false;


    // loop forever
    while (1)
        {

            // wait forever on the input channel
#ifdef DEBUG_CONSTANCY
            printf ("Constancy Thread calling get_message.\n");
#endif

            int msg_get_success = msg_get(CONSTANCY_THREAD,&in_hdr, in_data, MSG_DATA_LEN_MAX);
            if(msg_get_success != MSG_OK)
                {
                    fprintf(stderr, "constancy thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
                }
            else{
#ifdef DEBUG_CONSTANCY
                    // print on stderr
                    printf ("CONSTANCY got msg type %d from proc %d to proc %d, %d bytes data\n", in_hdr.type, in_hdr.from, in_hdr.to, in_hdr.length);
#endif


                    // process the message
                    switch (in_hdr.type)
                        {
                        case PNG:			/* respond to ping request */
                            {
                                // respond with a SPI (Status Ping) message
                                msg_send(  in_hdr.from, in_hdr.to, SPI, 0,NULL);
                                break;
                            }
                        case WCON:
                           {
                              color_correction::gray_world b1;
                              float ml,ma,mb;
                              pthread_mutex_lock(&constancyMutex);
                              cv::Mat inImage = constancyImage.clone();
                              pthread_mutex_unlock(&constancyMutex);
                              b1.preprocess(inImage,1,2,&ml,&ma,&mb);
                              pthread_mutex_lock(&illuminationMutex);
                              theIlluminationVector.ml = ml;
                              theIlluminationVector.ma = ma;
                              theIlluminationVector.mb = mb;
                              theIlluminationVector.validData = true;
                              pthread_mutex_unlock(&illuminationMutex);
                              break;
                           }
                        default:			// recieved an unknown message type
                           {
                               printf ("Color Constancy Thread: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", CONSTANCY_THREAD, in_hdr.type, in_hdr.from, in_hdr.to, in_hdr.length);
                               break;
                           }
                        }



                }// end else
        }


}


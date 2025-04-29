/* ----------------------------------------------------------------------


20 Feb 2023   jch    former rov control code used as shell for habcam jpeg saving thread

---------------------------------------------------------------------- */
/* standard ansi C header files */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* posix header files */
#define  POSIX_SOURCE 1
#include <pthread.h>

#include <string.h>
#include <sys/stat.h>
#include <exiv2/exiv2.hpp>


/* jason header files */
#include "../../dsvimlib/include/imageTalk.h"
#include <../../dsvimlib/include/mkdir_p.h>

#include "../../dsvimlib/include//msg_util.h"
#include "../../dsvimlib/include//launch_timer.h"
#include "../../dsvimlib/include//time_util.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageA2b.h"
#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/IniFile.h"
#include "vimc.h"
#include "color_constancy.hpp"

#include "jpegThread.h"

extern cv::Mat  jpegImage;
extern char *recordingPrefix;

extern jpegSave_t  jpegToSave[2];
extern pthread_mutex_t jpegMutex[2];

typedef struct
{
    int lastYear;
    int lastMonth;
    int lastDay;
    int lastHour;
    int lastTenMinute;
    int lastDayOfYear;
} lastJPGsT;

static lastJPGsT lastJPEGS[2];
static char theJPGDataDir[2][512];

extern avtCameraT  avtCameras[MAX_N_OF_CAMERAS];

extern int makeTimeString (double total_secs, int year, int month, int monthDay, int hour, int min, int seconds, int milliseconds, char *str, char *prefix, char *suffix);

/* ----------------------------------------------------------------------

jpeg Thread

Modification History:
DATE         AUTHOR  COMMENT
26-Mar-2025  jch      modified from sensor thread in habcam code
---------------------------------------------------------------------- */


void *jpegThread (void *)
{

    msg_hdr_t in_hdr = { 0 };
    unsigned char in_data[MSG_DATA_LEN_MAX] = { 0 };


    int msg_success = msg_queue_new(JPEG_THREAD, "jpeg thread");
    if(msg_success != MSG_OK)
        {
            fprintf(stderr, "jpeg thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
            fflush (stdout);
            fflush (stderr);
            abort ();
        }

    // wakeup message
    printf ("JPEG (thread %d) initialized \n", JPEG_THREAD);
    printf ("JPEG File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);

    pthread_mutex_init(&(jpegMutex[0]), NULL);
    pthread_mutex_init(&(jpegMutex[1]), NULL);

    // loop forever
    while (1)
        {

            // wait forever on the input channel
#ifdef DEBUG_JPEG
            printf ("Jpeg Thread calling get_message.\n");
#endif

            int msg_get_success = msg_get(JPEG_THREAD,&in_hdr, in_data, MSG_DATA_LEN_MAX);
            if(msg_get_success != MSG_OK)
                {
                    fprintf(stderr, "jpeg thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
                }
            else
               {
#ifdef DEBUG_CONSTANCY
                    // print on stderr
                    printf ("jpeg got msg type %d from proc %d to proc %d, %d bytes data\n", in_hdr.type, in_hdr.from, in_hdr.to, in_hdr.length);
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
                        case WJPG:
                          {
                             bool needANewJPGDirectory[2];
                             needANewJPGDirectory[0] = false;
                             needANewJPGDirectory[1] = false;
                             int hour;
                             int dayOfYear;
                             int month;
                             int minute;
                             int *theCameraNumber = (int *) in_data;
                             int cameraNumber = *theCameraNumber;
                             pthread_mutex_lock(&(jpegMutex[cameraNumber]));
                             hour = jpegToSave[cameraNumber].hour;
                             dayOfYear = jpegToSave[cameraNumber].dayOfYear;
                             month = jpegToSave[cameraNumber].month;
                             minute = jpegToSave[cameraNumber].minute;
                             int year = jpegToSave[cameraNumber].year;
                             int secs = jpegToSave[cameraNumber].secs;
                             int milliseconds = jpegToSave[cameraNumber].milliseconds;
                             cv::Mat imageToSave = jpegToSave[cameraNumber].theImage.clone();
                             int day = jpegToSave[cameraNumber].day;
                             char description[2050];
                             strncpy(description,jpegToSave[cameraNumber].imageDescription, 2047);
                             pthread_mutex_unlock(&(jpegMutex[cameraNumber]));
                             int	trialTenMinute = minute - (minute % 10);


                             if(dayOfYear != lastJPEGS[cameraNumber].lastDayOfYear)
                                {
                                   needANewJPGDirectory[cameraNumber] = true;
                                   lastJPEGS[cameraNumber].lastDayOfYear = dayOfYear;

                                }
                             else
                                {
                                   if(hour != lastJPEGS[cameraNumber].lastHour)
                                      {
                                         needANewJPGDirectory[cameraNumber] = true;
                                         lastJPEGS[cameraNumber].lastHour = hour;
                                      }
                                   else
                                      {
                                         if(trialTenMinute != lastJPEGS[cameraNumber].lastTenMinute)
                                            {
                                               needANewJPGDirectory[cameraNumber] = true;
                                               lastJPEGS[cameraNumber].lastTenMinute = trialTenMinute;
                                            }
                                      }
                                }
                             if(needANewJPGDirectory[cameraNumber])
                                {
                                   snprintf(&(theJPGDataDir[cameraNumber][0]),511,"%s/%04d%02d%02d/%04d%02d%02d_%02d/%04d%02d%02d_%02d%02d",avtCameras[cameraNumber].jpgSaveRoot,year,month+1,day,
                                            year,month+1,day,hour,
                                            year,month+1,day,hour,trialTenMinute );
                                   struct stat sb;
                                   if (stat(&(theJPGDataDir[cameraNumber][0]), &sb) != 0)
                                      {
                                         /* path does not exist - create directory */
                                         int directoryRetCode = mkdir_p(&(theJPGDataDir[cameraNumber][0]),ACCESSPERMS);
                                         if (directoryRetCode < 0)
                                            {
                                               printf("error creating directory %s\n,&(theJPGDataDir[cameraNumber][0])");
                                            }
 #if 0
                                         else
                                            {
                                               char loggingRecord[2048];
                                               int logLen = snprintf(loggingRecord,2047,"SYS %04d/%02d/%02d %02d:%02d:%02d.%03d MKDIR %s RETCODE %d",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,theJPGDataDir,directoryRetCode);
                                               msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                                            }
#endif
                                      }

                                }
                             char jpgPrefix[512];
                             char jpgImageName[768];
                             char jpgImageTimeString[512];
                             snprintf(jpgPrefix,511,"%s%s",avtCameras[cameraNumber].jpgPrefix,recordingPrefix);
                             //int numChars = makeTimeString(thePairTime,jpgImageTimeString,jpgPrefix, "jpg");
                             int numChars = makeTimeString(3.0,year, month+1, day, hour, minute, secs, milliseconds, jpgImageTimeString,jpgPrefix, "jpg");

                             snprintf(jpgImageName,767,"%s/%s",&(theJPGDataDir[cameraNumber][0]),jpgImageTimeString);
                             bool jpgWriteResult;
                            // if(0 == cameraNumber)
                                {
                                   //jpgWriteResult = cv::imwrite(jpgImageName,leftJpegImage);
                                   jpgWriteResult = cv::imwrite(jpgImageName,imageToSave);
                                   /*cv::namedWindow("left");
                                                cv::imshow("lefts",leftColorImage);
                                                  cv::waitKey(0);
                                                 cv::destroyWindow("lefts");*/
                                }
 #if 0
                             else
                                {
                                   //jpgWriteResult = cv::imwrite(jpgImageName,rightJpegImage);
                                   jpgWriteResult = cv::imwrite(jpgImageName,rightJpegImage);
                                }
 #endif
                             //printf(" wrote %s\n",jpgImageName);
                             if(jpgWriteResult)
                                {
                                   Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(jpgImageName);
                                   image->readMetadata();
                                   Exiv2::ExifData &exifData = image->exifData();
                                   exifData["Exif.Image.ImageDescription"] = description;
                                   image->writeMetadata();
                                }


                             break;
                          }

                        default:			// recieved an unknown message type
                           {
                               printf ("JpegThread: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", CONSTANCY_THREAD, in_hdr.type, in_hdr.from, in_hdr.to, in_hdr.length);
                               break;
                           }
                        }



                }// end else
        }


}


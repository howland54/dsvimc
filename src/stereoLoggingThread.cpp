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

#include <time.h>
#include <sys/timeb.h>
#include <sys/stat.h>

#include <iostream>
#include <sstream>

/* local includes */
#include "../../dsvimlib/include/time_util.h"

#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageTalk.h"		/* jasontalk protocol and structures */
#include "../../dsvimlib/include/msg_util.h"		/* utility functions for messaging */
#include "lcmHandleThread.h"
#include "stereoLoggingThread.h"

/* posix header files */
#define  POSIX_SOURCE 1


#include <pthread.h>
#include "vimc.h"
extern avtCameraT  avtCameras[MAX_N_OF_CAMERAS];
char  *stereoSaveRoot;
char *imgRoot;
bool  saveStereo;
extern int   nOfAvtCameras;

long int leftTime;
long int rightTime;

bool stereoWriteResult;

int makeTimeString (double total_secs, char *str, char *prefix, char *suffix);

static int lastYear;
static int lastMonth;
static int lastDay;
static int lastHour;
static int lastTenMinute;
static int lastDayOfYear;

int     stereoPairCount;
extern stereo_event_t theStereoEvent;

extern lcm::LCM myLcm;


FILE *tenMinuteLogFile;

class State
{
public:
    int usefulVariable;
};

extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

int leftCameraID;
int rightCameraID;
bool amIStereoRecording;

static cv::Mat  leftImage;
static cv::Mat  rightImage;
static cv::Mat  workingImage;

cv::Mat  leftColorImage;
cv::Mat  rightColorImage;
cv::Mat  leftNormalizedImage;
cv::Mat  rightNormalizedImage;
cv::Mat dst;


char *recordingPrefix;

void recordingParameterCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const image::image_parameter_t *image, State *user)
{
    if("RECORDING" == image->key)
        {
            if("0" == image->value)
                {
                    amIStereoRecording = false;
                }
            else if("1" == image->value)
                {
                    amIStereoRecording = true;
                }
        }
}

void stereoCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const image::image_t *image, State *user)
{


    bool needNewDirectory = false;
    char theDataDir[512];
    theStereoEvent.imageState = 0;
    image::image_t leftImageToPublish;
    image::image_t rightImageToPublish;
    if(channel == avtCameras[leftCameraID].lcmChannelName)
        {
            leftTime = image->utime;
            workingImage = cv::Mat(image->height, image->width, CV_16UC1, (void *)image->data.data());
            leftImage = workingImage.clone();


            leftColorImage = cv::Mat(image->height, image->width, CV_16UC3);
            //leftNormalizedImage = cv::Mat(image->height, image->width, CV_16UC1);
            dst = cv::Mat(image->height, image->width, CV_8UC3);;
            //cv::normalize(leftImage,leftNormalizedImage,0, 255,cv::NORM_MINMAX);

            cv::cvtColor(leftImage,leftColorImage,cv::COLOR_BayerBG2BGR,0);
            leftColorImage.convertTo(dst,CV_8UC3,0.003891051); // 1/257 to get the full range

            leftImageToPublish.width = image->width;
            leftImageToPublish.height = image->height;
            leftImageToPublish.size = image->width * image->height * 3;
            leftImageToPublish.data.resize((uint32_t)leftImageToPublish.size);
            leftImageToPublish.pixelformat = image::image_t::PIXEL_FORMAT_BGR;
            leftImageToPublish.utime =image->utime;
            std::copy(dst.datastart, dst.datastart +  leftImageToPublish.size, leftImageToPublish.data.begin());
            int success = myLcm.publish("LeftColor",&leftImageToPublish);


            if(avtCameras[rightCameraID].doNotUseInStereoLogging)
                {
                    rightTime = leftTime;
                    rightImage = cv::Mat(image->height, image->width, CV_8U, 128);
                    theStereoEvent.imageState = 1;
                }
            /*cv::namedWindow("left");
         cv::imshow("left",leftColorImage);
         cv::waitKey(0); // Wait for any keystroke in the window

         cv::destroyWindow("windowName"); //destroy the created window* */

            //printf(" got a left\n");

        }
    else
        {
            rightTime = image->utime;
            workingImage = cv::Mat(image->height, image->width, CV_16UC1, (void *)image->data.data());
            rightImage = workingImage.clone();


            rightColorImage = cv::Mat(image->height, image->width, CV_16UC3);
            //rightNormalizedImage = cv::Mat(image->height, image->width, CV_16UC1);
            dst = cv::Mat(image->height, image->width, CV_8UC3);;
            //cv::normalize(rightImage,rightNormalizedImage,0, 255,cv::NORM_MINMAX);

            cv::cvtColor(rightImage,rightColorImage,cv::COLOR_BayerBG2BGR,0);
            rightColorImage.convertTo(dst,CV_8UC3,0.003891051); // 1/257 to get the full range

            rightImageToPublish.width = image->width;
            rightImageToPublish.height = image->height;
            rightImageToPublish.size = image->width * image->height * 3;
            rightImageToPublish.data.resize((uint32_t)rightImageToPublish.size);
            rightImageToPublish.pixelformat = image::image_t::PIXEL_FORMAT_BGR;
            rightImageToPublish.utime =image->utime;
            std::copy(dst.datastart, dst.datastart +  rightImageToPublish.size , rightImageToPublish.data.begin());
            //std::string theTopic("RightColor");
            int success = myLcm.publish("RightColor",&rightImageToPublish);


            if(avtCameras[leftCameraID].doNotUseInStereoLogging)
                {
                    leftTime = rightTime;
                    leftImage = rightImage;
                    rightImage.operator=(128);
                    theStereoEvent.imageState = 2;
                }
            // cv::namedWindow("right");
            // cv::imshow("right",rightImage);

            //printf(" got a right\n");

        }
            if(!amIStereoRecording)
                {
                    printf(" not logging!\n");
                    return;
                }

    if(leftTime && rightTime)
        {
            if(abs(leftTime - rightTime) < INTERIMAGE_CRITERIA)
                {
                    // store the concatenated image!
                    // use the left time to make a file name
                    // compute the unix time of the left time
                    /*cv::namedWindow("lefts");
               cv::imshow("lefts",leftImage);
               cv::namedWindow("rights");
               cv::imshow("rights",rightImage);
                 cv::waitKey(0);
                cv::destroyWindow("lefts");
                cv::destroyWindow("rights" );*/


                    stereoPairCount++;
                    double thePairTime = (double)leftTime/1000.0;
                    struct timeb  ftime_time;
                    struct tm     gmtime_time;

                    ftime_time.time      = (time_t) floor(thePairTime);
                    ftime_time.millitm   = (unsigned short int) (fmod(thePairTime,1.0) * 1000.0);
                    ftime_time.timezone  =  0;
                    ftime_time.dstflag   =  0;

                    gmtime_r(&ftime_time.time, &gmtime_time);
                    int year = gmtime_time.tm_year+1900;
                    int month = gmtime_time.tm_mon;
                    int day = gmtime_time.tm_mday;
                    int hour = gmtime_time.tm_hour;
                    int minute = gmtime_time.tm_min;
                    int dayOfYear = gmtime_time.tm_yday;

                    int	trialTenMinute = minute - (minute % 10);

                    if(dayOfYear != lastDayOfYear)
                        {
                            needNewDirectory = true;
                            lastDayOfYear = dayOfYear;
                        }
                    else
                        {
                            if(hour != lastHour)
                                {
                                    needNewDirectory = true;
                                    lastHour = hour;
                                }
                            else
                                {
                                    if(trialTenMinute != lastTenMinute)
                                        {
                                            needNewDirectory = true;
                                            lastTenMinute = trialTenMinute;
                                        }
                                }
                        }



                    if(needNewDirectory)
                        {
                            snprintf(theDataDir,511,"%s/%04d%02d%02d/%04d%02d%02d_%02d/%04d%02d%02d_%02d%02d",stereoSaveRoot,year,month+1,day,
                                     year,month+1,day,hour,
                                     year,month+1,day,hour,trialTenMinute );
                            int directoryRetCode = mkdir_p(theDataDir,ACCESSPERMS);
                            char dataFileName[512];
                            snprintf(dataFileName,511,"%s/%04d%02d%02d_%02d%02d.img",imgRoot,year,month+1,day,hour,trialTenMinute);
                            if(tenMinuteLogFile)
                                {
                                    fclose(tenMinuteLogFile);
                                }
                            tenMinuteLogFile = fopen(dataFileName,"wa");
                            //int logLen = snprintf(loggingRecord,2047,"SYS %04d/%02d/%02d %02d:%02d:%02d.%03d MKDIR %s RETCODE %d",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,theDataDir,directoryRetCode);
                            //msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                        }
                    char imageTimeString[256];
                    std::vector<int> tags;
                    char imageName[768];

                    //  cv::waitKey(0);
                    //  cv::destroyWindow("left");
                    //  cv::destroyWindow("right");


                    int numChars = makeTimeString(thePairTime,imageTimeString,recordingPrefix, "tif");
                    cv::Mat stereoImage = cv::Mat(image->height, image->width*2, CV_16UC1);
                    cv::hconcat(leftImage,rightImage,stereoImage);
                    snprintf(imageName,767,"%s/%s",theDataDir,imageTimeString);
                    tags = {TIFFTAG_COMPRESSION, COMPRESSION_NONE};
                    stereoWriteResult = cv::imwrite(imageName,stereoImage,tags);



                    if(stereoWriteResult)
                        {
                            /*char noCommaDescription[1024];
                            int len = snprintf(noCommaDescription,1024, "%s %.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.5f,%.2f,%.5f,%d,%d,%d,%d,%.4f,%.4f,%.2f",imageTimeString,theStereoEvent.latitude, theStereoEvent.longitude,theStereoEvent.heading,theStereoEvent.pitch,
                                theStereoEvent.roll, theStereoEvent.altitude0,theStereoEvent.altitude1,theStereoEvent.depth,theStereoEvent.salinity, theStereoEvent.temperature, theStereoEvent.dO,
                                theStereoEvent.cdom, theStereoEvent.fluor, theStereoEvent.scatter, theStereoEvent.therm,theStereoEvent.ph1, theStereoEvent.ph2,theStereoEvent.fathometer);
                            msg_send(LOGGING_THREAD, ANY_THREAD, LOG,len,noCommaDescription);
                            if(tenMinuteLogFile)
                                {
                                    fprintf(tenMinuteLogFile,"%s\n",noCommaDescription);
                                    fflush(tenMinuteLogFile);
                                }
                        */

                        }


                }

            // 10 June 2015 jch replace conductivity with salinity

            //fprintf(logger->theLogDirFile,"%s,%s\n",unqualifiedFileName,noCommaDescription);
            leftTime = 0;
            rightTime = 0;

        }





            // int logLen = snprintf(loggingRecord,2047,"IMG %04d/%02d/%02d %02d:%02d:%02d.%03d IMWRITE %s %d %0.2f %0.2f",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,fileName,avtCameras[cameraNumber].actualSettings.cameraSynced,
            //msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
            // cv::imwrite(imageTimeString,stereoImage);




}

int makeTimeString (double total_secs, char *str, char *prefix, char *suffix)
{
    int num_chars;

    // for min, sec

    double secs_in_today;
    //   double day;
    double hour;
    double min;
    double sec;

    // for date and hours
    struct tm *tm;
    time_t current_time;


    // read gettimeofday() clock and compute min, and
    // sec with microsecond precision

    secs_in_today = fmod (total_secs, 24.0 * 60.0 * 60.0);
    hour = secs_in_today / 3600.0;
    min = fmod (secs_in_today / 60.0, 60.0);
    sec = fmod (secs_in_today, 60.0);
    int msec = 1000.0 * (sec - (int)sec);

    // call time() and localtime for hour and date
    current_time = time (NULL);
    tm = localtime (&current_time);

    // do I need  a new directory?

    int trialYear = tm->tm_year + 1900;
    int trialMonth = tm->tm_mon + 1;
    int trialDay = tm->tm_mday;
    int trialHour = tm->tm_hour;
    int	trialTenMinute = tm->tm_min - (tm->tm_min % 10);


    num_chars = sprintf (str, "%s.%04d%02d%02d.%02d%02d%02d%03d.%d.%s",prefix,(int) tm->tm_year +1900, (int) tm->tm_mon + 1, (int) tm->tm_mday, (int) tm->tm_hour, (int) min, (int)sec, msec,stereoPairCount,suffix);


    return num_chars;

}

/*
 * snprintf(unqualifiedFileName,512,"%s.%04d%02d%02d.%02d%02d%02d%03d.%d.tif",imageNamePrefix,gmtTime->tm_year+1900,gmtTime->tm_mon+1,gmtTime->tm_mday, gmtTime->tm_hour,
                        gmtTime->tm_min,gmtTime->tm_sec,milliseconds,pairNumber);
               snprintf (filename, sizeof filename, "%s/%s",
                         logger->currentSubDirectory,unqualifiedFileName );
                         */

/* ----------------------------------------------------------------------
   stereo logging Thread


   Modification History:
   DATE        AUTHOR   COMMENT
   16 Sept 2019  jch      creation, derive from stereo logging thread

---------------------------------------------------------------------- */
void *stereoLoggingThread (void *)
{

    msg_hdr_t hdr = { 0 };
    unsigned char data[MSG_DATA_LEN_MAX] = { 0 };
    lcm::LCM stereoLcm("udpm://239.255.76.67:7667?ttl=0");
    State state;
    tenMinuteLogFile = NULL;
    leftTime = 0;
    rightTime = 0;
    stereoPairCount = 0;
    amIStereoRecording  = true;
    theStereoEvent.dO = 0.012;
    theStereoEvent.ph1 = 0.123;
    theStereoEvent.ph2 = 0.234;
    theStereoEvent.cdom = 345;
    theStereoEvent.roll = 0.456;
    theStereoEvent.depth = 0.567;
    theStereoEvent.fluor = 678;
    theStereoEvent.pitch = 0.789;
    theStereoEvent.therm = 89;
    theStereoEvent.heading = 0.901;
    theStereoEvent.scatter = 987;
    theStereoEvent.latitude = -99.99;
    theStereoEvent.salinity = 0.876;
    theStereoEvent.altitude0 = 0.765;
    theStereoEvent.altitude1 = 0.654;
    theStereoEvent.longitude = -999.0;
    theStereoEvent.temperature = 0.543;
    theStereoEvent.conductivity = 0.432;
    theStereoEvent.imageState = 3;


    for(int cameraNumber = 0; cameraNumber < nOfAvtCameras; cameraNumber++)
        {
            stereoLcm.subscribeFunction(avtCameras[cameraNumber].lcmChannelName, &stereoCallback, &state);
        }
    // now subscribe to some control data
    stereoLcm.subscribeFunction("COMMAND_PARAMETERS", &recordingParameterCallback, &state);


    // wakeup message
    printf ("LOGGING_THREAD (thread %d) initialized \n", LOGGING_THREAD);
    printf ("LOGGING_THREAD File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);

    IniFile  *iniFile = new IniFile();
    int okINI = iniFile->openIni(flyIniFile);
    if(!okINI)
        {
            printf ("%s ini file does not exist...exiting!\n", flyIniFile);
            fflush (stdout);
            fflush (stderr);
            abort ();
        }
    else
        {
            leftCameraID = iniFile->readInt("GENERAL", "LEFT_CAMERA_ID",1);
            if((leftCameraID <= 0) || (leftCameraID > nOfAvtCameras))
                {
                    printf ("%s bad id for left camera...exiting!\n", flyIniFile);
                    fflush (stdout);
                    fflush (stderr);
                    abort ();
                }
            else
                {
                    leftCameraID -= 1;
                }
            if(0 == leftCameraID)
                {
                    rightCameraID = 1;
                }
            else
                {
                    rightCameraID = 0;
                }
            saveStereo =  (bool)iniFile->readInt("GENERAL","LOG_STEREO",true);
            if(saveStereo)
                {
                    stereoSaveRoot = iniFile->readString("GENERAL", "STEREO_SAVE_ROOT","./");
                    imgRoot  = iniFile->readString("GENERAL", "STEREO_IMG_ROOT","./");
                    struct stat sb;
                    if (stat(imgRoot, &sb) != 0)
                        {
                            /* path does not exist - create directory */
                            if (mkdir_p(imgRoot, ACCESSPERMS) < 0)
                                {
                                    exit(-1);
                                }
                        }

                }
            recordingPrefix = iniFile->readString("GENERAL","IMAGE_PREFIX","HAB");

            for(int cameraNumber = 0; cameraNumber < nOfAvtCameras; cameraNumber++)
                {
                    char cameraLabel[32];
                    snprintf(cameraLabel,31,"CAMERA_%01d",cameraNumber + 1);
                    avtCameras[cameraNumber].doNotUseInStereoLogging = (bool)iniFile->readInt(cameraLabel,"CAMERA_BAD",0);
                    avtCameras[cameraNumber].saveJPG = (bool)iniFile->readInt(cameraLabel,"STORE_JPG",0);
                    char *prefixString = iniFile->readString(cameraLabel,"JPG_PREFIX","UNK");
                    if(!strncmp(prefixString,"UNK",3))
                        {
                            char prefix[6];
                            snprintf(prefix,5,"CAM%01d",cameraNumber + 1);
                            avtCameras[cameraNumber].jpgPrefix = strdup(prefix);
                        }
                    else
                        {
                            avtCameras[cameraNumber].jpgPrefix = strdup(prefixString);
                        }
                    free(prefixString);
                }

            iniFile->closeIni();

        }


    // loop forever
    while(true)
        {
            stereoLcm.handle();
        }

}




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

#include <exiv2/exiv2.hpp>


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
char *imgRoot;
bool  saveStereo;
extern int   nOfAvtCameras;

long int leftTime;
long int rightTime;
bool  makeTenMinuteLogFiles;

bool stereoWriteResult;

int makeTimeString (double total_secs, char *str, char *prefix, char *suffix);

static int lastYear;
static int lastMonth;
static int lastDay;
static int lastHour;
static int lastTenMinute;
static int lastDayOfYear;

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

char    *altimeterStereoChannelName;
char    *ctdStereoChannelName;
char    *gpsStereoChannelName;
char    *fathometerStereoChannelName;
char    *attitudeStereoChannelName;


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

void gpsCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const marine_sensor::MarineSensorGPS_t  *myGPS, State *user )
{
    theStereoEvent.latitude = myGPS->latitude;
    theStereoEvent.longitude = myGPS->longitude;
}

void ctdCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const marine_sensor::MarineSensorCtd_t *myCTD, State *user )
{
    theStereoEvent.conductivity = myCTD->sea_water_electrical_conductivity;
    theStereoEvent.depth= myCTD->depth;
    theStereoEvent.temperature = myCTD->sea_water_temperature;
}

void fathometerCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const marine_sensor::MarineSensorFathometer_t  *myFathometer, State *user )
{
    theStereoEvent.fathometer = myFathometer->depth;
}

void altimeterCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const marine_sensor::marineSensorAltimeter_t  *myAlt, State *user )
{
    theStereoEvent.altitude0 = myAlt->altitude;
}
void attitudeCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const marine_sensor::MarineSensorAttitudeSensor_t  *myAtt, State *user )
{
    theStereoEvent.heading = myAtt->heading;
    theStereoEvent.pitch = myAtt->pitch;
    theStereoEvent.roll = myAtt->roll;
}

void stereoCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const image::image_t *image, State *user)
{


    bool needNewDirectory = false;
    bool needANewJPGDirectory = false;
    char theDataDir[512];
    char theJPGDataDir[512];
    char description[1024];
    char loggingRecord[2048];

    theStereoEvent.imageState = 0;
    image::image_t leftImageToPublish;
    image::image_t rightImageToPublish;
    int whichCamera = 0;
    if(channel == avtCameras[leftCameraID].lcmChannelName)
        {
            whichCamera = 0;
            leftTime = image->utime;
            workingImage = cv::Mat(image->height, image->width, CV_16UC1, (void *)image->data.data());


            leftImage = workingImage.clone();


            leftColorImage = cv::Mat(image->height, image->width, CV_16UC3);
            //leftNormalizedImage = cv::Mat(image->height, image->width, CV_16UC1);
            dst = cv::Mat(image->height, image->width, CV_8UC3);;
            //cv::normalize(leftImage,leftNormalizedImage,0, 255,cv::NORM_MINMAX);

            cv::cvtColor(leftImage,leftColorImage,cv::COLOR_BayerBG2BGR,0);
            //leftColorImage.convertTo(dst,CV_8UC3,0.003891051); // 1/257 to get the full range
            leftColorImage.convertTo(dst,CV_8UC3,0.0625); // 1/16 to get the full range

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
           /* cv::namedWindow("left");
         cv::imshow("left",leftColorImage);
         cv::waitKey(0); // Wait for any keystroke in the window

         cv::destroyWindow("windowName");*/

            //printf(" got a left\n");

        }
    else
        {
            whichCamera = 1;
            rightTime = image->utime;
            workingImage = cv::Mat(image->height, image->width, CV_16UC1, (void *)image->data.data());
            rightImage = workingImage.clone();


            rightColorImage = cv::Mat(image->height, image->width, CV_16UC3);
            //rightNormalizedImage = cv::Mat(image->height, image->width, CV_16UC1);
            dst = cv::Mat(image->height, image->width, CV_8UC3);;
            //cv::normalize(rightImage,rightNormalizedImage,0, 255,cv::NORM_MINMAX);

            cv::cvtColor(rightImage,rightColorImage,cv::COLOR_BayerBG2BGR,0);
            rightColorImage.convertTo(dst,CV_8UC3,0.0625); // 1/16 to get the full range

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

                            snprintf(theDataDir,511,"%s/%04d%02d%02d/%04d%02d%02d_%02d/%04d%02d%02d_%02d%02d",imgRoot,year,month+1,day,
                                     year,month+1,day,hour,
                                     year,month+1,day,hour,trialTenMinute );
                            struct stat sb;
                            if (stat(theDataDir, &sb) != 0)
                                {
                                    if(tenMinuteLogFile)
                                        {
                                            fclose(tenMinuteLogFile);
                                        }
                                    int directoryRetCode = mkdir_p(theDataDir,ACCESSPERMS);
                                    char dataFileName[512];
                                    if(makeTenMinuteLogFiles)
                                        {
                                            snprintf(dataFileName,511,"%s/%04d%02d%02d_%02d%02d.img",imgRoot,year,month+1,day,hour,trialTenMinute);
                                            tenMinuteLogFile = fopen(dataFileName,"wa");
                                        }
                                    int logLen = snprintf(loggingRecord,2047,"SYS %04d/%02d/%02d %02d:%02d:%02d.%03d MKDIR %s RETCODE %d",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,theDataDir,directoryRetCode);
                                    msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                                }



                        }
                    char imageTimeString[256];
                    std::vector<int> tags;
                    char imageName[768];

                    //  cv::waitKey(0);
                    //  cv::destroyWindow("left");
                    //  cv::destroyWindow("right");


                    snprintf(description,1024, "lat=%.6f,lon=%.6f,hdg=%.2f,pitch=%.2f,roll=%.2f,alt0=%.2f,alt1=%.2f,depth=%.2f,c=%.5f,t=%.2f,O2=%.5f,cdom=%d,fluor=%d,scatter=%d,therm=%d,ph1=%.4f,ph2=%.4f,fath=%.2f",theStereoEvent.latitude, theStereoEvent.longitude,theStereoEvent.heading,theStereoEvent.pitch,
                             theStereoEvent.roll, theStereoEvent.altitude0,theStereoEvent.altitude1,theStereoEvent.depth,theStereoEvent.conductivity, theStereoEvent.temperature, theStereoEvent.dO,
                             theStereoEvent.cdom, theStereoEvent.fluor, theStereoEvent.scatter, theStereoEvent.therm,theStereoEvent.ph1, theStereoEvent.ph2, theStereoEvent.fathometer);
                    //Now writing image to the file one strip at a timeif (description)


                    int numChars = makeTimeString(thePairTime,imageTimeString,recordingPrefix, "tif");
                    cv::Mat stereoImage = cv::Mat(image->height, image->width*2, CV_16UC1);
                    cv::hconcat(leftImage,rightImage,stereoImage);

                    /*double minVal;
                                double maxVal;
                                cv::Point minLoc;
                                cv::Point maxLoc;

                                cv::minMaxLoc( stereoImage, &minVal, &maxVal, &minLoc, &maxLoc );

                                std::cout << "min val: " << minVal << std::endl;
                                std::cout << "max val: " << maxVal << std::endl;
                                */
                    snprintf(imageName,767,"%s/%s",theDataDir,imageTimeString);
                    int theImageWidthBytes = image->width * 4;
                    int stereoTiffSuccess = 0;
                    TIFF *out= TIFFOpen(imageName, "w");
                    if(out)
                        {
                            TIFFSetField (out, TIFFTAG_IMAGEWIDTH, image->width*2);  // set the width of the image
                            TIFFSetField(out, TIFFTAG_IMAGELENGTH, image->height);    // set the height of the image
                            TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, 1);   // set number of channels per pixel
                            TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 16);    // set the size of the channels
                            TIFFSetField(out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);    // set the origin of the image.
                            //   Some other essential fields to set that you do not have to understand for now.
                            TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
                            TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
                            TIFFSetField(out, TIFFTAG_COMPRESSION, COMPRESSION_NONE);



                            tsize_t linebytes =  theImageWidthBytes;     // length in memory of one row of pixel in the image.
                            unsigned char *buf = NULL;        // buffer used to store the row of pixel information for writing to file


                            // We set the strip size of the file to be size of one row of pixels
                            TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(out, 1));
                            if (description)
                                {
                                    TIFFSetField (out, TIFFTAG_IMAGEDESCRIPTION, description);
                                }
                            else
                                {
                                    TIFFSetField (out, TIFFTAG_IMAGEDESCRIPTION, "");
                                }

                            buf = (unsigned char *)stereoImage.datastart;
                            for (size_t i=0; i< image->height ;i++)
                                {
                                    if (!TIFFWriteScanline (out, buf, i, 0))
                                        {
                                            printf ("error writing TIFF buffer to disk");
                                            break;
                                        }
                                    else
                                        buf += theImageWidthBytes;
                                }
                            (void) TIFFClose(out);
                            stereoTiffSuccess = 1;

                            char noCommaDescription[1024];
                            int len = snprintf(noCommaDescription,1024, "%s %.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.5f,%.2f,%.5f,%d,%d,%d,%d,%.4f,%.4f,%.2f",imageTimeString,theStereoEvent.latitude, theStereoEvent.longitude,theStereoEvent.heading,theStereoEvent.pitch,
                                theStereoEvent.roll, theStereoEvent.altitude0,theStereoEvent.altitude1,theStereoEvent.depth,theStereoEvent.salinity, theStereoEvent.temperature, theStereoEvent.dO,
                                theStereoEvent.cdom, theStereoEvent.fluor, theStereoEvent.scatter, theStereoEvent.therm,theStereoEvent.ph1, theStereoEvent.ph2,theStereoEvent.fathometer);
                            //msg_send(LOGGING_THREAD, ANY_THREAD, LOG,len,noCommaDescription);
                            if(tenMinuteLogFile && makeTenMinuteLogFiles)
                                {
                                    fprintf(tenMinuteLogFile,"%s\n",noCommaDescription);
                                    fflush(tenMinuteLogFile);
                                }

                        }
                    else
                        {
                            stereoTiffSuccess = 0;
                        }
                    stereoWriteResult = stereoTiffSuccess;

                    // here's the image description that was in the old code

                    /*char description[1024];
                             snprintf(description,1024, "lat=%.6f,lon=%.6f,hdg=%.2f,pitch=%.2f,roll=%.2f,alt0=%.2f,alt1=%.2f,depth=%.2f,c=%.5f,t=%.2f,O2=%.5f,cdom=%d,fluor=%d,scatter=%d,therm=%d,ph1=%.4f,ph2=%.4f,fath=%.2f",theStereoEvent.latitude, theStereoEvent.longitude,theStereoEvent.heading,theStereoEvent.pitch,
                                     theStereoEvent.roll, theStereoEvent.altitude0,theStereoEvent.altitude1,theStereoEvent.depth,theStereoEvent.conductivity, theStereoEvent.temperature, theStereoEvent.dO,
                                     theStereoEvent.cdom, theStereoEvent.fluor, theStereoEvent.scatter, theStereoEvent.therm,theStereoEvent.ph1, theStereoEvent.ph2, theStereoEvent.fathometer);*/

                   // stereoWriteResult = cv::imwrite(imageName,stereoImage,tags);

                }

            // 10 June 2015 jch replace conductivity with salinity

            //fprintf(logger->theLogDirFile,"%s,%s\n",unqualifiedFileName,noCommaDescription);
            leftTime = 0;
            rightTime = 0;

        }


    if(avtCameras[whichCamera].saveJPG)
        {
            Exiv2::XmpParser::initialize();
            ::atexit(Exiv2::XmpParser::terminate);
            double theImageTime;
            if(leftCameraID == whichCamera)
                {
                    theImageTime = leftTime/1000.0;
                }
            else
                {
                    theImageTime = rightTime/1000.0;
                }

            struct timeb  ftime_time;
            struct tm     gmtime_time;

            ftime_time.time      = (time_t) floor(theImageTime);
            ftime_time.millitm   = (unsigned short int) (fmod(theImageTime,1.0) * 1000.0);
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

            if(dayOfYear != lastJPEGS[whichCamera].lastDayOfYear)
                {
                    needANewJPGDirectory = true;
                    lastJPEGS[whichCamera].lastDayOfYear = dayOfYear;
                }
            else
                {
                    if(hour != lastJPEGS[whichCamera].lastHour)
                        {
                            needANewJPGDirectory = true;
                            lastJPEGS[whichCamera].lastHour = hour;
                        }
                    else
                        {
                            if(trialTenMinute != lastJPEGS[whichCamera].lastTenMinute)
                                {
                                    needANewJPGDirectory = true;
                                    lastJPEGS[whichCamera].lastTenMinute = trialTenMinute;
                                }
                        }
                }
            if(needANewJPGDirectory)
                {
                    snprintf(theJPGDataDir,511,"%s/%04d%02d%02d/%04d%02d%02d_%02d/%04d%02d%02d_%02d%02d",avtCameras[whichCamera].jpgSaveRoot,year,month+1,day,
                             year,month+1,day,hour,
                             year,month+1,day,hour,trialTenMinute );
                    struct stat sb;
                    if (stat(theJPGDataDir, &sb) != 0)
                        {
                            /* path does not exist - create directory */
                            int directoryRetCode = mkdir_p(theJPGDataDir,ACCESSPERMS);
                            if (directoryRetCode < 0)
                                {
                                    printf("error creating directory %s\n,theJPGDataDir");
                                }
                            else
                                {
                                    int logLen = snprintf(loggingRecord,2047,"SYS %04d/%02d/%02d %02d:%02d:%02d.%03d MKDIR %s RETCODE %d",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,theJPGDataDir,directoryRetCode);
                                    msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                                }
                        }

                }
            char jpgPrefix[512];
            char jpgImageName[768];
            char jpgImageTimeString[512];
            snprintf(jpgPrefix,511,"%s%s",avtCameras[whichCamera].jpgPrefix,recordingPrefix);
            int numChars = makeTimeString(theImageTime,jpgImageTimeString,jpgPrefix, "jpg");
            snprintf(jpgImageName,767,"%s/%s",theJPGDataDir,jpgImageTimeString);
            bool jpgWriteResult
                    ;if(0 == whichCamera)
                {
                    jpgWriteResult = cv::imwrite(jpgImageName,dst);
                    /*cv::namedWindow("left");
                                   cv::imshow("lefts",leftColorImage);
                                     cv::waitKey(0);
                                    cv::destroyWindow("lefts");*/
                }
            else
                {
                    jpgWriteResult = cv::imwrite(jpgImageName,dst);
                }
            if(jpgWriteResult)
                {
                    Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(jpgImageName);
                    image->readMetadata();
                    Exiv2::ExifData &exifData = image->exifData();
                    exifData["Exif.Image.ImageDescription"] = description;
                    image->writeMetadata();
                }
        }



}


            // int logLen = snprintf(loggingRecord,2047,"IMG %04d/%02d/%02d %02d:%02d:%02d.%03d IMWRITE %s %d %0.2f %0.2f",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,fileName,avtCameras[cameraNumber].actualSettings.cameraSynced,
            //msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
            // cv::imwrite(imageTimeString,stereoImage);






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
    theStereoEvent.dO = 99.99;
    theStereoEvent.ph1 = 99.99;
    theStereoEvent.ph2 = 99.99;
    theStereoEvent.cdom = 99.99;
    theStereoEvent.roll = 0.456;
    theStereoEvent.depth = 0.567;
    theStereoEvent.fluor = 99.99;
    theStereoEvent.pitch = 0.789;
    theStereoEvent.therm = 99.99;
    theStereoEvent.heading = 0.901;
    theStereoEvent.scatter = 99.99;
    theStereoEvent.latitude = -99.99;
    theStereoEvent.salinity = 0.876;
    theStereoEvent.altitude0 = 0.765;
    theStereoEvent.altitude1 = 99.99;
    theStereoEvent.longitude = -999.0;
    theStereoEvent.temperature = 0.543;
    theStereoEvent.conductivity = 0.432;
    theStereoEvent.imageState = 3;


    makeTenMinuteLogFiles = false;

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
            altimeterStereoChannelName = iniFile->readString("ALTIMETER","CHANNEL_NAME", "ALTIMETER");
            ctdStereoChannelName = iniFile->readString("CTD","CHANNEL_NAME","CTD");
            gpsStereoChannelName = iniFile->readString("GPS","CHANNEL_NAME","GPS");
            fathometerStereoChannelName = iniFile->readString("FATHOMETER","CHANNEL_NAME","FATHOMETER");
            attitudeStereoChannelName = iniFile->readString("MICROSTRAIN","CHANNEL_NAME","ATTITUDE");
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
            makeTenMinuteLogFiles = (bool)iniFile->readInt("GENERAL","MAKE_TEN_MINUTE_LOG_FILES",0);


            recordingPrefix = iniFile->readString("GENERAL","IMAGE_PREFIX","HAB");

            for(int cameraNumber = 0; cameraNumber < nOfAvtCameras; cameraNumber++)
                {
                    char cameraLabel[32];
                    snprintf(cameraLabel,31,"CAMERA_%01d",cameraNumber + 1);
                    avtCameras[cameraNumber].doNotUseInStereoLogging = (bool)iniFile->readInt(cameraLabel,"CAMERA_BAD",0);
                    avtCameras[cameraNumber].saveJPG = (bool)iniFile->readInt(cameraLabel,"STORE_JPG",0);

                    char *jpgSaveRoot = iniFile->readString(cameraLabel,"JPEG_SAVE_ROOT","./jpg");
                    avtCameras[cameraNumber].jpgSaveRoot = strdup(jpgSaveRoot);
                    free(jpgSaveRoot);
                    if(avtCameras[cameraNumber].saveJPG)
                        {
                            struct stat sb;
                            if (stat(avtCameras[cameraNumber].jpgSaveRoot, &sb) != 0)
                                {
                                    /* path does not exist - create directory */
                                    if (mkdir_p(avtCameras[cameraNumber].jpgSaveRoot, ACCESSPERMS) < 0)
                                        {
                                            exit(-1);
                                        }
                                }
                        }

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
    for(int cameraNumber = 0; cameraNumber < nOfAvtCameras; cameraNumber++)
        {
            stereoLcm.subscribeFunction(avtCameras[cameraNumber].lcmChannelName, &stereoCallback, &state);
        }
    // now subscribe to some control data
    stereoLcm.subscribeFunction("COMMAND_PARAMETERS", &recordingParameterCallback, &state);
    stereoLcm.subscribeFunction(gpsStereoChannelName, gpsCallback, &state);
    stereoLcm.subscribeFunction(ctdStereoChannelName, ctdCallback, &state);
    stereoLcm.subscribeFunction(altimeterStereoChannelName, altimeterCallback, &state);
    stereoLcm.subscribeFunction(fathometerStereoChannelName, fathometerCallback, &state);
    stereoLcm.subscribeFunction(attitudeStereoChannelName, attitudeCallback, &state);



    // loop forever
    while(true)
        {
            stereoLcm.handle();
        }

}




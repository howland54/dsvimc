/*=============================================================================
  Copyright (C) 2013 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.cpp

  Description: The frame observer that is used for notifications from VimbaCPP
               regarding the arrival of a newly acquired frame.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

/* note original AVT code has been modified to fit camera requirements of habcam, May 2023 jch*/

#include <iostream>
#include <iomanip>
#include <time.h>
#include <sys/time.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#include <tiff.h>
#include "FrameObserver.h"



extern lcm::LCM myLcm;


extern avtCameraT  avtCameras[MAX_N_OF_CAMERAS];


template <class T>
void DeleteArray(T *pArray)
{
   delete [] pArray;
}
int makeFileName(char *filename, struct timeval tv,char *prefix)
{
    struct tm *gmtTime = gmtime(&tv.tv_sec);
    int milliseconds = (float)tv.tv_usec/1000.0;
    return sprintf(filename,"%s.%04d%02d%02d.%02d%02d%03d.%03d.tif",prefix,gmtTime->tm_year+1900, gmtTime->tm_mon+1,gmtTime->tm_mday,
                    gmtTime->tm_hour, gmtTime->tm_min, gmtTime->tm_sec, milliseconds);
}


FrameObserver::FrameObserver( AVT::VmbAPI::CameraPtr pCamera, int theCameraNumber):   IFrameObserver( pCamera )
{

   myCameraPtr = pCamera;
   imageCount = 0;
   cameraNumber = theCameraNumber;

   // set up some time variables so that new directories, etc will be created upon startup

   struct timeval tv;
   gettimeofday(&tv,NULL);
   tv.tv_sec -= 86401.0;
   struct tm *gmtTime = gmtime(&tv.tv_sec);
   lastYear = gmtTime->tm_year+1900;
   lastMonth = gmtTime->tm_mon;
   lastDay = gmtTime->tm_mday;
   lastHour = gmtTime->tm_hour;
   lastMinute = gmtTime->tm_min;
   lastDayOfYear = gmtTime->tm_yday;

}




void FrameObserver::FrameReceived( const AVT::VmbAPI::FramePtr pFrame )
{
   VmbFrameStatusType eReceiveStatus;
   bool anyError = false;
   bool needNewDirectory = false;
   char loggingRecord[2048];
   cv::Mat  imageCopy;
   if( VmbErrorSuccess == pFrame->GetReceiveStatus( eReceiveStatus ) )
      {
         struct timeval tv;
         gettimeofday(&tv,NULL);
         AVT::VmbAPI::FramePtr myFrame = pFrame;
         VmbUchar_t *pBuffer;
         VmbUint64_t lTime;
         pFrame->GetTimestamp(lTime);
         rov_time_t imageTime = (double)lTime/avtCameras[cameraNumber].cameraFrequency;
         if(SYNC_SLAVE != avtCameras[cameraNumber].actualSettings.cameraSynced)
            {
               imageTime = (double)lTime/avtCameras[cameraNumber].cameraFrequency +  avtCameras[cameraNumber].startCameraTime;
            }

         //rov_time_t imageTime = rov_get_time();
         //rov_time_t imageTime = (double)myTime;
         //printf(" image time:  %0.3f\n",imageTime);VmbUint16_t
         VmbErrorType err = SP_ACCESS( myFrame )->GetImage( pBuffer );
         if ( VmbErrorSuccess == err )
            {
               image::image_t imageToPublish;
               VmbUint32_t scratchVar;

               err = pFrame->GetWidth(scratchVar);
               if(VmbErrorSuccess == err)
                  {
                     imageToPublish.width = (int32_t)scratchVar;
                  }
               else
                  {
                     anyError = true;
                  }
               err = pFrame->GetHeight(scratchVar);
               if(VmbErrorSuccess == err)
                  {
                     imageToPublish.height = (int32_t)scratchVar;
                  }
               else
                  {
                     anyError = true;
                  }
               err = pFrame->GetImageSize(scratchVar);
               if(VmbErrorSuccess == err)
                  {
                     imageToPublish.size = (int32_t)scratchVar;
                  }
               else
                  {
                     anyError = true;
                  }

               // now promote the image



               if(!anyError)
                  {
                       /* VmbUint16_t *place;
                        VmbUint16_t value;
                        place = (VmbUint16_t *)pBuffer;
                        int pixelShift = 0;
                        for(int pixelNum = 0; pixelNum < imageToPublish.width * imageToPublish.height; pixelNum++)
                            {
                                value = *(pBuffer + pixelShift * sizeof(VmbUint16_t)) << 4;
                                *place = value;
                                place ++;
                                pixelShift++;
                            }
*/

                       //cv::Mat cvMat = cv::Mat(imageToPublish.height, imageToPublish.width, CV_8UC1, pBuffer ); // below line added 5 May 2023 jch
                       cv::Mat cvMat = cv::Mat(imageToPublish.height, imageToPublish.width, CV_16UC1, pBuffer);
                       imageCopy = cvMat.clone();
                       double minVal;
                                                       /*double maxVal;
                                                       cv::Point minLoc;
                                                       cv::Point maxLoc;

                                                       cv::minMaxLoc( imageCopy, &minVal, &maxVal, &minLoc, &maxLoc );

                                                       std::cout << "min val: " << minVal << std::endl;
                                                       std::cout << "max val: " << maxVal << std::endl;*/

                       /*cv::namedWindow("left single");
                    cv::imshow("left single",imageCopy);
                    cv::waitKey(0); // Wait for any keystroke in the window

                    cv::destroyWindow("left single");*/

                     if(cvMat.empty())
                        {
                           printf("empty image!\n");
                           return;
                        }

                     if( avtCameras[cameraNumber].saveImages)
                        {
                           char imageName[768];
                           char fileName[128];

                           struct timeb  ftime_time;
                           struct tm     gmtime_time;

                           ftime_time.time      = (time_t) floor(imageTime);
                           ftime_time.millitm   = (unsigned short int) (fmod(imageTime,1.0) * 1000.0);
                           ftime_time.timezone  =  0;
                           ftime_time.dstflag   =  0;

                           gmtime_r(&ftime_time.time, &gmtime_time);
                           int year = gmtime_time.tm_year+1900;
                           int month = gmtime_time.tm_mon;
                           int day = gmtime_time.tm_mday;
                           int hour = gmtime_time.tm_hour;
                           int minute = gmtime_time.tm_min;
                           int dayOfYear = gmtime_time.tm_yday;

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
                              }
                           if(needNewDirectory)
                              {
                                 snprintf(theDataDir,511,"%s/%04d%02d%02d/%04d%02d%02d_%02d00",avtCameras[cameraNumber].saveDirectoryRoot,year,month+1,day,year,month+1,day,hour);
                                 int directoryRetCode = mkdir_p(theDataDir,ACCESSPERMS);
                                 int logLen = snprintf(loggingRecord,2047,"SYS %04d/%02d/%02d %02d:%02d:%02d.%03d MKDIR %s RETCODE %d",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,theDataDir,directoryRetCode);
                                 msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                              }


                           std::vector<int> tags = {TIFFTAG_COMPRESSION, COMPRESSION_NONE};
                           makeFileName(fileName,tv,avtCameras[cameraNumber].filenamePrefix);

                           snprintf(imageName,767,"%s/%s",theDataDir,fileName);
                           cv::imwrite(imageName,16.0 * imageCopy,tags);
                           int logLen = snprintf(loggingRecord,2047,"IMG %04d/%02d/%02d %02d:%02d:%02d.%03d IMWRITE %s %d %0.2f %0.2f",year,month+1,day,hour,minute,gmtime_time.tm_sec,ftime_time.millitm,fileName,avtCameras[cameraNumber].actualSettings.cameraSynced,
                                                 avtCameras[cameraNumber].actualSettings.theGain, avtCameras[cameraNumber].actualSettings.theShutter);
                           msg_send(LOGGING_THREAD, ANY_THREAD, LOG,logLen,loggingRecord);
                        }
                     //imageToPublish.nmetadata = 0;
                     imageToPublish.pixelformat = image::image_t::PIXEL_FORMAT_GRAY;
                     imageToPublish.data.resize((uint32_t)imageToPublish.size);
                     imageToPublish.utime =(long int)( 1000.0 * imageTime);

                     std::copy(pBuffer, pBuffer + imageToPublish.size, imageToPublish.data.begin());
                     char topicName[32];
                     snprintf(topicName,32,"Vim%0d",cameraNumber);
                     std::string theTopic(topicName);
                    // printf(" the size:  %d\n",sizeof(imageToPublish));
                     //avtCameras[cameraNumber].lcmChannelName
                     int success = myLcm.publish(avtCameras[cameraNumber].lcmChannelName,&imageToPublish);
                     //printf(" camera topic %s success = %d\n",topicName,success);
                     printf(".");
                     fflush(stdout);
                  }

               imageCount++;
            }
         myCameraPtr->QueueFrame( pFrame );
      }
   else
      {
           printf("could not get recieve status from incoming frame!\n");
      }
}








/* ----------------------------------------------------------------------

   code for talkinng to Vimba cameras from Allied Vision
   
   Modification History:
   DATE        AUTHOR   COMMENT
   13-Apr-2017  jch      creation, derive from jason crossbow prompt thread
   31 Jan 2023  jch      previous changes made it work with Fly cameras, then Allied Vision Vimba.  Now modify for Habcam
   ---------------------------------------------------------------------- */

/* ansii c headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <unistd.h>

#include <lcm/lcm-cpp.hpp>



/* local includes */
#include "../../dsvimlib/include/time_util.h"
#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageTalk.h"		/* jasontalk protocol and structures */
#include "../../dsvimlib/include/msg_util.h"		/* utility functions for messaging */
#include "../../dsvimlib/include/launch_timer.h"

#include "AVTAttribute.h"

#include "vimcBusThread.h"

#include "vimcThread.h"

/* posix header files */
#define  POSIX_SOURCE 1
#include <pthread.h>

#include "vimc.h"


extern avtCameraT  avtCameras[MAX_N_OF_CAMERAS];
extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;
extern lcm::LCM myLcm;


extern VimbaSystem          *vSystem;

//extern FlyCapture2::BusManager busMgr;


VmbErrorType configureCamera(avtCameraT *mycamera, AVT::VmbAPI::CameraPtr theAVTCamera)
{
   VmbErrorType res = VmbErrorNotFound;
   AVT::VmbAPI::CameraPtr   theCamera = theAVTCamera;
   AVTAttribute avtAttribute;

   // Set the GeV packet size to the highest possible value
   // (In this example we do not test whether this cam actually is a GigE cam)
   AVT::VmbAPI::FeaturePtr pCommandFeature;
   if( VmbErrorSuccess == SP_ACCESS( theCamera )->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ) )
      {
         if( VmbErrorSuccess == SP_ACCESS( pCommandFeature )->RunCommand() )
            {
               bool bIsCommandDone = false;
               int iterationCount = 0;
               do
                  {
                     if(iterationCount > MAX_ITERATION_COUNT)
                         {
                             break;
                         }
                     if( VmbErrorSuccess != SP_ACCESS( pCommandFeature )->IsCommandDone( bIsCommandDone ) )
                        {
                           break;
                        }
                     iterationCount++;
                  } while( false == bIsCommandDone );
            }
      }
   // if we used a config file, apply it
   if(strcmp(mycamera->xmlFileName,NO_INITIAL_FILE_NAME))
      {
         VmbFeaturePersistSettings_t settingsStruct;
         settingsStruct.loggingLevel = 4;
         settingsStruct.maxIterations = 5;
         settingsStruct.persistType = VmbFeaturePersistNoLUT;
         std::string xmlFilename = mycamera->xmlFileName;
         res = theCamera->LoadCameraSettings(xmlFilename, &settingsStruct);
         if(VmbErrorSuccess != res)
            {
               std::stringstream ss;
               ss.str( "" );
               ss << "Could not load camera settings from file '" << xmlFilename << "' [error code: " << res << "]";
               std::cout << ss.str() << std::endl;
            }
         else
            {
               std::stringstream ss;
               ss.str( "" );
               ss << "loaded camera settings from file " << xmlFilename;
               std::cout << ss.str() << std::endl;
            }
      }

   // now set the trigger so that when we query it later, we get the right one
   if( VmbErrorSuccess == SP_ACCESS( theCamera )->GetFeatureByName( "TriggerSelector", pCommandFeature ) )
      {
         res = SP_ACCESS( pCommandFeature )->SetValue ( "FrameStart" );
         if( VmbErrorSuccess != res )
            {
               std::stringstream ss;
               ss.str( "" );
               ss << "Could not set trigger selector!\n ";
               std::cout << ss.str() << std::endl;
            }
      }

   // now check for a special feature file

   if(strcmp(mycamera->parameterFileName,NO_INITIAL_FILE_NAME))
      {
         FILE *parameterFD = fopen (mycamera->parameterFileName, "r");
         if(NULL == parameterFD)
            {
               std::stringstream ss;
               ss.str( "" );
               ss << "Could not load camera settings from file '" << mycamera->parameterFileName << "' [error code: " << res << "]";
               std::cout << ss.str() << std::endl;
            }
         else
            {
               int file_position = fseek (parameterFD, 0, SEEK_SET);
               char myLine[MAX_CHARACTER_COUNT], *ch;
               char parameter[MAX_CHARACTER_COUNT];
               char parameterValue[MAX_CHARACTER_COUNT];

               while (!feof (parameterFD))
                  {
                     ch = fgets (&(myLine[0]), MAX_CHARACTER_COUNT - 1, parameterFD);
                     if (ch)
                        {
                           if("#" == ch)
                              {
                                 continue;
                              }
                           int items = sscanf(&(myLine[0]),"%s %s",&parameter, &parameterValue);
                           if(2 == items)
                              {
                                 std::string parameterString(parameter);
                                 std::string parameterValueString(parameterValue);
                                 int res =  avtAttribute.setAttribute(theCamera,parameterString,parameterValueString);
                              }

                        }
                  }
               fclose(parameterFD);
            }

      }

   // now check on a few key features
   VmbInt64_t lFreq;
   if( VmbErrorSuccess == SP_ACCESS( theCamera )->GetFeatureByName( "GevTimestampTickFrequency", pCommandFeature ) )
      {
         pCommandFeature->GetValue(lFreq);
         mycamera->cameraFrequency = lFreq;
      }


   if( VmbErrorSuccess == SP_ACCESS( theCamera )->GetFeatureByName( "GevTimestampControlLatch", pCommandFeature ) )
      {
         if( VmbErrorSuccess == SP_ACCESS( pCommandFeature )->RunCommand() )
            {
               bool bIsCommandDone = false;
               do
                  {
                     if( VmbErrorSuccess != SP_ACCESS( pCommandFeature )->IsCommandDone( bIsCommandDone ) )
                        {
                           break;
                        }
                  } while( false == bIsCommandDone );
            }
      }
   AVT::VmbAPI::FeaturePtr pTimestampFeature;

   // now get the time tick
   rov_time_t timeNow = rov_get_time();

   res = SP_ACCESS( theCamera )->GetFeatureByName( "GevTimestampValue", pTimestampFeature );
   if( VmbErrorSuccess == res )
      {
         VmbInt64_t actualTime;
         res = SP_ACCESS( pTimestampFeature )->GetValue( actualTime );
         if( VmbErrorSuccess == res )
            {
               // if we got to here, it means that at time startCameraTime,
               // the camear had been running for startSeconds (see below) seconds
               // that meansthat onsubsequent times we get a time tick, we take the tick and add it to the
               // startCameraTime minus startSeconds;
               double timeDelta = (double)actualTime/(double)lFreq;
               mycamera->startCameraTime = timeNow - timeDelta;
               printf("cameradelta %.2f start time %.2f\n",timeDelta,mycamera->startCameraTime);
            }
      }
   return res;
}


VmbErrorType setGainAuto(int theCameraNumber, AVT::VmbAPI::CameraPtr theAVTCamera,double low, double high)
{

   if(!avtCameras[theCameraNumber].plugged)
      {
         return VmbErrorDeviceNotOpen;
      }

   AVT::VmbAPI::FeaturePtr pGainFeature;
   VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "GainAuto", pGainFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( pGainFeature )->SetValue ( "Continuous" );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "GainAutoMin", pGainFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }

   res = SP_ACCESS( pGainFeature )->SetValue ( low);
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "GainAutoMax", pGainFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }

   res = SP_ACCESS( pGainFeature )->SetValue ( high);
   return res;
 }

VmbErrorType setBinning(int theCameraNumber, AVT::VmbAPI::CameraPtr theAVTCamera,bool setBinTrue)
{
   if(!avtCameras[theCameraNumber].plugged)
      {
         return VmbErrorDeviceNotOpen;
      }

   AVT::VmbAPI::FeaturePtr pHorizontalBinningFeature;
   AVT::VmbAPI::FeaturePtr pVerticalBinningFeature;
   AVT::VmbAPI::FeaturePtr pWidthFeature;
   AVT::VmbAPI::FeaturePtr pHeightFeature;
   AVT::VmbAPI::FeaturePtr pWidthMaxFeature;
   AVT::VmbAPI::FeaturePtr pHeightMaxFeature;



   VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "BinningHorizontal", pHorizontalBinningFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "BinningVertical", pVerticalBinningFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "Width", pWidthFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "Height", pHeightFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "SensorWidth", pWidthMaxFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   VmbInt64_t theWidthMax;
   res = pWidthMaxFeature->GetValue(theWidthMax);

   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "SensorHeight", pHeightMaxFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   VmbInt64_t theHeightMax;
   res = pHeightMaxFeature->GetValue(theHeightMax);


   if(setBinTrue)
      {
         res = SP_ACCESS( pHorizontalBinningFeature )->SetValue ( 2 );
         res = SP_ACCESS( pVerticalBinningFeature )->SetValue ( 2 );
         res = SP_ACCESS( pWidthFeature )->SetValue ( theWidthMax/2 );
         res = SP_ACCESS( pHeightFeature )->SetValue ( theHeightMax/2 );

      }
   else
      {
         res = SP_ACCESS( pHorizontalBinningFeature )->SetValue ( 1 );
         res = SP_ACCESS( pVerticalBinningFeature )->SetValue ( 1 );
         res = SP_ACCESS( pWidthFeature )->SetValue ( theWidthMax );
         res = SP_ACCESS( pHeightFeature )->SetValue ( theHeightMax );

      }

   return res;

}

VmbErrorType setGainManual(int theCameraNumber, AVT::VmbAPI::CameraPtr theAVTCamera)
{

   if(!avtCameras[theCameraNumber].plugged)
      {
         return VmbErrorDeviceNotOpen;
      }

   AVT::VmbAPI::FeaturePtr pGainFeature;
   VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "GainAuto", pGainFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( pGainFeature )->SetValue ( "Off" );

   return res;
}

VmbErrorType setGain(int theCameraNumber, AVT::VmbAPI::CameraPtr theAVTCamera,double theGain)
{
   avtCameras[theCameraNumber].desiredSettings.theGain = theGain;
   if(!avtCameras[theCameraNumber].plugged)
      {
         return VmbErrorDeviceNotOpen;
      }
   AVT::VmbAPI::FeaturePtr pGainFeature;
   VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "GainRaw", pGainFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   // make sure the gain is an integral value (albeit contained in a double)
   int theRoundGain = (int)round(theGain);
   res = SP_ACCESS( pGainFeature )->SetValue (theRoundGain );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   long long theGainActual;
   res = SP_ACCESS( pGainFeature )->GetValue(theGainActual);
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   avtCameras[theCameraNumber].actualSettings.theGain = (double)theGainActual;
   return res;
}

VmbErrorType setExposureAuto(int theCameraNumber, AVT::VmbAPI::CameraPtr theAVTCamera,double minimum, double maximum)
{
   if(!avtCameras[theCameraNumber].plugged)
      {
         return VmbErrorDeviceNotOpen;
      }
   AVT::VmbAPI::FeaturePtr pExposureFeature;
   VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "ExposureAuto", pExposureFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( pExposureFeature )->SetValue ( "Continuous" );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "ExposureAutoMin", pExposureFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( pExposureFeature )->SetValue ( (int)minimum);
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "ExposureAutoMax", pExposureFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( pExposureFeature )->SetValue ( (int)maximum);
   return res;

}

VmbErrorType setExposureManual(int theCameraNumber, AVT::VmbAPI::CameraPtr theAVTCamera)
{
   if(!avtCameras[theCameraNumber].plugged)
      {
         return VmbErrorDeviceNotOpen;
      }
   AVT::VmbAPI::FeaturePtr pExposureFeature;
   VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "ExposureAuto", pExposureFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( pExposureFeature )->SetValue ( "Off" );
   return res;
}

VmbErrorType setExposure(int theCameraNumber, AVT::VmbAPI::CameraPtr theAVTCamera,double exposureValue)
{
   avtCameras[theCameraNumber].desiredSettings.theShutter = exposureValue;
   if(!avtCameras[theCameraNumber].plugged)
      {
         return VmbErrorDeviceNotOpen;
      }

   AVT::VmbAPI::FeaturePtr pExposureFeature;
   VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "ExposureTimeAbs", pExposureFeature );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   res = SP_ACCESS( pExposureFeature )->SetValue ( exposureValue );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   double actualExposure;
   res = SP_ACCESS( pExposureFeature )->GetValue ( actualExposure );
   if( VmbErrorSuccess != res )
      {
         return res;
      }
   avtCameras[theCameraNumber].actualSettings.theShutter = actualExposure;
   std::cout << "Actual exposure set to " << actualExposure << " microseconds" << std::endl;
   return res;
}


/* ----------------------------------------------------------------------
   vimc Thread


   Modification History:
   DATE        AUTHOR   COMMENT
   13-Apr-217  jch      creation, derive from jason crossbow prompt thread

---------------------------------------------------------------------- */
void *vimcThread (void *threadNumber)
{

   msg_hdr_t hdr = { 0 };
   unsigned char data[MSG_DATA_LEN_MAX] = { 0 };
   long int myThreadNumber = (long int)threadNumber;
   using namespace AVT::VmbAPI;
   CameraPtr theAVTCamera;
   FrameObserver *theObserver;
   e_cameraParameterT parameterQueryTypes[MAX_N_OF_PARAMETER_QUERIES];
   int currentQueryType = CAMERA_SHUTTER;
   parameterQueryTypes[CAMERA_SHUTTER] = CAMERA_SHUTTER;
   parameterQueryTypes[CAMERA_GAIN] = CAMERA_GAIN;

   // wakeup message
   printf ("VIMBA_CAMERA (thread %d) initialized \n", (int)myThreadNumber);
   printf ("VIMBA_CAMERA File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);

   int msg_success = msg_queue_new(myThreadNumber, "fly thread");

   if(msg_success != MSG_OK)
      {
         fprintf(stderr, "vimba camera thread: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
         fflush (stdout);
         fflush (stderr);
         abort ();
      }
   int theCameraNumber = myThreadNumber - FLY_THREAD_BASE;
   avtCameras[theCameraNumber].sendingThread = myThreadNumber;
   avtCameras[theCameraNumber].subThread = myThreadNumber + 100;
   avtCameras[theCameraNumber].actualSettings.cameraSynced = SYNC_UNKNOWN;
   VmbErrorType res = VmbErrorNotFound;


   //pthread_create (&(avtCameras[theCameraNumber].cameraSubThread), &DEFAULT_ROV_THREAD_ATTR, flyBlockThread, (void *)&(avtCameras[theCameraNumber]));
   launched_timer_data_t *queryTimer;
   launched_timer_data_t *repetitionTimer;
   repetitionTimer = launch_timer_new(CAMERA_TRIGGER_PERIOD,0,myThreadNumber,STILL);
   // loop forever
   while (1)
      {

         int msg_get_success = msg_get(myThreadNumber,&hdr, data, MSG_DATA_LEN_MAX);
         if(msg_get_success != MSG_OK)
            {
               fprintf(stderr, "fly thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
            }
         else
            {
               // process new message
               switch (hdr.type)
                  {

                  case SQT:
                     {
                        queryTimer = launch_timer_new(CAMERA_QUERY_PERIOD, -1, myThreadNumber,  CAMERA_QUERY);
                        break;
                     }
                  case CAMA:
                     {
                        printf(" camera added to bus, sn: %s\n", data);
                        avtCameras[theCameraNumber].plugged = true;
                        res = vSystem->OpenCameraByID( (char *)data, VmbAccessModeFull, theAVTCamera );
                        std::cout << " camera " << theCameraNumber << ",result of open: " << res << std::endl;
                        if(VmbErrorSuccess == res)
                           {

                              res = configureCamera(&(avtCameras[theCameraNumber]),theAVTCamera);
                              printf(" return from configure camera: %d\n",res);
                           }
                        if(VmbErrorSuccess == res)
                           {
                              theObserver = new FrameObserver( theAVTCamera,theCameraNumber );
                              res = theAVTCamera->StartContinuousImageAcquisition( NUM_FRAMES, IFrameObserverPtr( theObserver ));
                              if(VmbErrorSuccess == res)
                                 {
                                    msg_send(myThreadNumber,myThreadNumber,SQT,0,NULL);
                                 }
                           }
                        break;
                     }
                  case CAMR:
                     {
                        printf(" camera removed from bus, sn: %ld\n",(long int) data);
                        avtCameras[theCameraNumber].plugged = false;
                        res = theAVTCamera->StopContinuousImageAcquisition();
                        res = theAVTCamera->Close();

                        launch_timer_disable(queryTimer);
                        free(queryTimer);
                        break;
                     }

                  case WCG:
                     {
                        double *newGain = (double *)data;
                        res = setGainManual(theCameraNumber,theAVTCamera);
                        if(VmbErrorSuccess != res)
                           {
                              printf(" could not set camera %d gain to manual\n", theCameraNumber);
                              break;
                           }

                        res = setGain(theCameraNumber,theAVTCamera,*newGain);
                        if(VmbErrorSuccess != res)
                           {
                              printf(" could not set camera %d gain\n",theCameraNumber);
                              break;
                           }


                        break;
                     }

                  case WCE:
                     {
                        double *newExposure = (double *)data;
                        res = setExposureManual(theCameraNumber,theAVTCamera);
                        if(VmbErrorSuccess != res)
                           {
                              printf(" could not set camera %d exposure to manual\n", theCameraNumber);
                              break;
                           }

                        res = setExposure(theCameraNumber,theAVTCamera,*newExposure);
                        if(VmbErrorSuccess != res)
                           {
                              printf(" could not set camera %d exposure\n",theCameraNumber);
                              break;
                           }
                        break;
                     }
                  case WBIN:
                     {
                        double *binOrNot = (double *)data;
                        res = theAVTCamera->StopContinuousImageAcquisition();

                        if(*binOrNot > 0.5)
                           {
                              res = setBinning(theCameraNumber,theAVTCamera,true);
                           }
                        else
                           {
                              res = setBinning(theCameraNumber,theAVTCamera,false);
                           }
                        theObserver = new FrameObserver( theAVTCamera,theCameraNumber);
                        res = theAVTCamera->StartContinuousImageAcquisition( NUM_FRAMES, IFrameObserverPtr( theObserver ));
                        break;
                     }
                  case WCAG:
                     {

                        double *parameters = (double *) data;
                        double gainOrNot = parameters[0];
                        double theMin = parameters[1];
                        double theMax = parameters[2];
                        if(gainOrNot > 0.1)
                           {
                              setGainAuto(theCameraNumber,theAVTCamera,theMin, theMax);
                           }
                        else
                           {
                              setGainManual(theCameraNumber,theAVTCamera);
                           }

                        break;
                     }
                  case WCAE:
                     {

                        double *parameters = (double *) data;
                        double exposureOrNot = parameters[0];
                        double theMin = parameters[1];
                        double theMax = parameters[2];
                        if(exposureOrNot > 0.1)
                           {
                              setExposureAuto(theCameraNumber,theAVTCamera,theMin, theMax);
                           }
                        else
                           {
                              setExposureManual(theCameraNumber,theAVTCamera);
                           }

                        break;
                     }
                  case WREP:
                     {
                        double repRate = *(double *) data;
                        launch_timer_enable(repetitionTimer,repRate,-1,myThreadNumber,STILL);

                        break;
                     }
                  case WRES:
                     {
                        double repRate = *(double *) data;
                        launch_timer_disable(repetitionTimer);

                        break;
                     }
                  case STILL:
                     {
                        //std::cout << "about to  trigger start\n";
                        FeaturePtr pCommandFeature;
                        if( VmbErrorSuccess == SP_ACCESS( theAVTCamera )->GetFeatureByName( "TriggerSoftware", pCommandFeature ) )
                           {

                              VmbError_t res = SP_ACCESS( pCommandFeature )->RunCommand();
                              //std::cout << "triggered it\n";
                              if(res == VmbErrorSuccess)
                                 //if( VmbErrorSuccess == SP_ACCESS( pCommandFeature )->RunCommand() )
                                 {

                                    bool bIsCommandDone = false;
                                    do
                                       {
                                          if( VmbErrorSuccess != SP_ACCESS( pCommandFeature )->IsCommandDone( bIsCommandDone ) )
                                             {
                                                break;
                                             }
                                       } while( false == bIsCommandDone );
                                 }
                              else
                                 {
                                    std::cout << "failed trigger\n";
                                    std::cout << "error code: " << AVT::VmbAPI::Examples::ErrorCodeToMessage((VmbErrorType)res) << std::endl;

                                 }
                           }
                        break;
                     }
                  case CAMERA_QUERY:
                     {
                        //printf(" camera query\n");
                        image::image_parameter_t imageParameter;
                        if(MAX_N_OF_PARAMETER_QUERIES == currentQueryType)
                           {
                              currentQueryType = CAMERA_SHUTTER;
                           }

                        if(CAMERA_SHUTTER == parameterQueryTypes[currentQueryType])
                           {
                              //printf(" exposure\n");
                              imageParameter.key = "EXPOSURE";
                              AVT::VmbAPI::FeaturePtr pExposureFeature;
                              VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "ExposureTimeAbs", pExposureFeature );
                              if( VmbErrorSuccess == res )
                                 {
                                    double theExposureActual;
                                    res = SP_ACCESS( pExposureFeature )->GetValue(theExposureActual);
                                    if( VmbErrorSuccess == res )
                                       {
                                          avtCameras[theCameraNumber].actualSettings.theShutter = theExposureActual;
                                          imageParameter.value = std::to_string(theExposureActual);
                                          imageParameter.cameraNumber = theCameraNumber;
                                          myLcm.publish("M_STATUS_PARAMETERS", &imageParameter);
                                          //printf(" exposure = %lf %s\n",theExposureActual);
                                       }
                                 }
                           }
                        else if(CAMERA_GAIN == parameterQueryTypes[currentQueryType])
                           {
                             // printf(" gain\n");
                              imageParameter.key = "GAIN";
                              AVT::VmbAPI::FeaturePtr pGainFeature;
                              VmbErrorType res = SP_ACCESS( theAVTCamera )->GetFeatureByName( "GainRaw", pGainFeature );
                              if( VmbErrorSuccess == res )
                                 {
                                    long long theGainActual;
                                    res = SP_ACCESS( pGainFeature )->GetValue(theGainActual);
                                    //printf(" res %d\n",res);
                                    if( VmbErrorSuccess == res )
                                       {
                                          avtCameras[theCameraNumber].actualSettings.theGain = (double)theGainActual;
                                          imageParameter.value = std::to_string(theGainActual);
                                          imageParameter.cameraNumber = theCameraNumber;
                                          myLcm.publish("M_STATUS_PARAMETERS", &imageParameter);
                                          //printf(" gain = %lf\n",(double)theGainActual);

                                       }
                                 }
                           }
                        currentQueryType++;
                        break;
                     }
                  case PNG:
                     {			// recieved a PNG (Ping) message

                        const char *msg = "vimc thread is Alive!";

                        // respond with a SPI (Status Ping) message
                        msg_send( hdr.from, myThreadNumber, SPI, strlen (msg), msg);
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

                        printf ("fly  thread: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", (int)myThreadNumber, hdr.type, hdr.from, hdr.to, hdr.length);
                        break;
                     }

                  }			// switch
            } // end else msg get ok
      }

}

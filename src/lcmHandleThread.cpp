/* -----------------------------------------------------------------------------
   lcm_handle_thread: handles all incoming LCM messages

   Modification History:
   DATE         AUTHOR   COMMENT
   2012-06-01   SS       Created and written
   -------------------------------------------------------------------------- */

#include "lcmHandleThread.h"
#include  "vimc.h"
extern lcm::LCM myLcm;
extern lcm::LCM squeezeLcm;
extern lcm::LCM lightLcm;

extern dsplLightT  dsplLights[MAX_N_OF_LIGHTS];
extern int   nOfDSPLLights;


class State
{
public:
   int usefulVariable;
};

void lightCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const raw::bytes_t *bytes, State *user)
{
   const unsigned char *theBytes = &*bytes->data.begin();
   //printf(" received light data %s\n",(char *)theBytes);
   int theBytesLength = bytes->length;
   for (int lightNumber = 0; lightNumber < nOfDSPLLights;  lightNumber++)
      {
          if(channel == dsplLights[lightNumber].commsInChannel)
             {
                msg_send(DSPLLIGHT_THREAD,LIGHT_THREAD_BASE + lightNumber,SAS,theBytesLength, theBytes);
                break;
             }
      }

}


void parameterCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const image::image_parameter_t *image, State *user)
{
   double value = std::stod(image->value);
   //printf(" got a parameter callback! value = %.2f\n",value);
   if("GAIN" == image->key)
      {
         printf(" the new gain is %0.1f\n",value);
         msg_send(FLY_THREAD_BASE + image->cameraNumber,LCM_RECEIVE_THREAD,WCG,(sizeof(value)),&value);
      }
   else if("DECIMATION" == image->key)
      {
          //printf(" change decimation rate!\n");
         int intValue = (int)value;
          msg_send(FLY_THREAD_BASE+ image->cameraNumber,LCM_RECEIVE_THREAD,WDECIMATION,(sizeof(intValue)),&intValue);
      }
   else if("STILL" == image->key)
      {
         printf(" take a still!\n");
         msg_send(FLY_THREAD_BASE + image->cameraNumber,LCM_RECEIVE_THREAD,STILL,0,NULL);
      }
   else if("EXPOSURE" == image->key)
      {
         printf(" the new exposure is %0.1f\n",value);
         msg_send(FLY_THREAD_BASE+ image->cameraNumber,LCM_RECEIVE_THREAD,WCE,(sizeof(value)),&value);
      }
   else if("AUTO_GAIN" == image->key.substr(0,9))
      {
         double theValues[3];
         char *theLine = (char *)image->key.c_str();
         theValues[0] = value;
         int items = sscanf(theLine,"AUTO_GAIN %lf %lf",&(theValues[1]), &(theValues[2]) );
         if(2 == items)
            {
               msg_send(FLY_THREAD_BASE+ image->cameraNumber,LCM_RECEIVE_THREAD,WCAG,(3 * sizeof(theValues)),&theValues);
               printf(" auto gain set to %0.1f\n",value);
            }
      }
   else if("BINNING" == image->key)
      {
         printf(" binning to %0.1f\n",value);
         msg_send(FLY_THREAD_BASE+ image->cameraNumber,LCM_RECEIVE_THREAD,WBIN,(sizeof(value)),&value);
      }
   else if("AUTO_EXPOSURE" == image->key.substr(0,13))
      {
         double theValues[3];
         char *theLine = (char *)image->key.c_str();
         theValues[0] = value;
         int items = sscanf(theLine,"AUTO_EXPOSURE %lf %lf",&(theValues[1]), &(theValues[2]) );
         if(2 == items)
            {
               msg_send(FLY_THREAD_BASE+ image->cameraNumber,LCM_RECEIVE_THREAD,WCAE,(3 * sizeof(theValues)),&theValues);
               printf(" auto exposure set to %0.1f\n",value);
            }
      }
   else if("REPETION_INTERVAL" == image->key.substr(0,17))
      {
         msg_send(FLY_THREAD_BASE+ image->cameraNumber,LCM_RECEIVE_THREAD,WREP,(sizeof(value)),&value);
      }
   else if("REPETION_STOP" == image->key)
      {
         msg_send(FLY_THREAD_BASE+ image->cameraNumber,LCM_RECEIVE_THREAD,WRES,0,NULL);
      }
   else if("LIGHT_LEVEL" == image->key)
      {
         int thedata[2];
         thedata[0] = (int)image->cameraNumber;
         thedata[1] = value;
         msg_send(DSPLLIGHT_THREAD,LCM_RECEIVE_THREAD,WLL,sizeof(thedata),thedata);
      }
   else if("LIGHT_CHANNEL" == image->key)
      {
         int thedata[2];
         thedata[0] = (int)image->cameraNumber;
         thedata[1] = value;
         msg_send(DSPLLIGHT_THREAD,LCM_RECEIVE_THREAD,WLC,sizeof(thedata),thedata);
      }
   else if("LIGHT_TEMP_REQUEST" == image->key.substr(0,18))
      {
         int thedata;
         thedata = (int)image->cameraNumber;
         msg_send(DSPLLIGHT_THREAD,LCM_RECEIVE_THREAD,RLT,sizeof(thedata),&thedata);
      }
   else if("LIGHT_HUMID_REQUEST" == image->key.substr(0,19))
      {
         int thedata;
         thedata = (int)image->cameraNumber;
         msg_send(DSPLLIGHT_THREAD,LCM_RECEIVE_THREAD,RLH,sizeof(thedata),&thedata);
      }


}

void *lcmHandleThread (void *)
{

   // Create lcm instance
   /*lcm::LCM *myLcm = new lcm::LCM("");
   if (!myLcm)
      {
         printf("lcm_create() failed\n");
         exit (EXIT_FAILURE);
      }
   else
      {
         printf( "lcm_create() succeeded");
      };*/

   State state;

   int squeezFileNo = squeezeLcm.getFileno();
   int lightFileNo = lightLcm.getFileno();
// change these subscribe functions to accomodate separation of bandwidth hog, 24 October 2019 jch
#if 0
   myLcm.subscribeFunction("COMMAND_PARAMETERS", &parameterCallback, &state);
   for (int lightNumber = 0; lightNumber < nOfDSPLLights;  lightNumber++)
      {
         myLcm.subscribeFunction(dsplLights[lightNumber].commsInChannel,&lightCallback, &state);
      }
#endif
   squeezeLcm.subscribeFunction("COMMAND_PARAMETERS", &parameterCallback, &state);
   for (int lightNumber = 0; lightNumber < nOfDSPLLights;  lightNumber++)
      {
         printf(" light %d subscribibng to %s\n",lightNumber,dsplLights[lightNumber].commsInChannel);
         lightLcm.subscribeFunction(dsplLights[lightNumber].commsInChannel,&lightCallback, &state);
      }


   // loop forever
   while(true)
      {
         //myLcm.handle();
         //squeezeLcm.handle();

         pollfd theFds[2];
         theFds[0].events = POLLIN;
         theFds[0].fd = squeezFileNo;
         theFds[1].events = POLLIN;
         theFds[1].fd = lightFileNo;
         poll(theFds,2,10);
         if(theFds[0].revents == POLLIN)
            {
               squeezeLcm.handle();
            }
         else if(theFds[1].revents == POLLIN)
            {
               lightLcm.handle();
            }
      }
}


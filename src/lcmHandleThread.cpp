/* -----------------------------------------------------------------------------
   lcm_handle_thread: handles all incoming LCM messages

   Modification History:
   DATE         AUTHOR   COMMENT
   2012-06-01   SS       Created and written
   2018-2023    jch      extensively modified and customized for use in dsvimc
   -------------------------------------------------------------------------- */

#include "lcmHandleThread.h"
#include  "vimc.h"
extern lcm::LCM myLcm;


stereo_event_t theStereoEvent;

class State
{
public:
   int usefulVariable;
};


void parameterCallback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,const image::image_parameter_t *image, State *user)
{
   double value = std::stod(image->value);
   //printf(" got a parameter callback! value = %.2f\n",value);
   if("GAIN" == image->key)
      {
         printf(" the new gain is %0.1f\n",value);
         msg_send(FLY_THREAD_BASE + image->cameraNumber,LCM_RECEIVE_THREAD,WCG,(sizeof(value)),&value);
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

   
   myLcm.subscribeFunction("COMMAND_PARAMETERS", &parameterCallback, &state);
   

   // loop forever
   while(true)
      {
         myLcm.handle();
      }
}


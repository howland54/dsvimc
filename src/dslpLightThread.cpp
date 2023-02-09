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
#include <iostream>
#include <sstream>
//#define DEBUG_LOGGING

/* local includes */
#include <../../dsvimlib/include/time_util.h>
#include <../../dsvimlib/include/convert.h>
#include <../../dsvimlib/include/global.h>
#include <../../dsvimlib/include/imageTalk.h>		/* jasontalk protocol and structures */
#include <../../dsvimlib/include/msg_util.h>		/* utility functions for messaging */
#include <../../dsvimlib/include/launch_timer.h>

#include "dsplLightThread.h"

/* posix header files */
#define  POSIX_SOURCE 1


#include <pthread.h>

unsigned char ucCalculateChecksum(  char *pcCommandString );
void sendNext(lightManagementT   *theLightManagement, char* theString, int stringLength);
extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;
extern dsplLightT  dsplLights[MAX_N_OF_LIGHTS];

extern lcm::LCM myLcm;
extern lcm::LCM squeezeLcm;
extern lcm::LCM lightLcm;



/* ----------------------------------------------------------------------
   dslp light thread


   Modification History:
   DATE        AUTHOR   COMMENT
   7 March 2019  jch      creation, derive from logging thread

---------------------------------------------------------------------- */
void *dsplLightThread (void *)
{

   msg_hdr_t hdr = { 0 };
   unsigned char data[MSG_DATA_LEN_MAX] = { 0 };
   

   lightManagementT  lightManagement[MAX_N_OF_LIGHTS];
   for(int lightNumber = 0; lightNumber < MAX_N_OF_LIGHTS; lightNumber++)
      {
         lightManagement[lightNumber].gotAckNack = false;
         lightManagement[lightNumber].ticksSinceSentData = 0;
         lightManagement[lightNumber].nextCommandToSend[0] = NULL;
         lightManagement[lightNumber].msgLength = 0;
         lightManagement[lightNumber].timeouts = 0;
         lightManagement[lightNumber].nacks = 0;
         lightManagement[lightNumber].activeRequest = 0;
         lightManagement[lightNumber].lastDataRecieved = rov_get_time() - 3600.0;
      }

   // wakeup message
   printf ("DSPL Light Thread (thread %d) initialized \n", DSPLLIGHT_THREAD);
   printf ("DSPLLIGHT_THREAD File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);
   usleep(700000);
   int msg_success = msg_queue_new(DSPLLIGHT_THREAD, "dspl light thread");

   if(msg_success != MSG_OK)
      {
         fprintf(stderr, "dspl light: %s\n",MSG_ERROR_CODE_NAME(msg_success) );
         fflush (stdout);
         fflush (stderr);
         abort ();
      }


   launched_timer_data_t *healthTimer;
   healthTimer = launch_timer_new(0.5,-1,DSPLLIGHT_THREAD,RHEALTH);

   launched_timer_data_t *commsTimer;
   commsTimer = launch_timer_new(DSPL_COMMS_TICK,-1,DSPLLIGHT_THREAD,LCOM);


   // loop forever
   while (1)
      {

         int msg_get_success = msg_get(DSPLLIGHT_THREAD,&hdr, data, MSG_DATA_LEN_MAX);
         if(msg_get_success != MSG_OK)
            {
               fprintf(stderr, "dspl light thread thread--error on msg_get:  %s\n",MSG_ERROR_CODE_NAME(msg_get_success));
            }
         else
            {
               // process new message
               switch (hdr.type)
                  {

                  case PNG:
                     {			// recieved a PNG (Ping) message

                        const char *msg = "dspl light thread thread is Alive!";

                        // respond with a SPI (Status Ping) message
                        msg_send( hdr.from, BUS_THREAD, SPI, strlen (msg), msg);
                        break;
                     }
                  case BYE:  // received a bye message--time to give up the ghost--
                     {
                        return(NULL);
                        break;
                     }
                  case SAS:
                     {
                        data[hdr.length] = '\0';
                        //printf(" lst request %d got a SAS %s\n",lightManagement[0].lastRequest,(char *)data);
                        bool gotAnAckNack = false;
                        if((0x0a == data[hdr.length-1] ) && (0x0d == data[hdr.length-2] ))
                           {
                              gotAnAckNack = true;
                           }

                        for(int lightNumber = 0; lightNumber < nOfDSPLLights; lightNumber++)
                           {
                              if( (LIGHT_THREAD_BASE+lightNumber) == hdr.from)
                                 {
                                    lightManagement[lightNumber].lastDataRecieved = rov_get_time();
                                    lightManagement[lightNumber].gotAckNack = gotAnAckNack;
                                    if(hdr.length >= 3)
                                       {
                                          int items;
                                          if(data[0] == '?')
                                             {
                                                lightManagement[lightNumber].nacks++;
                                                break;
                                             }
                                          if(0 == lightManagement[lightNumber].lastRequest) // a temp request
                                             {
                                                int tempTemp;
                                                items = sscanf((char *)data,"%d",&tempTemp);
                                                if(items)
                                                   {
                                                      dsplLights[lightNumber].temperature = (double) tempTemp;
                                                   }
                                             }
                                          else if(1 == lightManagement[lightNumber].lastRequest) // a humidity request
                                             {
                                                int tempHumid;
                                                items = sscanf((char *)data,"%d",&tempHumid);
                                                if(items)
                                                   {
                                                      dsplLights[lightNumber].humidity = (double) tempHumid;
                                                   }
                                             }
                                          else if(2 == lightManagement[lightNumber].lastRequest) // a channel request
                                             {
                                                int tempChannel;
                                                items = sscanf((char *)data,"%d",&tempChannel);
                                                if(items)
                                                   {
                                                      dsplLights[lightNumber].mode = (double) tempChannel;
                                                   }
                                             }
                                          else if(3 == lightManagement[lightNumber].lastRequest) // a power level request
                                             {
                                                int tempLevel;
                                                items = sscanf((char *)data,"%d",&tempLevel);
                                                if(items)
                                                   {
                                                      dsplLights[lightNumber].powerLevel = (double) tempLevel;
                                                   }
                                             }
                                       }
                                 }
                           }

                        break;
                     }
                  case LCOM:
                     {
                        struct timeval utc;
                        gettimeofday(&utc, NULL);
                        int64_t utime = (int64_t)utc.tv_sec * 1000000 + (int64_t)utc.tv_usec;
                        for(int lightNumber = 0; lightNumber < nOfDSPLLights; lightNumber++)
                           {

                              //printf(" lcom ack:  %d ticks %d\n",lightManagement[lightNumber].gotAckNack,lightManagement[lightNumber].ticksSinceSentData );
                              if((lightManagement[lightNumber].gotAckNack) || (lightManagement[lightNumber].ticksSinceSentData > LIGHT_TICK_CRITERIA))
                                 {
                                    if(lightManagement[lightNumber].nextCommandToSend[0])
                                       {

                                          if(lightManagement[lightNumber].ticksSinceSentData > LIGHT_TICK_CRITERIA)
                                                {
                                                   lightManagement[lightNumber].timeouts++;
                                                }
                                          // send the message, set the next command to NULL, zero the ticks
                                          //printf(" sent messg to %d length %d content %s\n",LIGHT_THREAD_BASE+lightNumber,lightManagement[lightNumber].msgLength,&(lightManagement[lightNumber].nextCommandToSend[0]));
                                          //msg_send(LIGHT_THREAD_BASE+lightNumber,DSPLLIGHT_THREAD,WAS,lightManagement[lightNumber].msgLength,&(lightManagement[lightNumber].nextCommandToSend[0]));
                                          raw::bytes_t outgoingMsg;
                                          outgoingMsg.utime = utime;
                                          outgoingMsg.length = lightManagement[lightNumber].msgLength;

                                          outgoingMsg.data.insert(outgoingMsg.data.begin(),(unsigned char *)&(lightManagement[lightNumber].nextCommandToSend[0]),(unsigned char *)&(lightManagement[lightNumber].nextCommandToSend[0]) + outgoingMsg.length);
                                          //printf(" publishing %s to %s\n",(char *)&(lightManagement[lightNumber].nextCommandToSend[0]),dsplLights[lightNumber].commsOutChannel);
                                          lightLcm.publish(dsplLights[lightNumber].commsOutChannel,&outgoingMsg);


                                          lightManagement[lightNumber].nextCommandToSend[0] = NULL;
                                          lightManagement[lightNumber].msgLength = 0;
                                          lightManagement[lightNumber].ticksSinceSentData = 0;
                                       }
                                 }
                              else
                                 {
                                    lightManagement[lightNumber].ticksSinceSentData++;

                                 }
                           }
                        break;
                     }
                  case RHEALTH:  // need to request health data
                     {
                        rov_time_t theTime = rov_get_time();
                        for (int lightNumber = 0; lightNumber < nOfDSPLLights; lightNumber++)
                           {
                              char requestString[32];
                              int len = 0;
                              int whatToRequest = lightManagement[lightNumber].activeRequest;
                              if(0 == whatToRequest)  // in this case, ask for temperature
                                 {
                                    len = snprintf(requestString,32,"!%s:temp?*",dsplLights[lightNumber].lightID);

                                 }
                              else if(1 == whatToRequest)// otherwise, ask for humidity
                                 {
                                    len = snprintf(requestString,32,"!%s:rhum?*",dsplLights[lightNumber].lightID);
                                 }
                              else if(2 == whatToRequest)// otherwise, ask for active channel
                                 {
                                    len = snprintf(requestString,32,"!%s:chsw?*",dsplLights[lightNumber].lightID);
                                 }
                              else if(3 == whatToRequest)// otherwise, ask for light output
                                 {
                                    len = snprintf(requestString,32,"!%s:lout?*",dsplLights[lightNumber].lightID);
                                 }
                              sendNext(&(lightManagement[lightNumber]),requestString,len) ;
                              lightManagement[lightNumber].lastRequest = whatToRequest;
                              lightManagement[lightNumber].activeRequest++;
                              if(4 == lightManagement[lightNumber].activeRequest)
                                 {
                                    lightManagement[lightNumber].activeRequest = 0;
                                 }
                              // now post a status message
                              dsplLight::lightStatus_t  lightStatus;
                              lightStatus.utime =theTime;
                              lightStatus.lightNumber = lightNumber;
                              lightStatus.lightLevel = dsplLights[lightNumber].powerLevel;
                              lightStatus.humidity = (int32_t) dsplLights[lightNumber].humidity;
                              lightStatus.temperature = (int32_t) dsplLights[lightNumber].temperature;
                              lightStatus.channelMode = (int32_t) dsplLights[lightNumber].mode;
                              lightStatus.secsSinceComms = theTime - lightManagement[lightNumber].lastDataRecieved;
                              lightStatus.nackCount = lightManagement[lightNumber].nacks;
                              squeezeLcm.publish("M_DSPL_LIGHTS",&lightStatus);
                           }

                        break;
                     }
                  case WLL:
                     {
                        int *parameters =  (int *) data;
                        int lightNumber = parameters[0];
                        int desiredLightLevel = parameters[1];
                        char requestString[32];
                        int len = 0;
                        len = snprintf(requestString,32,"!%s:lout=%03d*",dsplLights[lightNumber].lightID,desiredLightLevel);
                        sendNext(&(lightManagement[lightNumber]),requestString,len) ;
                        lightManagement[lightNumber].activeRequest = 3;  // ask for it next
                        //printf(" got a WLL %s\n",requestString);
                        break;
                     }
                  case WLC:
                     {
                        int *parameters =  (int *) data;
                        int lightNumber = parameters[0];
                        int desiredChannelSetting = parameters[1];
                        char requestString[32];
                        int len = 0;
                        len = snprintf(requestString,32,"!%s:chsw=%01d*",dsplLights[lightNumber].lightID,desiredChannelSetting);
                        sendNext(&(lightManagement[lightNumber]),requestString,len) ;
                        lightManagement[lightNumber].activeRequest = 2; // ask for it next
                        //printf(" got a WLC %s\n",requestString);
                        break;
                     }


                  default:
                     {			// recieved an unknown message type

                        printf ("dspllight thread: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE - " "got msg type %d from proc %d to proc %d, %d bytes data\n", BUS_THREAD, hdr.type, hdr.from, hdr.to, hdr.length);
                        break;
                     }

                  }			// switch
            } // end eles msg get ok
      }

}
unsigned char ucCalculateChecksum(  char * pcCommandString )
{
   unsigned int uiRunningTotal = 0;
   unsigned int i;

   /*Calculate checksum*/
   for( i = 0; i < strlen( pcCommandString ); i++)
   {
      /*Add character to running total*/
      uiRunningTotal += ( unsigned int ) pcCommandString[i];
   }
   /*Mask the lower byte and return the value to the caller*/
   return (unsigned char) ( uiRunningTotal & 0xFF);
}

void sendNext(lightManagementT   *theLightManagement, char *theString, int stringLength)
{
   unsigned char cs = ucCalculateChecksum(theString);
   int len = stringLength + snprintf(&(theString[stringLength]),32 ,"%02X\r\n",cs);
   strncpy(&(theLightManagement->nextCommandToSend[0]),theString,31);
   theLightManagement->msgLength = len;
}


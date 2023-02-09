#ifndef DSPLLIGHTTHREAD_H
#define DSPLLIGHTTHREAD_H
#include <sys/time.h>
#include <lcm/lcm-cpp.hpp>

#include <../../dsvimlib/include/time_util.h>
#include <mesobot-lcmtypes/dsplLight/lightStatus_t.hpp>
#include <mesobot-lcmtypes/raw/bytes_t.hpp>

#include "vimc.h"

extern void *dsplLightThread (void *);

extern char *flyIniFile;
extern int nOfDSPLLights;

#define LIGHT_TICK_CRITERIA   3
#define DSPL_COMMS_TICK 0.1

typedef struct
{
   int         ticksSinceSentData;
   bool        gotAckNack;
   char        nextCommandToSend[32];
   int         msgLength;
   int         timeouts;
   int         nacks;
   int         activeRequest;
   int         lastRequest;
   rov_time_t  lastDataRecieved;
}   lightManagementT;

typedef struct
{
   double temperature;
   double humidity;
   int mode;
   int powerLevel;
} dsplLightStatusT;

#endif // DSPLLIGHTTHREAD_H

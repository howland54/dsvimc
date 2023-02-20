/* ----------------------------------------------------------------------

Sensor thread for ROV control system

Modification History:
DATE         AUTHOR  COMMENT
23-JUL-2000  LLW     Created and written.
04 MAR 02    LLW     Commisioned altimeter in new controller, added 
error and temp to altimeter data struct. 
07 JUL 02    LLW     Removed filtered depth from depth sensor data struct
07 JUL 02    LLW     Changed vehicle depth sensor from single to array of structs
14 Nov 02    jch     added external_fix_t and external fix to structures
27 OCT 2003  JCK&LLW XVISION
13 MAY 2004 LLW Cleaned up TCM2 parsing code and assignment in sensor_thread.cpp
13 Oct 2004 LLW Enabled DVZ processing for Auto-Altitude
13 OCT 2004  LLW      Created dvlnav_data and dvlnav_data_last in sensor_t.
                      Retired dvlnav_state and dvlnav_state_last from sensor_t,
                      the dvlnav state data is now in dvlnav_data.state and  
                      dvlnav_data_last.state 
20 Feb 2023   jch     used as shell for habcam sensor thread
---------------------------------------------------------------------- */
#ifndef SENSOR_PROCESS_INC
#define SENSOR_PROCESS_INC

#include <pthread.h>


// ----------------------------------------------------------------------
// DEBUG FLAG:  Uncomment these and recompile to get verbose debugging 
// ----------------------------------------------------------------------
//#define DEBUG_SENSOR
//#define DEBUG_ALTIMETER
//#define DEBUG_SBE_37
//#define DEBUG_PAROSCIENTIFIC
//#define DEBUG_HONEYWELL_MAG
//#define DEBUG_DVLNAV
//#define DEBUG_SENSOR_INI
//#define DEBUG_TCM2

// ----------------------------------------------------------------------

#define ONE_ATMOSPHERE_IN_METERS  10.197163
#define SENSOR_THREAD_INTERVAL  0.25
#define BOGUS_ALTIMETER_VALUE -999.0
#define INVALID_ALTITUDE  -999.0

#define DEFAULT_MICROSTRAIN_ROLL_OFFSET    0.00
#define DEFAULT_MICROSTRAIN_PITCH_OFFSET    0.00
#define DEFAULT_MICROSTRAIN_HDG_OFFSET    0.00

extern char *flyIniFile;


extern void *sensorThread (void *);



#define NUM_DEPTH_SENSORS 1

/* ====================================================================== 
   Sensor data structure: Contains readings from all sensors in
   engineering units.  

   23-JUL-00 LLW   Complely revised and restructured
   07 JUL 02 LLW   Changed vehicle depth sensor from single to array of structs
   27 OCT 2003  JCK&LLW XVISION

   =================================================================== */


/* a structure for scientific unit sensor readings */
typedef struct
{
  // mutex for shared memory access  
  pthread_mutex_t mutex;

  // housekeeping stuff used internally in sensor thread
  unsigned msgs_sent;
  unsigned msgs_received;
  int verbose_mode;


  

}
sensor_t;



void sensor_thread_get_data (sensor_t * data);
extern int read_sensor_config(FILE * ini_fd,sensor_t *sensor);

#endif

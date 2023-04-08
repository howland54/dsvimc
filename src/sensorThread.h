/* ----------------------------------------------------------------------


20 Feb 2023   jch     former rov code used as shell for habcam sensor thread
---------------------------------------------------------------------- */
#ifndef SENSOR_PROCESS_INC
#define SENSOR_PROCESS_INC

#include <pthread.h>

#include <lcm/lcm-cpp.hpp>

#include "nmea.h"

#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/marineSensorAltimeter_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorCtd_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorGPS_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorFathometer_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorAttitudeSensor_t.hpp"
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

#define  UNKNOWN_SOUND_SPEED     -999.0
#define  UNKNOWN_SALINITY        -99.9

#define MAX_MESSAGE_LENGTH  1024


#define NUM_DEPTH_SENSORS 1

typedef struct
{
  double conductivity;
  double temperature;
  double depth;
  double  salinity;
  double  pressure;
  double  sound_velocity;
  double  O2;
  int valid_parse;
  int valid_data;
  int bad_cnt;			//number bad parses since last good one
}
ctd_t;

typedef struct
{
  double pos;
  double temp;
  int    error;
}
altimeter_t;

typedef struct
{
    double  latitude;
    double  longitude;
}
gps_t;

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

  ctd_t     ctd;
  altimeter_t   altimeter;
  altimeter_t   fathometer;
  double        fathometerMultiplier;
  gps_t         vesselPosition;



  

}
sensor_t;


#endif

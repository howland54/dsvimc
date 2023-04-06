/* ----------------------------------------------------------------------

   code for tsimulating a pair of avt cameras and some other sensors on Habcam

   Modification History:
   DATE        AUTHOR   COMMENT
   1-April-2023  jch      creation, derive from bus thread
   ---------------------------------------------------------------------- */

#ifndef SIMULATION_THREAD_INC
#define SIMULATION_THREAD_INC


#include <VimbaCPP/Include/VimbaSystem.h>
#include <VimbaCPP/Include/VimbaCPP.h>
using AVT::VmbAPI::VimbaSystem;

#include <CameraObserver.h>


#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/marineSensorAltimeter_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorCtd_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorGPS_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorFathometer_t.hpp"
#include "../../habcam-lcmtypes/marine_sensor/marine_sensor/MarineSensorAttitudeSensor_t.hpp"



/* ---------------------------------------------------------------------
   bus thread external def
   --------------------------------------------------------------------- */
extern void *simulationThread (void *);



#endif

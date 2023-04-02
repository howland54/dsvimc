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





/* ---------------------------------------------------------------------
   bus thread external def
   --------------------------------------------------------------------- */
extern void *simulationThread (void *);



#endif

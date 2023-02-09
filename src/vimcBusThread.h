/* ----------------------------------------------------------------------

   code for talkinig to the USB bus for flycapture

   Modification History:
   DATE        AUTHOR   COMMENT
   13-Apr-217  jch      creation, derive from jason crossbow prompt thread
   ---------------------------------------------------------------------- */

#ifndef BUS_THREAD_INC
#define BUS_THREAD_INC


#include <VimbaCPP/Include/VimbaSystem.h>
#include <VimbaCPP/Include/VimbaCPP.h>
using AVT::VmbAPI::VimbaSystem;

#include <CameraObserver.h>





/* ---------------------------------------------------------------------
   bus thread external def
   --------------------------------------------------------------------- */
extern void *busThread (void *);



#endif

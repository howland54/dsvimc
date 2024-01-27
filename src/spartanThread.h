/* ----------------------------------------------------------------------

** spartan Thread
** spartanThread.cpp
**
** Description:
**
**
**
** History:
**       Date       Who      Description
**      ------      ---      --------------------------------------------
**      1/2024      jch      Create from DATA_THREAD.h
---------------------------------------------------------------------- */
#ifndef SPARTAN_INC
#define SPARTAN_INC

extern char *flyIniFile;

// ----------------------------------------------------------------------
// DEBUG FLAG:  Uncomment this and recompile to get verbosr debugging 
// ----------------------------------------------------------------------
// #define DEBUG_SPARTAN

// ----------------------------------------------------------------------

extern void *global_control;
extern void *spartanThread (void *);

#define SPARTAN_DEFAULT_QUERY_INTERVAL  0.2
#define SPARTON_DEFAULT_LATITUDE 41.7
#define SPARTON_DEFAULT_LONGITUDE -71.0

#define WMM_QUERY_INTERVAL 5.17

#endif

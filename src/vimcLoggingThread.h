/* ----------------------------------------------------------------------


   ---------------------------------------------------------------------- */

#ifndef LOGGING_THREAD_INC
#define LOGGING_THREAD_INC

#include <time.h>
#include <sys/time.h>

#include "../../dsvimlib/include/IniFile.h"
#define MAX_LOGGING_LENGTH 2048

typedef struct
{
  int ascii_log_flag;
  int last_hour;
  char ascii_log_file_name[256];
  FILE *logAsciiFilePointer;
  char *logging_directory;
}
logging_t;


extern int logThisNow (logging_t *log, char *record_data);
extern int readIniLoggingProcess (logging_t *log);

extern char *flyIniFile;



/* ---------------------------------------------------------------------
   logging thread external def
   --------------------------------------------------------------------- */
extern void *loggingThread (void *);



#endif

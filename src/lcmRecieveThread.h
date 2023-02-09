#ifndef LCM_RECIEVE_INC
#define LCM_RECIEVE_INC

/* standard ansi C header files */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* posix header files */
#define  POSIX_SOURCE 1
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include <opencv2/opencv.hpp>

#include <dsvimlib/IniFile.h>

#include <mesobot-lcmtypes/image/image_t.hpp>


#define MAX_IMAGE_SIZE 15000000
#define MAX_N_OF_IMAGE_BUFFERS 8

typedef struct {
   unsigned char           *imagePointer;
   pthread_mutex_t         imageMutex;
   long int                imageSize;
   int                     rows;
   int                     columns;
   pthread_t               computeSubThread;
   roiDetector_t           roiParameters;
} imageHolder;

typedef struct {
   imageHolder    images[MAX_N_OF_IMAGE_BUFFERS];
   int            nextImage;
}  imageBuffer;

extern char *lcmRIniFile;


extern void *lcmRecieveThread (void *);

#endif

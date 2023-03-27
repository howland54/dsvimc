/* ----------------------------------------------------------------------

Ms Thread

Modification History:
DATE         AUTHOR   COMMENT
26-OCT-2006  LLW      Created and written.

---------------------------------------------------------------------- */
/* standard ansi C header files */
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* posix header files */
#define  POSIX_SOURCE 1

#include <pthread.h>
#include <termios.h>
#include <unistd.h>

/* jason header files */
#include "jasontalk.h"		/* jasontalk protocol and structures */
#include "launch_timer.h"	/* launch timer stuff */
#include "msg_util.h"		/* utility functions for messaging */
#include "stderr.h"             /* error handling */
#include "sio_thread.h"	/* this thread */
#include "ms_thread.h"	/* this thread */
#include "microstrain.h"
#include "config_file.h"

/* changed 4/2/09 by jch, put in prompt to sample microstrain, it will only run at max rate.
   this is way too fast, so sample every (for now) .2 seconds
   should make this an ini file  entry*/

extern pthread_attr_t DEFAULT_ROV_THREAD_ATTR;

micro_strain_t 	ms = {0};

/* ---------------------------------------------------------------------- 

int ms_update(sio_thread_t *sio, char inchar)

  Returns    1  when new characters have arrived at the serial port,
  but not yet a full data packet
  -1  when no new characters have been recieved at the serial port
  0  when a new dvl data packet has been received successfully,
  and the checksum indicates that it is good data.

  DATE         WHO             WHAT
  -----------  --------------  ----------------------------
  26 OCT 2007  Louis Whitcomb  Created and Written

  ---------------------------------------------------------------------- */
int ms_update(sio_thread_t *sio, char inchar)
{

   static int parse_sync =0;
   //  static int buff_count =0;
   int status = 1;


#define START_OF_DATA_CHAR 0x31

   /* are we synced with the input stream? (i.e. have we seen a "%" character)?*/
   if(parse_sync == 0)
      {
         /* we are not synced, so see if it is a "START OF DATA" character */
         /* if the character is START_OF_DATA_CHAR */
         if(inchar == START_OF_DATA_CHAR)
            {

               /* store the character in the input buffer */
               sio->inchars[sio->inchar_index] = inchar;

               /* increment the buffer count */
               sio->inchar_index++;

               /* increment the buffer count set the sync flag */
               parse_sync = 1;
               //rov_time_t tn = rov_get_time();

               //printf("DATA START...%.2f %x\n",tn,inchar);
            }

      }
   else
      {
         /* if we are synced with the data stream get the character from the serial
    port input buffer and store it to be parsed  */
         sio->inchars[sio->inchar_index] = inchar;

         /* increment the buffer count */
         sio->inchar_index++;


         /* if we have recieved a full number of characters after the valid
    start character, then parse the sring */
         if (sio->inchar_index >= 23)
            { // got new char, synced, got full string

               //          status =  process_microstrain_binary_string
               //                int process_microstrain_binary_string(micro_strain_t * microstrain, unsigned char * ustr, int len)
               //printf("PARSING...");

               status = microstrain_parse_binary_string(&ms, sio->inchars, 23);

               if( status == MICROSTRAIN_STRING_OK)
                  {

                     //printf(" sending an MSD\n");
                     msg_send(SENSOR_THREAD,
                              MS_THREAD,
                              MSD,
                              sizeof(ms),
                              &ms);

                     log_microstrain_ascii_string( &ms);
                  }

               // zero out the counters
               sio->inchar_index = 0;
               parse_sync = 0;


            }
      }

   return(status);

}


/* ----------------------------------------------------------------------

   Function to process incoming characters

   This is a first generation version.
   Needs to be modified to read() a block of characters
   at a time and process them more efficiently.

   Modification History:
   DATE         AUTHOR  COMMENT
   19-JUL-2000  LLW     Created and written.
   2007-10-26   LLW     Modified for microstrain binary sensor

---------------------------------------------------------------------- */
static void * sio_rx_thread (void *arg)
{

   stderr_printf("MS_THREAD: initiating rx subthread\n");

   sio_thread_t *sio = (sio_thread_t *) arg;
   int fid;
   ssize_t num;
   char new_chars[256] = { 0 };

   sio = (sio_thread_t *) arg;
   fid = sio->sio_port_fid;

   while (1)
      {
         // read a character
         num = read(fid, new_chars, 1);
         //      printf("%02X",new_chars[0]);

         // lock the data structure mutex so that we chan diddle with the data
         //      pthread_mutex_lock (&sio->mutex);

         // add the character to the serial I/O array
         //	  sio->inchars[sio->inchar_index] = new_chars[0];

         // keep rx stats
         sio->total_rx_chars += num;

         ms_update(sio, new_chars[0]);

         //      pthread_mutex_unlock (&sio->mutex);

      }

   return NULL;
}



/* ----------------------------------------------------------------------

ms thread function to open serial port

Modification History:
DATE         AUTHOR   COMMENT
26-OCT-2006  LLW      Created and written.

---------------------------------------------------------------------- */
static int open_serial (sio_thread_t * sio)
{

   struct termios tio = { 0 };

   // open the serial port
   sio->sio_port_fid = open(sio->my_sio_port_table_entry.sio_com_port_name, O_RDWR | O_NOCTTY);

   // return immediately if device fails to open
   if (sio->sio_port_fid < 0)
      return (sio->sio_port_fid);

   // set tio.cflag
   tio.c_cflag = CLOCAL;		// ignore modem control lines like carrier detect
   tio.c_cflag |= CREAD;		// enable uart reciever

   // switch statement to set the baud rate bits in tio.cflag
   // use a macro here to save typing and reduce errors
   // the serial I/O macros B9600, etc, are defined in /usr/include/bits/termios.h
   // note that we use the macro text concatenation operator ## here
#define setbaud( BAUD ) case  BAUD :  tio.c_cflag |= B ## BAUD ; break

   switch (sio->my_sio_port_table_entry.baud)
      {
      setbaud (0);
      setbaud (50);
      setbaud (75);
      setbaud (110);
      setbaud (134);
      setbaud (150);
      setbaud (200);
      setbaud (300);
      setbaud (600);
      setbaud (1200);
      setbaud (1800);
      setbaud (2400);
      setbaud (4800);
      setbaud (9600);
      setbaud (19200);
      setbaud (38400);
      setbaud (57600);
      setbaud (115200);
      setbaud (230400);
      setbaud (460800);
      setbaud (500000);
      setbaud (576000);
      setbaud (921600);
      setbaud (1000000);
      setbaud (1152000);
      setbaud (1500000);
      setbaud (2000000);
      setbaud (2500000);
      setbaud (3000000);
      setbaud (3500000);
      setbaud (4000000);
      default:
         tio.c_cflag |= B9600;
         break;
      }

   // switch statement to set the data bit width in tio.cflag
   switch (sio->my_sio_port_table_entry.baud)
      {
      case 5:
         tio.c_cflag |= CS5;
         break;
      case 6:
         tio.c_cflag |= CS6;
         break;
      case 7:
         tio.c_cflag |= CS7;
         break;
      case 8:
         tio.c_cflag |= CS8;
         break;
      default:
         tio.c_cflag |= CS8;
         break;
      }


   // ignore incoming BREAK events
   tio.c_iflag |= IGNBRK;

   // switch statement to set the parity tio.cflag
   // default is no parity
   switch (sio->my_sio_port_table_entry.parity)
      {
      case SIO_PARITY_ODD:
         tio.c_cflag |= PARENB;	// enable output parity generation, default to odd
         tio.c_cflag |= PARODD;
         tio.c_iflag |= INPCK;	// enable input parity checking
         break;
      case SIO_PARITY_EVEN:
         tio.c_cflag |= PARENB ;	// enable output parity generation, even
         tio.c_cflag &= ~PARODD;
         tio.c_iflag |= INPCK;	// enable input parity checking
         break;
      case SIO_PARITY_MARK:
      case SIO_PARITY_SPACE:
      case SIO_PARITY_NONE:
      default:
         break;
      }

   // set stop bit parametrs in tio.cflag
   switch (sio->my_sio_port_table_entry.stop_bits)
      {
      case 2:
         tio.c_cflag |= CSTOPB;
         break;
      case 1:
      default:
         break;
      }


   // set flow control parametrs
   switch (sio->my_sio_port_table_entry.flow_control)
      {
      case SIO_FLOW_CONTROL_XONXOFF:
         tio.c_iflag |= IXON | IXOFF;	// enable XON/XOFF
         printf(" set an xon off parameter!\n");
         break;
      case SIO_FLOW_CONTROL_RTSCTS:
         tio.c_cflag |= CRTSCTS;	// enable hardware RTC CTS flow control
         break;
      case SIO_FLOW_CONTROL_XONOFF_RTSCTS:
         tio.c_iflag |= IXON | IXOFF;	// enable XON/XOFF
         tio.c_cflag |= CRTSCTS;	// enable hardware RTC CTS flow control
         break;
      case SIO_FLOW_CONTROL_NONE:
      default:
         break;
      }

   // set tio.oflag
   tio.c_oflag = 0;

   // set input mode (non-canonical, no echo,...)
   // echoing will be done in manually in this thread, not by UNIX
   tio.c_lflag = 0;
   tio.c_cc[VTIME] = 0;		/* inter-character timer unused */
   tio.c_cc[VMIN] = 1;		/* blocking read until at least 1 chars received */

   // flush the serial port
   // 2007-09-20 LLW Changed TCIFLUSH to TCIOFLUSH
   tcflush (sio->sio_port_fid, TCIOFLUSH);

   // set the attributes
   tcsetattr (sio->sio_port_fid, TCSANOW, &tio);

   // flush the serial port
   // 2007-09-20 LLW added second flush
   // tcflush (sio->sio_port_fid, TCIOFLUSH);

   return( sio->sio_port_fid );

}



/* ----------------------------------------------------------------------

ms Thread

Modification History:
DATE         AUTHOR   COMMENT
26-OCT-2006  LLW      Created and written.

---------------------------------------------------------------------- */
void * ms_thread (void *thread_num)
{

   // declare local variables for message handling
   msg_hdr_t hdr = { 0, 0, 0, 0};
   unsigned char data[MSG_DATA_LEN_MAX+1] = { 0 };
   FILE *ini_fd;
   // sio port stuff
   sio_thread_t sio ={ PTHREAD_MUTEX_INITIALIZER, -1 };

   // declare working local variables
   int my_thread_num;
   int msg_success;
   //launched_timer_data_t * timer_stuff;
   int have_a_port = 0;

   /* get my thread number */
   my_thread_num = (long int) thread_num;

   launched_timer_data_t * sync_timer = launch_timer_new(MICROSTRAIN_POLL_INTERVAL, -1, MS_THREAD, MSPI);



   // ------------------------------------------------------------
   // wakeup message
   // ------------------------------------------------------------
   stderr_printf ("MS (thread %d, %s) initialized \n", MS_THREAD, JASONTALK_THREAD_NAME(my_thread_num));
   stderr_printf ("MS File %s compiled at %s on %s\n", __FILE__, __TIME__, __DATE__);

   // ------------------------------------------------------------
   // initialize a message queue for me
   // ------------------------------------------------------------
   msg_success = msg_queue_new(my_thread_num, "ms_thread");
   char *device;
   if(msg_success != MSG_OK)
      {
         stderr_printf ("MS THREAD: Could not initialize queue, error is %s\n",
                        MSG_ERROR_CODE_NAME(msg_success));
         fflush (stderr);
         abort ();
      }


   /* Launch a timer to send me messages at specified interval */
   /*  timer_stuff = launch_timer_new( 1.0,             // send a message at this dt
                 -1,              // this many times (-1 = infinite)
                 my_thread_num,      // send the messages to this thread (me)
                 TIMER_UPDATE_RESPONSE);  // send this type of message (header only, no data)
  */

   ini_fd = fopen (rov_ini_file_name, "r");
   if (!ini_fd)
      {
         fprintf (stderr, "%s ini file does not exist...exiting!\n", rov_ini_file_name);
         fflush (stdout);
         fflush (stderr);
         abort ();
      }

   else
      {
         device = rov_read_string (ini_fd, "MS_THREAD", "DEVICE", "null");
         fclose(ini_fd);
      }
   // ------------------------------------------------------------
   // open serial port
   // ------------------------------------------------------------
   // set sio port parameters
   sio.inchar_index = 0;
   sio.total_tx_chars = 0;
   sio.total_rx_chars = 0;
   sio.sio_thread_num  = my_thread_num;

   // hard code port parameters for now
   sio.my_sio_port_table_entry.thread_number = my_thread_num;
   //  sio.my_sio_port_table_entry.sio_com_port_name = "/dev/usb/ttyUSB3";
   sio.my_sio_port_table_entry.sio_com_port_name = strdup(device);//"/dev/ttyd07";
   sio.my_sio_port_table_entry.name_of_thing_this_com_port_is_connected_to = strdup("Microstrain");
   sio.my_sio_port_table_entry.baud = 38400;
   sio.my_sio_port_table_entry.parity = SIO_PARITY_NONE;
   sio.my_sio_port_table_entry.data_bits = 8;
   sio.my_sio_port_table_entry.stop_bits = 1;
   sio.my_sio_port_table_entry.flow_control= SIO_FLOW_CONTROL_NONE;
   sio.my_sio_port_table_entry.echo = 0;

   if(open_serial(&sio) >= 0)
      {
         stderr_printf("%s opened serial port %s OK\n", JASONTALK_THREAD_NAME(my_thread_num), sio.my_sio_port_table_entry.sio_com_port_name);
         have_a_port = 1;
      }
   else
      {
         stderr_printf("%s FAILED to open serial port %s\n", JASONTALK_THREAD_NAME(my_thread_num), sio.my_sio_port_table_entry.sio_com_port_name);
      }

   if (have_a_port != 0)
      {
         pthread_create (&sio.sio_rx_subthread, &DEFAULT_ROV_THREAD_ATTR, sio_rx_thread, (void *) (&sio));

         stderr_printf("MS_THREAD: SIO just launched rx thread.\n");

      }


   // ------------------------------------------------------------
   // loop forever
   // ------------------------------------------------------------
   while (1)
      {

         // #define DEBUG_MS 1

         /* wait forever on the input channel */
         msg_get(my_thread_num, &hdr, data, MSG_DATA_LEN_MAX);

         // ------------------------------------------------------------
         /* process new message */
         // ------------------------------------------------------------
         switch (hdr.type)
            {

            /* ----------------------------------------
        received PNG (Ping) message
        ---------------------------------------- */
            case PNG:
               {
                  const char *msg = "MS_THREAD is Alive!";


                  /* respond with a SPI (Status Ping) message */
                  // be careful emulating this - this sends a message BACK
                  // to the originator of the message I just received
                  msg_send(hdr.from, hdr.to, SPI, strlen (msg), msg);

                  // print newsy message
                  stderr_printf ("MS_THREAD: (thread %d) got msg type %s (%d)  from proc %s (%d) to proc %s (%d), %d bytes data\n",
                                 MS_THREAD,
                                 JASONTALK_MESSAGE_NAME(hdr.type),
                                 hdr.type,
                                 JASONTALK_THREAD_NAME(hdr.from),
                                 hdr.from,
                                 JASONTALK_THREAD_NAME(hdr.to),
                                 hdr.to,
                                 hdr.length);


                  break;
               }
            case MSPI:
               {
                  //printf(" sending a prompt\n");
                  pthread_mutex_lock (&sio.mutex);
                  unsigned char data = GYRO_STAB_EULER_W_INST_VECT;
                  // write the bytes
                  write (sio.sio_port_fid, &data, 1);



                  // unlock the mutex
                  pthread_mutex_unlock (&sio.mutex);


                  break;
               }

               /* ----------------------------------------
        received timer message at 10 Hz
        ---------------------------------------- */
            case TIMER_UPDATE_RESPONSE:
               {
                  // do housekeeping stuff that needs to be done at a periodic

                  //	    stderr_printf("\nTHANK YOU SIR MAY I HAVE ANOTHER!!!\n\n");

                  // this is a demo, so print a "." on every message
                  // stderr_printf(".");
                  // fflush(stdout);
                  break;
               }

               /* ----------------------------------------
        received SPI (Status Ping) message
        ---------------------------------------- */
            case SPI:
               break;

               /* ----------------------------------------
        received an unknown message type
        ---------------------------------------- */
            default:
               {
                  stderr_printf ("MS_THREAD: (thread %d) ERROR: RECIEVED UNKNOWN MESSAGE TYPE:\n"
                                 "MS_THREAD: got msg type %s (%d)  from proc %s (%d) to proc %s (%d), %d bytes data\n",
                                 MS_THREAD,
                                 JASONTALK_MESSAGE_NAME(hdr.type),
                                 hdr.type,
                                 JASONTALK_THREAD_NAME(hdr.from),
                                 hdr.from,
                                 JASONTALK_THREAD_NAME(hdr.to),
                                 hdr.to,
                                 hdr.length);
                  break;
               }

            }			// switch


      }


}

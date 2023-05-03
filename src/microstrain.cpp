/* ----------------------------------------------------------------------

microstrain interface library

MODIFICATION HISTORY
DATE         WHO             WHAT
-----------  --------------  ----------------------------
2007 02 5    SCM             PORTED PORTIONS OF FILES FROM JCK, SCM, LLW and JCH. for Microstrain.
2007 10 26   LLW             Ported to linux
2007 10 30   LLW             HACK change sign on microstrain ROLL and PITCH as a quick and dirty for Nereus
2007 10 30 LLW HACK add 180 degrees to microstrain heading
2007 11 16 LLW HACK removed
2023 -4 23 jch  modify for habcam

---------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "microstrain.h"
#include "../../dsvimlib/include/imageTalk.h"

#include "../../dsvimlib/include//msg_util.h"
#include "../../dsvimlib/include//launch_timer.h"
#include "../../dsvimlib/include//time_util.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageA2b.h"
#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/IniFile.h"


#define fsat(x,max) ((x)>fabs(max))? fabs(max) : (((x)<(-fabs(max)))? (-fabs(max)): x)

#if __WIN32__
#define strncasecmp strncmpi

#endif

/*--------------------------------------------------------------------------
 * convert2int
 * Convert two adjacent bytes to an integer.
 *
 * parameters:  buffer : pointer to first of two buffer bytes.
 * returns:     the converted value.
 *--------------------------------------------------------------------------*/

int convert2int(char* buffer)
{
   int x = (buffer[0]&LSB_MASK)*256 + (buffer[1]&LSB_MASK);
   return x;
}

/*--------------------------------------------------------------------------
 * convert2short
 * Convert two adjacent bytes to a short.
 *
 * parameters:  buffer : pointer to first of two buffer bytes.
 * returns:     the converted value.
 *--------------------------------------------------------------------------*/

short convert2short(char* buffer)
{
   short x = (buffer[0]&LSB_MASK)*256 + (buffer[1]&LSB_MASK);
   return x;
}

/*--------------------------------------------------------------------------
 * calcChecksum
 * Calculate checksum on a received data buffer.
 *
 * Note: The last two bytes, which contain the received checksum,
 *       are not included in the calculation.
 *
 * parameters:  buffer : pointer to the start of the received buffer.
 *              length - the length (in chars) of the buffer.
 *
 * returns:     the calculated checksum.
 *--------------------------------------------------------------------------*/

int calcChecksum( char* buffer, int length) {
   int CHECKSUM_MASK = 0xFFFF;
   int checkSum, i;

   if (length<4)
      {
         return -1;
      }

   checkSum = buffer[0] & LSB_MASK;
   for (i=1; i<length-2; i = i+2)
      {
         checkSum += convert2int(&buffer[i]);
      }
   return(checkSum & CHECKSUM_MASK);
}


/* -----------------------------------------------------------------------

PARSE INCOMING MICROSTRAIN STRINGS

MODIFICATION HISTORY
DATE         WHO             WHAT
-----------  --------------  ----------------------------
2007/02/05   SCM             MICROSTRAIN PARSING COMMAND
---------------------------------------------------------------------- */



int get_microstrain_cmd_byte(char * str, char msg_type)
{

   switch (msg_type)
      {
      case GYRO_STAB_QUAT_W_INST_VECT:
         str[0] =0x10;
         str[1] =0x00;
         str[2] =0x0C;
         return 31;
      case GYRO_STAB_QUAT_W_INST_VECT_ANGRATE:
         str[0] =0x10;
         str[1] =0x00;
         str[2] = 0x12;
         return   31;
      case GYRO_STAB_EULER_W_INST_VECT:
         str[0] =0x10;
         str[1] =0x00;
         str[2] = 0x31;
         return 23;
      case INST_EULER:
         str[0] =0x10;
         str[1] =0x00;
         str[2] = 0x0D;
         return 11;
      case GYRO_STAB_EULER:
         str[0] =0x10;
         str[1] =0x00;
         str[2] = 0x0E;
         return 11;
      case INST_QUAT:
         str[0] =0x10;
         str[1] =0x00;
         str[2] =0x04;
         return 13;
      case GYRO_QUAT:
         str[0] =0x10;
         str[1] =0x00;
         str[2] = 0x05;
         return 13;
      case GYRO_STAB_VECT:
         str[0] =0x10;
         str[1] =0x00;
         str[2] =0x02;
         return 23;
      case INST_VECT:
         str[0] =0x10;
         str[1] =0x00;
         str[2] =0x03;
         return 23;
      }

}
/* -----------------------------------------------------------------------

PARSE INCOMING MICROSTRAIN STRINGS

MODIFICATION HISTORY
DATE         WHO             WHAT
-----------  --------------  ----------------------------
2007/02/05   SCM             MICROSTRAIN PARSING COMMAND
2007/02/20   SCM             PORTED HEX CODE LOGGING CODE
---------------------------------------------------------------------- */

int microstrain_parse_binary_string(micro_strain_t * microstrain, char * ustr, int len)
{

   static int first_time = 0;
   int i;
   char * str  = (char *) ustr;
   int csum = 0;
   int rxsum = 0;
   rov_time_t time_now;
   //str[len+1] = NULL;

   //  printf("AA\n");

   csum = calcChecksum( str, len);

   //  printf("BB\n");

   rxsum = convert2int(&str[len-2]);

   if (csum != rxsum)
      {
         microstrain->first_time = 0;
         microstrain->num_bad_parse+=1;

         // printf("BAD  CHECKSUM RX=%02X != COMPUTED=%02X\n",0xFF & rxsum, 0xFF & csum);
         return MICROSTRAIN_BAD_CHK_SUM;

      }
   else
      {
         microstrain->num_good_parse+=1;
         //printf("GOOD CHECKSUM RX=%02X == COMPUTED=%02X\n",0xFF & rxsum, 0xFF & csum);
      }

   //timestamp it
   time_now = rov_get_time();
   microstrain->binary_data_time = time_now;
   //Code ported from dvl.cpp written by llw. 2/20/07
   // log the binary dvl string as hex

   int msglen = 0;
   char msg[10240];
   for(i=0;i<len;i++)
      {
         msglen += sprintf(&msg[msglen],"%02X",0xFF &((unsigned int) str[i]));
      }

   //  stderr_printf("MICROSTRAIN: %s\n",msg);


   if(first_time == 0)
      {
         microstrain->mag_field.val[DOF_X]  = 0.0;
         microstrain->mag_field.val[DOF_Y]  = 0.0;
         microstrain->mag_field.val[DOF_Z]  = 0.0;
         microstrain->mag_field.time[DOF_X]   = time_now;
         microstrain->mag_field.time[DOF_Y]   = time_now;
         microstrain->mag_field.time[DOF_Z]   = time_now;

         microstrain->timer_ticks   = 0;

         microstrain->first_time = 1;
         first_time =1;
      }

   switch(str[0])
      {
      case GYRO_STAB_VECT:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->mag_field.val[DOF_X]  = convert2short(&ustr[1])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Y]  = convert2short(&ustr[3])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Z]  = convert2short(&ustr[5])*MAG_SCALE_FACTOR;
            microstrain->mag_field.time[DOF_X]   = time_now;
            microstrain->mag_field.time[DOF_Y]   = time_now;
            microstrain->mag_field.time[DOF_Z]   = time_now;
            microstrain->mag_field.source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->raw_state.acc[DOF_X]  = convert2short(&ustr[7])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Y]  = convert2short(&ustr[9])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Z]  = convert2short(&ustr[11])*ACC_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_ROLL]  = convert2short(&ustr[13])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_PITCH] = convert2short(&ustr[15])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_HDG]   = convert2short(&ustr[17])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.acc_time[DOF_X]   = time_now;
            microstrain->raw_state.acc_time[DOF_Y]   = time_now;
            microstrain->raw_state.acc_time[DOF_Z]   = time_now;

            microstrain->raw_state.acc_source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_time[DOF_ROLL]   = time_now;
            microstrain->raw_state.vel_time[DOF_PITCH]   = time_now;
            microstrain->raw_state.vel_time[DOF_HDG]   = time_now;

            microstrain->raw_state.vel_source[DOF_ROLL] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_PITCH] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_HDG] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->timer_ticks   = convert2int(&ustr[19]);
            //  log_microstrain_binary_string(microstrain);
            //  return MICROSTRAIN_STRING_OK;
            break;
         }
      case INST_VECT:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->mag_field.val[DOF_X] = convert2short(&ustr[1])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Y] = convert2short(&ustr[3])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Z] = convert2short(&ustr[5])*MAG_SCALE_FACTOR;
            microstrain->mag_field.time[DOF_X]   = time_now;
            microstrain->mag_field.source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->mag_field.source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.time[DOF_Y]   = time_now;
            microstrain->mag_field.source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.time[DOF_Z]   = time_now;


            microstrain->raw_state.acc[DOF_X] = convert2short(&ustr[7])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Y] = convert2short(&ustr[9])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Z] = convert2short(&ustr[11])*ACC_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_ROLL]  = convert2short(&ustr[13])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_PITCH] = convert2short(&ustr[15])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_HDG]   = convert2short(&ustr[17])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.acc_time[DOF_X]   = time_now;
            microstrain->raw_state.acc_time[DOF_Y]   = time_now;
            microstrain->raw_state.acc_time[DOF_Z]   = time_now;

            microstrain->raw_state.acc_source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->raw_state.vel_time[DOF_ROLL]   = time_now;
            microstrain->raw_state.vel_time[DOF_PITCH]   = time_now;
            microstrain->raw_state.vel_time[DOF_HDG]   = time_now;

            microstrain->raw_state.vel_source[DOF_ROLL] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_PITCH] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_HDG] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->timer_ticks   = convert2int(&ustr[19]);
            // log_microstrain_binary_string(microstrain);
            // return MICROSTRAIN_STRING_OK;
            break;
         }
      case INST_EULER:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->raw_state.pos[DOF_ROLL]  = convert2short(&ustr[1])*EULER_SCALE_FACTOR;
            microstrain->raw_state.pos[DOF_PITCH] = convert2short(&ustr[3])*EULER_SCALE_FACTOR;
            microstrain->raw_state.pos[DOF_HDG]   = convert2short(&ustr[5])*EULER_SCALE_FACTOR;
            microstrain->raw_state.pos_time[DOF_ROLL]   = time_now;
            microstrain->raw_state.pos_time[DOF_PITCH]   = time_now;
            microstrain->raw_state.pos_time[DOF_HDG]   = time_now;

            microstrain->raw_state.pos_source[DOF_ROLL] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.pos_source[DOF_PITCH] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.pos_source[DOF_HDG] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->timer_ticks   = convert2int(&ustr[7]);
            // return MICROSTRAIN_STRING_OK;
            break;

         }
      case GYRO_STAB_EULER:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->raw_state.pos[DOF_ROLL]  = convert2short(&ustr[1])*EULER_SCALE_FACTOR;
            microstrain->raw_state.pos[DOF_PITCH] = convert2short(&ustr[3])*EULER_SCALE_FACTOR;
            microstrain->raw_state.pos[DOF_HDG]   = convert2short(&ustr[5])*EULER_SCALE_FACTOR;
            microstrain->raw_state.pos_time[DOF_ROLL]   = time_now;
            microstrain->raw_state.pos_time[DOF_PITCH]   = time_now;
            microstrain->raw_state.pos_time[DOF_HDG]   = time_now;

            microstrain->raw_state.pos_source[DOF_ROLL] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.pos_source[DOF_PITCH] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.pos_source[DOF_HDG] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->timer_ticks   = convert2int(&ustr[7]);
            //return MICROSTRAIN_STRING_OK;
            break;

         }
      case INST_QUAT:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->quaternion.source[QUAT_0] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_0]   = time_now;
            microstrain->quaternion.source[QUAT_1] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_1]   = time_now;
            microstrain->quaternion.source[QUAT_2] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_2]   = time_now;
            microstrain->quaternion.source[QUAT_3] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_3]   = time_now;

            microstrain->quaternion.val[QUAT_0]   = convert2short(&ustr[1])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_1]   = convert2short(&ustr[3])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_2]   = convert2short(&ustr[5])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_3]   = convert2short(&ustr[7])*QUAT_SCALE_FACTOR;

            microstrain->timer_ticks   = convert2int(&ustr[9]);
            // log_microstrain_binary_string(microstrain);
            // return MICROSTRAIN_STRING_OK;
            break;

         }
      case GYRO_QUAT:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->quaternion.source[QUAT_0] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_0]   = time_now;
            microstrain->quaternion.source[QUAT_1] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_1]   = time_now;
            microstrain->quaternion.source[QUAT_2] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_2]   = time_now;
            microstrain->quaternion.source[QUAT_3] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_3]   = time_now;

            microstrain->quaternion.val[QUAT_0]   = convert2short(&ustr[1])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_1]   = convert2short(&ustr[3])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_2]   = convert2short(&ustr[5])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_3]   = convert2short(&ustr[7])*QUAT_SCALE_FACTOR;
            microstrain->timer_ticks   = convert2int(&ustr[9]);
            //log_microstrain_binary_string(microstrain);
            //return MICROSTRAIN_STRING_OK;
            break;

         }

      case GYRO_STAB_EULER_W_INST_VECT:
         {

            microstrain->most_recent_string_time   = rov_get_time();
            // 2007 10 30 LLW HACK change sign on microstrain ROLL and PITCH as a quick and dirty for Nereus
            // 2007 11 16 LLW HACK corrected
            microstrain->raw_state.pos[DOF_ROLL]   = convert2short(&ustr[1])*EULER_SCALE_FACTOR;
            // 2007 10 30 LLW HACK change sign on microstrain ROLL and PITCH as a quick and dirty for Nereus
            // 2007 11 16 LLW HACK corrected
            microstrain->raw_state.pos[DOF_PITCH]  = convert2short(&ustr[3])*EULER_SCALE_FACTOR;
            // 2007 10 31 LLW HACK add 180 degrees to microstrain heading
            // 2007 11 16 LLW HACK corrected
            microstrain->raw_state.pos[DOF_HDG]    = fmod(convert2short(&ustr[5])*EULER_SCALE_FACTOR+720.0,360.0);

            microstrain->raw_state.pos_time[DOF_ROLL]  = time_now;
            microstrain->raw_state.pos_time[DOF_PITCH] = time_now;
            microstrain->raw_state.pos_time[DOF_HDG]   = time_now;
            microstrain->raw_state.pos_source[DOF_ROLL]   = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.pos_source[DOF_PITCH]  = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.pos_source[DOF_HDG]    = SRC_VEHICLE_MICROSTRAIN;

            microstrain->raw_state.acc[DOF_X] = convert2short(&ustr[7])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Y] = convert2short(&ustr[9])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Z] = convert2short(&ustr[11])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc_time[DOF_X]   = time_now;
            microstrain->raw_state.acc_time[DOF_Y]   = time_now;
            microstrain->raw_state.acc_time[DOF_Z]   = time_now;
            microstrain->raw_state.acc_source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->raw_state.vel[DOF_ROLL]  = convert2short(&ustr[13])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_PITCH] = convert2short(&ustr[15])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_HDG]   = convert2short(&ustr[17])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel_time[DOF_ROLL]    = time_now;
            microstrain->raw_state.vel_time[DOF_PITCH]   = time_now;
            microstrain->raw_state.vel_time[DOF_HDG]     = time_now;
            microstrain->raw_state.vel_source[DOF_ROLL]  = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_PITCH] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_HDG]   = SRC_VEHICLE_MICROSTRAIN;

            microstrain->timer_ticks   = convert2int(&ustr[19]);
            //log_microstrain_binary_string(microstrain);
            //return MICROSTRAIN_STRING_OK;
            break;
         }

      case GYRO_STAB_QUAT_W_INST_VECT:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->quaternion.val[QUAT_0]   = convert2short(&ustr[1])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_1]   = convert2short(&ustr[3])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_2]   = convert2short(&ustr[5])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_3]   = convert2short(&ustr[7])*QUAT_SCALE_FACTOR;

            microstrain->mag_field.val[DOF_X] = convert2short(&ustr[9])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Y] = convert2short(&ustr[11])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Z] = convert2short(&ustr[13])*MAG_SCALE_FACTOR;
            microstrain->mag_field.time[DOF_X]   = time_now;
            microstrain->mag_field.source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->mag_field.source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.time[DOF_Y]   = time_now;
            microstrain->mag_field.source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.time[DOF_Z]   = time_now;

            microstrain->raw_state.acc[DOF_X] = convert2short(&ustr[15])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Y] = convert2short(&ustr[17])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Z] = convert2short(&ustr[19])*ACC_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_ROLL]  = convert2short(&ustr[21])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_PITCH] = convert2short(&ustr[23])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_HDG]   = convert2short(&ustr[25])*ROT_RATE_SCALE_FACTOR;

            microstrain->raw_state.acc_time[DOF_X]   = time_now;
            microstrain->raw_state.acc_time[DOF_Y]   = time_now;
            microstrain->raw_state.acc_time[DOF_Z]   = time_now;

            microstrain->raw_state.acc_source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_time[DOF_ROLL]   = time_now;
            microstrain->raw_state.vel_time[DOF_PITCH]   = time_now;
            microstrain->raw_state.vel_time[DOF_HDG]   = time_now;

            microstrain->raw_state.vel_source[DOF_ROLL] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_PITCH] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_HDG] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->timer_ticks   = convert2int(&ustr[27]);

            //microstrain->good_parse += 1;
            //log_microstrain_binary_string(microstrain);
            //return MICROSTRAIN_STRING_OK;
            break;
         }
      case GYRO_STAB_QUAT_W_INST_VECT_ANGRATE:
         {
            microstrain->most_recent_string_time = rov_get_time();
            microstrain->quaternion.val[QUAT_0]   = convert2short(&ustr[1])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_1]   = convert2short(&ustr[3])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_2]   = convert2short(&ustr[5])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.val[QUAT_3]   = convert2short(&ustr[7])*QUAT_SCALE_FACTOR;
            microstrain->quaternion.source[QUAT_0] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_0]   = time_now;
            microstrain->quaternion.source[QUAT_1] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_1]   = time_now;
            microstrain->quaternion.source[QUAT_2] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_2]   = time_now;
            microstrain->quaternion.source[QUAT_3] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->quaternion.time[QUAT_3]   = time_now;
            microstrain->mag_field.val[DOF_X] = convert2short(&ustr[9])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Y] = convert2short(&ustr[11])*MAG_SCALE_FACTOR;
            microstrain->mag_field.val[DOF_Z] = convert2short(&ustr[13])*MAG_SCALE_FACTOR;
            microstrain->mag_field.source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.time[DOF_X]   = time_now;
            microstrain->mag_field.source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.time[DOF_Y]   = time_now;
            microstrain->mag_field.source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->mag_field.time[DOF_Z]   = time_now;

            microstrain->raw_state.acc[DOF_X] = convert2short(&ustr[15])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Y] = convert2short(&ustr[17])*ACC_SCALE_FACTOR;
            microstrain->raw_state.acc[DOF_Z] = convert2short(&ustr[19])*ACC_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_ROLL]  = convert2short(&ustr[21])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_PITCH] = convert2short(&ustr[23])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.vel[DOF_HDG]   = convert2short(&ustr[25])*ROT_RATE_SCALE_FACTOR;
            microstrain->raw_state.acc_time[DOF_X]   = time_now;
            microstrain->raw_state.acc_time[DOF_Y]   = time_now;
            microstrain->raw_state.acc_time[DOF_Z]   = time_now;

            microstrain->raw_state.acc_source[DOF_X] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Y] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.acc_source[DOF_Z] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_time[DOF_ROLL]   = time_now;
            microstrain->raw_state.vel_time[DOF_PITCH]   = time_now;
            microstrain->raw_state.vel_time[DOF_HDG]   = time_now;

            microstrain->raw_state.vel_source[DOF_ROLL] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_PITCH] = SRC_VEHICLE_MICROSTRAIN;
            microstrain->raw_state.vel_source[DOF_HDG] = SRC_VEHICLE_MICROSTRAIN;

            microstrain->timer_ticks   = convert2int(&ustr[27]);

            break;
         }


      }

   
   return MICROSTRAIN_STRING_OK;
}


int process_microstrain_binary_string(micro_strain_t * microstrain, unsigned char * ustr, int len)
{
   int var;
   var = microstrain_parse_binary_string(microstrain, (char *)  ustr, len);
   return var;
}

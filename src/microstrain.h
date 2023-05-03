/* ----------------------------------------------------------------------

   MICROSTRAIN interface library



   MODIFICATION HISTORY
   DATE         WHO             WHAT
   -----------  --------------  ----------------------------
   05 FEB 2007  Stephen Martin  Created and Written
   2007 10 26   LLW             Ported to linux   
   2018         jch             modify
   ---------------------------------------------------------------------- */

#ifndef MICROSTRAIN_INC
#define MICROSTRAIN_INC

#include <stdio.h>
#include "../../dsvimlib/include/imageTalk.h"
//#include "con_mat.h"

#define MICROSTRAIN_POLL_INTERVAL	0.10

#define MICROSTRAIN_STRING_OK 0
#define MICROSTRAIN_BAD_CHK_SUM 1
#define MICROSTRAIN_STRING_UNKNOWN 2
#define QUAT_0 0
#define QUAT_1 1
#define QUAT_2 2
#define QUAT_3 3

#define MAG_SCALE_FACTOR      2000.0/32768000.0
#define EULER_SCALE_FACTOR    360.0/65536.0
#define QUAT_SCALE_FACTOR     1.0/8192.0
#define ACC_SCALE_FACTOR      7000.0/32768000.0
#define ROT_RATE_SCALE_FACTOR 8500.0/32768000.0

#define GYRO_STAB_QUAT_W_INST_VECT 0x0C
#define GYRO_STAB_QUAT_W_INST_VECT_ANGRATE 0x12
#define GYRO_STAB_EULER_W_INST_VECT 0x31
#define INST_EULER 0x0D
#define GYRO_STAB_EULER 0x0E
#define INST_QUAT 0x04
#define GYRO_QUAT 0x05
#define GYRO_STAB_VECT 0x02
#define INST_VECT 0x03
//#define GYRO_STAB_QUAT_W_INST_VECT 0
//#define GYRO_STAB_QUAT_W_INST_VECT_ANGRATE 1
//#define GYRO_STAB_EULER_W_INST_VECT 2
//#define INST_EULER 3
//#define GYRO_STAB_EULER 4
//#define INST_QUAT 5
//#define GYRO_QUAT 6
//#define GYRO_STAB_VECT 7
//#define INST_VECT 8
/*typedef enum
  {
    GYRO_STAB_QUAT_W_INST_VECT,
    GYRO_STAB_QUAT_W_INST_VECT_ANGRATE,
    GYRO_STAB_EULER_W_INST_VECT,
    INST_EULER,
    GYRO_STAB_EULER,
    INST_QUAT,
    GYRO_QUAT,
    GYRO_STAB_VECT,
    INST_VECT
  } msg_type_t;*/

#define LSB_MASK 0xFF
#define MSB_MASK 0xFF00

typedef struct
{
  double val[4];
  rov_time_t    time[4];      /* time original data was received */
  data_source_t source[4];

} quaternion_t;

typedef struct
{
  double  val[3];
  rov_time_t    time[3];      /* time original data was received */
  data_source_t source[3];
} mag_field_t;

typedef struct
{
  char cmd_byte;
  int cmd_len;
  char send_str[25];
}microstrain_msg_t;


typedef struct
{
  //char           most_recent_string[OCTANS_NUM_STRING_HISTORY][MAX_OCTANS_STR_LEN];    /* most recent binary string */
  //int            most_recent_string_status[OCTANS_NUM_STRING_HISTORY];                 /* parse status */
  vehicle_state_t raw_state;
  mag_field_t mag_field;
  quaternion_t quaternion;
  rov_time_t  binary_data_time;
  int timer_ticks;
  microstrain_msg_t cmd_msg;
  int num_bad_parse;
  int num_good_parse;
  rov_time_t most_recent_string_time;
  int first_time;
} micro_strain_t;

extern int microstrain_parse_binary_string(micro_strain_t * microstrain, char * ustr, int len);
extern int process_microstrain_binary_string(micro_strain_t * microstrain, char * ustr, int len);
extern int get_microstrain_cmd_byte(char * str, char msg_type);

extern void log_microstrain_as_xbow_string(micro_strain_t * microstrain, char *timeString,double declination, int declination_type, int my_address);
#endif

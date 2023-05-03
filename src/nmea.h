/* ----------------------------------------------------------------------

   header file for using nmea data 

   Modification History:
   DATE        AUTHOR  COMMENT
  02 dec 04   JCH     Created and Written
  
   ---------------------------------------------------------------------- */


#ifndef NMEA_INC
#define NMEA_INC
#include "../../dsvimlib/include/convert.h"

#define	PVGGA	"$LCGGA"
#define  HEHDT	"$HEHDT"

typedef struct {
	double	latitude; // units are radians
	double	longitude;
	int		num_satellites;
	double	hdop;
	double	altitude;
	double	geoid_separation;
	double	age;
	int		valid;
	double	utc_time;


}	gpgga_t;

typedef struct {
	double	heading;
	int		valid;
} hehdt_t;
	
extern unsigned short compute_nmea_checksum(char *input_string);

extern gpgga_t	parse_gpgga(char *input_string, bool ignoreCheck);
extern gpgga_t	parse_lcgga(char *input_string);

extern hehdt_t	parse_hehdt(char *input_string);
extern hehdt_t	parse_pvhdg(char *input_string);
extern gpgga_t	parse_mvgga(char *input_string);


#endif

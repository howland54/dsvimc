/* ansii c headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>


#include "nmea.h"
/* ----------------------------------------------------------------------

** nmea parsing and writing routines
** 
**
** Description: the functions which do nmea checksums and the like
**
**
** History:
**       Date       Who      Description
**      ------      ---      --------------------------------------------
**      12/2004      jch      Create 

---------------------------------------------------------------------- */

unsigned short compute_nmea_checksum(char *msg)
{
	unsigned short checksum = 0;
	int	slen = strlen(msg);
   for(int charplace = 0; charplace < slen; charplace++)
      {
         if(msg[charplace] == '*')
            {
               break;
            }
         switch (msg[charplace])
            {
            case '$':

               break;
            case '*':

               continue;

            default:{
                     if(checksum == 0)
                        {
                           checksum = (unsigned short)msg[charplace];
                        }
                     else
                        {
                           checksum = checksum ^ (unsigned short)msg[charplace];
                        }
                     break;
                  }
            }
      }
	return checksum;

}

hehdt_t	parse_hehdt(char *input_string)
{
	hehdt_t	return_hehdt;
	return_hehdt.valid = 0;
	// first, run through the data and check the checksum
	unsigned short	computed_checksum = compute_nmea_checksum(input_string);
	char *checksum_place = strstr(input_string,"*");
	if(checksum_place){
         unsigned int read_checksum;
         int items = sscanf(checksum_place+1,"%02x",&read_checksum);

         if(items)
            {

               if(read_checksum == computed_checksum){
                     // the checksum is valid
                     double	heading;
                     char		headunits;
                     // parse the hehdt
                     items = sscanf(input_string,"$HEHDT,%lf,%c",&heading,&headunits);
                     if(items == 2){
                           if(headunits == 'T')
                              {
                                 return_hehdt.heading = 	heading * DTOR;
                                 return_hehdt.valid = 1;
                              }
                        }
                  }

            }

      }
	return return_hehdt;

}

hehdt_t	parse_pvhdg(char *input_string)
{
	hehdt_t	return_hehdt;
	return_hehdt.valid = 0;
	// first, run through the data and check the checksum
	unsigned short	computed_checksum = compute_nmea_checksum(input_string);
	char *checksum_place = strstr(input_string,"*");
   if(checksum_place)
      {
         unsigned int read_checksum;
         int items = sscanf(checksum_place+1,"%02x",&read_checksum);

         if(items){

               if(read_checksum == computed_checksum)
                  {
                     // the checksum is valid
                     double	heading;

                     // parse the hehdt
                     items = sscanf(input_string,"$PVHDG,%lf",&heading);
                     if(items == 1)
                        {

                           return_hehdt.heading = 	heading * DTOR;
                           return_hehdt.valid = 1;

                        }
                  }

            }

      }
	return return_hehdt;

}
gpgga_t	parse_gpgga(char *input_string, bool ignoreTheChecksum)

{
	gpgga_t	return_gpgga;
	return_gpgga.valid = 0;
   char *checksum_place = input_string;
   unsigned short	computed_checksum = 0;
	// first, run through the data and check the checksum
   if(!ignoreTheChecksum)
      {
         computed_checksum = compute_nmea_checksum(input_string);
         checksum_place = strstr(input_string,"*");
      }
   if(checksum_place)
      {
         unsigned int read_checksum;
         int items = 0;
         if(!ignoreTheChecksum)
            {
               items  = sscanf(checksum_place+1,"%02x",&read_checksum);
            }
         else
            {
               items = 1;
               read_checksum = computed_checksum;
            }

         if(items)
            {

               if(read_checksum != computed_checksum)
                  {
                     return return_gpgga;
                  }
               else
                  {  // the checksum is valid

                     // parse the gpgga
                     int	utc_hrs,utc_mins,latd,lond,nsat;
                     double	utc_secs,latmin,lonmin,hdop,alt;
                     char		le_or_w,ln_or_s,altunits;
                     int      n_or_s,e_or_w;
                     items = sscanf(input_string,"$GPGGA,%2d%2d%lf,%2d%lf,%c,%3d%lf,%c,%d,%lf,%lf,%c,",
                                    &utc_hrs,&utc_mins,&utc_secs,&latd,&latmin,&ln_or_s,&lond,&lonmin,&le_or_w,
                                    &nsat,&hdop,&alt,&altunits);

                     if(items != 13)
                        {
                           return return_gpgga;
                        }
                     if(ln_or_s == 'S')
                        {
                           n_or_s = -1;
                        }
                     else{
                           n_or_s = 1;
                        }
                     if(le_or_w == 'W')
                        {
                           e_or_w = -1;
                        }
                     else{
                           e_or_w = 1;
                        }
                     return_gpgga.latitude = (latd + latmin/60.0)*n_or_s*DTOR;
                     return_gpgga.longitude = (lond + lonmin/60.0)*e_or_w*DTOR;
                     return_gpgga.num_satellites = nsat;
                     return_gpgga.hdop = hdop;
                     if((altunits == 'f') || (altunits == 'F'))
                        {
                           alt *= METERS_FOOT;
                        }
                     return_gpgga.altitude = alt;

                     return_gpgga.valid = 1;
                     return_gpgga.utc_time = utc_hrs*3600 + utc_mins*60.0 + utc_secs;

                     return return_gpgga;
                  }
            }
         else{
               return return_gpgga;
            }
      }
	else{
         return return_gpgga;
      }


}

gpgga_t	parse_lcgga(char *input_string)

{
   gpgga_t	return_gpgga;
   return_gpgga.valid = 0;
   // first, run through the data and check the checksum
   unsigned short	computed_checksum = compute_nmea_checksum(input_string);
   char *checksum_place = strstr(input_string,"*");
   if(checksum_place)
      {
         unsigned int read_checksum;
         int items = sscanf(checksum_place+1,"%02x",&read_checksum);

         if(items)
            {

               if(read_checksum != computed_checksum)
                  {
                     return return_gpgga;
                  }
               else
                  {  // the checksum is valid

                     // parse the gpgga
                     int	utc_hrs,utc_mins,latd,lond,nsat;
                     double	utc_secs,latmin,lonmin,hdop,alt;
                     char		le_or_w,ln_or_s,altunits;
                     int      n_or_s,e_or_w;
                     items = sscanf(input_string,"$LCGGA,%2d%2d%lf,%2d%lf,%c,%3d%lf,%c,%d,%lf,%lf,%c,",
                                    &utc_hrs,&utc_mins,&utc_secs,&latd,&latmin,&ln_or_s,&lond,&lonmin,&le_or_w,
                                    &nsat,&hdop,&alt,&altunits);

                     if(items != 13)
                        {
                           return return_gpgga;
                        }
                     if(ln_or_s == 'S')
                        {
                           n_or_s = -1;
                        }
                     else{
                           n_or_s = 1;
                        }
                     if(le_or_w == 'W')
                        {
                           e_or_w = -1;
                        }
                     else{
                           e_or_w = 1;
                        }
                     return_gpgga.latitude = (latd + latmin/60.0)*n_or_s*DTOR;
                     return_gpgga.longitude = (lond + lonmin/60.0)*e_or_w*DTOR;
                     return_gpgga.num_satellites = nsat;
                     return_gpgga.hdop = hdop;
                     if((altunits == 'f') || (altunits == 'F'))
                        {
                           alt *= METERS_FOOT;
                        }
                     return_gpgga.altitude = alt;

                     return_gpgga.valid = 1;
                     return_gpgga.utc_time = utc_hrs*3600 + utc_mins*60.0 + utc_secs;

                     return return_gpgga;
                  }
            }
         else{
               return return_gpgga;
            }
      }
   else{
         return return_gpgga;
      }


}

gpgga_t	parse_mvgga(char *input_string)

{
	gpgga_t	return_gpgga;
	return_gpgga.valid = 0;
	// first, run through the data and check the checksum
	unsigned short	computed_checksum = compute_nmea_checksum(input_string);
	char *checksum_place = strstr(input_string,"*");
	if(checksum_place){
         unsigned int read_checksum;
         int items = sscanf(checksum_place+1,"%02x",&read_checksum);

         if(items){

               if(read_checksum != computed_checksum)
                  {
                     return return_gpgga;
                  }
               else
                  {  // the checksum is valid

                     // parse the gpgga
                     int	utc_hrs,utc_mins,latd,lond;
                     double	utc_secs,latmin,lonmin;
                     char		le_or_w,ln_or_s;
                     int      n_or_s,e_or_w;
                     // find the first comma

                     char *comma_loc = strstr(input_string,",");
                     if(!comma_loc)
                        {
                           return return_gpgga;
                        }
                     items = sscanf(comma_loc,",%2d%2d%lf,%2d%lf,%c,%3d%lf,%c,",
                                    &utc_hrs,&utc_mins,&utc_secs,&latd,&latmin,&ln_or_s,&lond,&lonmin,&le_or_w);

                     if(items != 9)
                        {
                           return return_gpgga;
                        }
                     if(ln_or_s == 'S')
                        {
                           n_or_s = -1;
                        }
                     else{
                           n_or_s = 1;
                        }
                     if(le_or_w == 'W')
                        {
                           e_or_w = -1;
                        }
                     else{
                           e_or_w = 1;
                        }
                     return_gpgga.latitude = (latd + latmin/60.0)*n_or_s*DTOR;
                     return_gpgga.longitude = (lond + lonmin/60.0)*e_or_w*DTOR;


                     return_gpgga.valid = 1;
                     return_gpgga.utc_time = utc_hrs*3600 + utc_mins*60.0 + utc_secs;

                     return return_gpgga;
                  }
            }
         else
            {
               return return_gpgga;
            }
      }
   else
      {
         return return_gpgga;
      }


}


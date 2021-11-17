#include <Arduino.h>
#include "mygps.h"


namespace MyGPS{


NMEADecoder::NMEADecoder(){

     gps_time[0] = 0;
}

char * NMEADecoder::getLatDMS(){
    return lat_degrees;
}



char * NMEADecoder::getLonDMS(){
    if(strlen(lon_degrees)==8){
        strcpy(lon_DMS,"0");
    }
    strcpy(lon_DMS+1, lon_degrees);
    return lon_DMS;
}


void NMEADecoder::mygps_loop(){
    if(gps_buf_ptr == 0)
      memset( gps_buf, '\0', 200 );

    while (Serial2.available()) {
      char cc = char(Serial2.read());
      gps_buf[gps_buf_ptr++] = cc;
      if(cc=='\n'){
        gps_buf[gps_buf_ptr] = 0;
        gps_buf_ptr=0;
        //Serial.printf("%s\r\n", gps_buf);
        
        char f1[100];   memset(f1,0,100); 
        char f2[100];   memset(f2,0,100); 
        char f3[100];   memset(f3,0,100); 
        char f4[100];   memset(f4,0,100); 
        char f5[100];   memset(f5,0,100); 
        char f6[100];   memset(f6,0,100); 
        char f7[100];   memset(f7,0,100); 
        char f8[100];   memset(f8,0,100); 
        char all[100];  memset(all,0,100); 
        char all2[100]; memset(all2,0,100); 
        char num_sats[100]; memset(num_sats,0,100); 
        char lat_str[100];  memset(lat_str,0,100); 
        char lon_str[100];  memset(lon_str,0,100); 
        
        sscanf(gps_buf,"%[^','],%20[^','],%20[^','],%20[^','],%20[^','],%20[^','],%20[^','],%20[^','],%s", 
                f1,f2,f3,f4,f5,f6,f7,f8,all);
        if(strcmp(f1,"$GPGSV")==0){
            //Serial.printf("the fourth substring in GPGSV is %s\r\n",f4);
            //split f4 if it has *    
            //sscanf(f4,"%[^'*']",num_sats);
            //num_satellites = atoi(num_sats);   
            //Serial.printf("num_satellites=%d\r\n",num_satellites);     
        }
        if(strcmp(f1,"$GPGGA")==0){
            sprintf(lat_degrees,"%.2f%s",atof(f3),f4);
            sprintf(lon_degrees,"%.2f%s",atof(f5),f6);

            if(strlen(lon_degrees)==8){
                char bb[20];
                strcpy(bb,lon_degrees);
                strcpy(lon_degrees+1,bb); // shift caracters 1 place to the right
                lon_degrees[0] = '0'; // place a 0 in front
            }

            if(atoi(f7) > 0)
                LastFixAtMsecs = millis();
                //Serial.printf("last fix at %lu msecs, %lu msecs since now\r\n",LastFixAtMsecs, millis()-LastFixAtMsecs );
            
            num_satellites = atoi(f8);

            strcpy(gps_time, f2);
            gps_time[6] = 0;
            
        }

        
      }  
    }
}


}
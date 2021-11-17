#ifndef MYGPS_H
#define MYGPS_H

namespace MyGPS{

    class NMEADecoder{
        public:

            NMEADecoder();

            int num_satellites = 0;
            char lat_degrees[100] ;
            char lon_degrees[100] ;
            char gps_time[9];

            unsigned long LastFixAtMsecs; // = 1440*60*1000;// msecs in one day

            void mygps_loop();

            char * getLatDMS(); // get a format like   4303.51N  
            char * getLonDMS(); // get e format like  01036.59E

        private:
            char gps_buf[200];
            int gps_buf_ptr = 0;
            char lat_DMS[100] ;
            char lon_DMS[100] ;
            
    };

    
}

#endif
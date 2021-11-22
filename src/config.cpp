#include <Arduino.h>
#include <config.h>
#include <tools.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

namespace Config_ns{

    std::vector<Entry>  Entries = {
        // Access Point
        {"ApSsid", CFG_AP_SSID, "String", "The Access Point SSID e.g. ESP32A_APRS "},
        {"ApPwd", CFG_AP_PWD, "String", "The Access Point SSID e.g. 12345678 "},
        {"ApPin", String(CFG_AP_PIN), "Int", "The Access Point Activation Pin e.g. 38"},
        // Client or Server        
        {"IsClientMode", CFG_IS_CLIENT_MODE ? "true" : "false", "Bool", "Modalita' Client (if checked) o Server"},
        {"UseDisplay", CFG_USE_DISPLAY ? "true" : "false", "Bool", "Use OLED display (if checked) or not"},
        // lora protocol parameters
        {"LoraFreq", String(CFG_LORA_FREQ), "Long", "Set the operating frequency e.g. 433775000"},
        {"LoraBw", String(CFG_LORA_BW), "Long", "Set the LoRa Bandwith e.g. 125000"},
        {"LoraSf", String(CFG_LORA_SF), "Long", "Set the LoRa Spreading Factor e.g. 12"},
        {"LoraCodingRate", String(CFG_LORA_CR), "Long", "Set the LoRa Coding Rate e.g. 7"},
        {"LoraPower", String(CFG_LORA_PWR), "Long", "Set the LoRa Power Level in dBm e.g. 20"},
        {"LoraEnableCrc", CFG_LORA_ENABLE_CRC ? "true" : "false", "Bool", "LoRa CRC check enabled"},
        // lora pinouts
        {"LoraPinSs", String(CFG_LORA_PIN_SS), "Int", "Set the LoRa SS Pin e.g. 18"},
        {"LoraPinRst", String(CFG_LORA_PIN_RST), "Int", "Set the LoRa RST Pin e.g. 14"},
        {"LoraPinDio0", String(CFG_LORA_PIN_DIO0), "Int", "Set the LoRa DIO Pin e.g. 26"},
        {"LoraUseIsr", CFG_LORA_USE_ISR ? "true" : "false", "Bool", "LoRa usage of ISR (stream mode e.g. for speech"},
        // Bluetooth settings
        {"BtName", CFG_BT_NAME, "String", "Bluetooth Device name e.g. loraprs"},
        {"BtEnableBle", CFG_BT_USE_BLE ? "true" : "false", "Bool", "Use bluetooth low energy (for ios devices)"},
        // aprs configuration
        {"AprsHost", "iz5oqo.duckdns.org", "String", "APRS host name e.g. foo.a_dns.org"},
        {"AprsPort", "14580", "Long", "APRS port number e.g. 14580"},
        {"AprsLogin", CFG_APRS_LOGIN, "String", "APRS login e.g. aprs_login"},
        {"AprsPass", CFG_APRS_PASS, "String", "APRS password e.g. aprs_password"},
        {"AprsFilter", CFG_APRS_FILTER, "String", "Multiple filters are space separated e.g. r/35.60/139.80/25"},
        // aprs payload string
        {"AprsSrcAddr", CFG_APRS_SRC_ADDR, "String", "The Source Address e.g. IW5ALZ-7"},
        {"AprsAddrSep1", CFG_APRS_ADDR_SEP, "String", "The first separator e.g. >"},
        {"AprsSwVerNum", CFG_APRS_SW_VER, "String", "The SW version number e.g. APZMDM"},
        {"AprsAddrSep2", CFG_APRS_ADDR_SEP2, "String", "The second separator e.g. ,"},
        {"AprsDgptrCode", CFG_APRS_DGPTR_CODE, "String", "The digipeating code e.g. WIDE1-1"},
        {"AprsMsgType", CFG_APRS_MSG_TYPE, "String", "The message type e.g. :"},
        {"AprsMsgFmt", CFG_APRS_MSG_FMT, "String", "The message format e.g. !"},
        {"AprsLat", CFG_APRS_LAT, "String", "The latitude e.g. 4319.15N"},
        {"AprsSymTable", CFG_APRS_SYM_TABLE, "String", "The Symbol Table e.g. /"},
        {"AprsLon", CFG_APRS_LON, "String", "The longitude e.g. 01120.61E"},
        {"AprsSymbol", CFG_APRS_SYMBOL, "String", "The symbol used e.g. -"},
        {"AprsMsg", CFG_APRS_MSG, "String", "The message e.g. LoRA Tracker 433.775MHz/BW125/SF10/CR5"},
        {"AprsRawBeacon", CFG_APRS_RAW_BKN, "String", "The complete beacon payload string e.g. IW5ALZ-7>APZMDM,WIDE1-1:!4319.15N/01120.61E-LoRA Tracker 433.775MHz/BW125/SF10/CR5"},
        {"BeaconPeriodMin", "20", "Int", "Beacon period in minutes e.g. 20"},
        // WiFi Credentials
        {"WifiSsid", CFG_WIFI_SSID, "String", "WiFi SSID to connect to e.g. wifi_network"},
        {"WifiKey", CFG_WIFI_KEY, "String", "WiFi password e.g. secret_password"},
        // frequency correction
        {"AutoFreqCorr", CFG_FREQ_CORR ? "true" : "false", "Bool", "correct own frequency based on received packet frequency deviation"},
        {"AutoFreqCorrHz", String(CFG_FREQ_CORR_DELTA), "Int", "correct when difference is larger than this value"},
        // aprs logic
        {"SignalReport", "true", "Bool", "append signal report on server side for the client to be sent to APRS-IS"},
        {"PersistAprsConn", CFG_PERSISTENT_APRS ? "true" : "false", "Bool", "keep aprs-is connection active all the time"},
        {"EnableRfToIs", CFG_RF_TO_IS ? "true" : "false", "Bool", "enable RF to APRS-IS submission"},
        {"EnableIsToRf", CFG_IS_TO_RF ? "true" : "false", "Bool", "enable APRS-IS to RF submission"},
        {"EnableRepeater", CFG_DIGIREPEAT ? "true" : "false", "Bool", "digirepeat incoming packets based on WIDEn-n paths"},
        {"EnableBeacon", CFG_BEACON ? "true" : "false", "Bool", "send AprsRawBeacon to RF and APRS-IS if EnableRfToIs is true"},
        // GPS    
        {"UseGPS", CFG_USE_GPS ? "true" : "false", "Bool", "Use GPS in Beacon Mode"},
        {"GPSRxPin", String(CFG_GPS_RX_PIN), "Int", "The ESP32 pin which GPS Rx is connected to e.g. 15"},
        {"GPSTxPin", String(CFG_GPS_TX_PIN), "Int", "The ESP32 pin which GPS Rx is connected to e.g. 15"},
        // external ptt tx control
        {"PttEnable", CFG_PTT_ENABLE ? "true" : "false", "Bool", "enable external ptt control"},
        {"PttPin", String(CFG_PTT_PIN), "Int", "ESP32 pin to set high on transmit e.g. 12"},
        {"PttTxDelayMs", String(CFG_PTT_TX_DELAY_MS), "Long", "ptt tx delay e.g. 50"},
        {"PttTxTailMs", String(CFG_PTT_TX_TAIL_MS), "Long", "ptt tx tail e.g. 10"}
    };

    void ConfigClass::Init(String _filename){
        Serial.println("ConfigClass constructor");
        filename = _filename;
       if(!SPIFFS.begin(true)){
           Serial.println("An Error has occurred while mounting SPIFFS");
           return;
        }
        LoadEntriesFromSPIFFS();
    };

    // return an Entry specified by its key
    Entry * ConfigClass::GetEntryByKey(String key){

        for (std::vector<Entry>::iterator it = Entries.begin() ; 
            it != Entries.end(); ++it){
                if(it->key == key){
                    return  &(*it);
                }
            }
        return NULL;
    }

    Entry ConfigClass::operator [](int i) const    {
        return Entries[i];
    }
    
    Entry * ConfigClass::operator [](String key)     {
        return GetEntryByKey(key);
    }

   //Load from SPIFFS to Entries
   void ConfigClass::LoadEntriesFromSPIFFS(){
        Serial.printf("LoadEntriesFromSPIFFS\r\n");
        // Open file for reading 
        File file = SPIFFS.open(filename, FILE_READ);
        
        if(!file){
            Serial.println("There was an error opening the file for reading");
            Serial.println("Maybe the file %s doesn't esist! Load default values and save them!");
            SaveDefaultsToSPIFFS();
            // Try again opening file for reading 
            file = SPIFFS.open(filename, FILE_READ);
            if(!file){
                Serial.println("There was an error opening the file for reading even after saving defaults!");
                return;
            }
        }
        
        // Allocate a temporary JsonDocument
        // Don't forget to change the capacity to match your requirements.
        // Use arduinojson.org/assistant to compute the capacity.
        int esize = this->size();
        Serial.printf("Entries size is %d bytes \r\n", esize);
        
        DynamicJsonDocument doc(esize) ;
        DeserializationError error = deserializeJson(doc, file);
        if (error){
            Serial.println(F("Failed to read file, using default configuration"));
            SaveDefaultsToSPIFFS();
            return;
        }    
        // Load Entries with value recovered from SPIFFS
        for(std::vector<Config_ns::Entry>::iterator it = Entries.begin(); it != Entries.end(); it++ ){
                //Serial.printf("Loading entry %s with value ",
                //    it->key.c_str());
                if(it->value_type == "Bool"){
                    it->SetBool(doc[it->key]);
                    //Serial.println(it->val_bool);
                }else if(it->value_type == "Int"){
                    it->SetInt(doc[it->key]);
                    //Serial.println(it->val_int);
                }else if(it->value_type == "Long"){
                    it->SetLong(doc[it->key]);
                    //Serial.println(it->val_long);
                }else if(it->value_type == "String"){
                    it->SetString(doc[it->key]);
                    //Serial.println(it->val_string.c_str());
                }
        }
    }

   //Save from Entries to filename in SPIFFS
   void ConfigClass::SaveEntriesToSPIFFS(){
        Serial.printf("Save EntriesToSPIFFS %s\r\n", filename.c_str());
        
        // Allocate a temporary JsonDocument
        // Don't forget to change the capacity to match your requirements.
        // Use arduinojson.org/assistant to compute the capacity.
        int esize = this->size();
        Serial.printf("Entries size is %d bytes \r\n", esize);
        
        DynamicJsonDocument doc(esize) ;

        for(std::vector<Config_ns::Entry>::iterator it = Entries.begin(); it != Entries.end(); it++ ){
                Serial.printf("Saving entry %s with value ",
                    it->key.c_str());
                if(it->value_type == "Bool"){
                    doc[it->key] = it->val_bool;
                    Serial.println(it->val_bool);
                }else if(it->value_type == "Int"){
                    doc[it->key] = it->val_int;
                    Serial.println(it->val_int);
                }else if(it->value_type == "Long"){
                    doc[it->key] = it->val_long;
                Serial.println(it->val_long);
                }else if(it->value_type == "String"){
                    doc[it->key] = it->val_string;
                Serial.println(it->val_string.c_str());
                }
        }

        // Open file for writing 
        File file = SPIFFS.open(filename, FILE_WRITE);
        
        if(!file){
            Serial.println("There was an error opening the file for writing");
            return;
        }

        // Serialize JSON to file
        if (serializeJson(doc, file) == 0) {
           Serial.println(F("Failed to write defaults to file"));
        }

        file.close();
    }

    //Save from Entries to filename in SPIFFS
    void ConfigClass::SaveDefaultsToSPIFFS(){
        Serial.printf("SaveDefaultsToSPIFFS %s\r\n", filename.c_str());
        
        // Allocate a temporary JsonDocument
        // Don't forget to change the capacity to match your requirements.
        int esize = this->size();
        Serial.printf("Entries size is %d bytes \r\n", esize);
        DynamicJsonDocument doc(esize);

        for(std::vector<Config_ns::Entry>::iterator it = Entries.begin(); it != Entries.end(); it++ ){
                //Serial.printf("Saving entry %s with default value ",
                //    it->key.c_str());
                if(it->value_type == "Bool"){
                    bool bb = (it->value_default == "true"); 
                    doc[it->key] = bb;
                    //Serial.println(bb);
                }else if(it->value_type == "Int"){
                    int ii = atoi(it->value_default.c_str()); 
                    doc[it->key]=ii;
                    //Serial.println(ii);
                }else if(it->value_type == "Long"){
                    long ll = atol(it->value_default.c_str());
                    doc[it->key]=ll;
                    //Serial.println(ll);
                }else if(it->value_type == "String"){
                    String ss = it->value_default;
                    doc[it->key] = ss;
                    //Serial.println(ss);
                }
        }
        
        SPIFFS.remove(filename); // remove the file, if any

        // Open file for writing 
        File file = SPIFFS.open(filename, FILE_WRITE);
        
        if(!file){
            Serial.println("There was an error opening the file for writing");
            return;
        }
        
        // Serialize JSON to file
        if (serializeJson(doc, file) == 0) {
           Serial.println(F("Failed to write defaults to file"));
        }
        // Close the file
        file.close();
    }

    // Prints the content of a file to the Serial
    void ConfigClass::printFile() {
        // Open file for reading 
        File file = SPIFFS.open(filename, FILE_READ);
        if(!file){
            Serial.println("There was an error opening the file for reading");
            return;
        }

    // Extract each characters by one by one
    while (file.available()) {
        Serial.print((char)file.read());
    }
    Serial.println();

    // Close the file
    file.close();
    }

    int ConfigClass::size(){
        int size = 0;
        for(std::vector<Config_ns::Entry>::iterator it = Entries.begin(); it != Entries.end(); it++ ){
            size += it->mysize;    
        }
        // round to the higher power of 2 i.e. 1024 or 2048 or
        double aa = powf(2, int(log2(size*1.5)+0.99999)); //1.5 is a safety margin
        return (int) aa;
    }

}
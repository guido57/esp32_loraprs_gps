#define LED_TOGGLE_PERIOD     1000

#define SERIAL_BAUD_RATE      115200

// change pinouts if not defined through native board LORA_* definitions
#ifndef LORA_RST
#pragma message("LoRa pin definitions are not found, redefining...")
#define LORA_RST              14
#define LORA_IRQ              26
#endif

#ifndef BUILTIN_LED
#pragma message("BUILDIN_LED is not found, defining as 2")
#define BUILTIN_LED           2
#endif

#define CFG_IS_CLIENT_MODE    false // set to false to enable beaconing. It was true
#define CFG_USE_DISPLAY       false

#define CFG_LORA_PIN_SS       5 //GG it was
#define CFG_LORA_PIN_RST      LORA_RST
#define CFG_LORA_PIN_DIO0     LORA_IRQ
#define CFG_LORA_USE_ISR      false // set to true for incoming packet ISR usage (stream mode, e.g. speech)

#define CFG_LORA_FREQ         433.775e6 
#define CFG_LORA_BW           125e3
#define CFG_LORA_SF           10
#define CFG_LORA_CR           5
#define CFG_LORA_PWR          20
#define CFG_LORA_ENABLE_CRC   true  // set to false for speech data

#define CFG_BT_NAME           "loraprs"
#define CFG_BT_USE_BLE        false // set to true to use bluetooth low energy (for ios devices)

#define CFG_APRS_LOGIN        "IZ5OQO-7"
#define CFG_APRS_PASS         "24248"
#define CFG_APRS_FILTER        "r/35.60/139.80/25"
//#define CFG_APRS_RAW_BKN      "IW5EJM-11>APZMDM,WIDE1-1:!4351.90N/01105.12E#LoRA 433.775MHz/BW125/SF12/CR7/0x34" //Prato
//#define CFG_APRS_RAW_BKN      "IZ5OQO-7>APZMDM,WIDE1-1:!4303.51NL01036.59E&LoRA iGate 433.775MHz/BW125/SF10/CR5/0x34" //Siena
//#define CFG_APRS_RAW_BKN      "IW5ALZ-7>APZMDM,WIDE1-1:!4303.51NL01036.59E&LoRA Tracker 433.775MHz/BW125/SF10/CR5" //Siena
#define CFG_APRS_RAW_BKN        "IW5ALZ-7>APZMDM,WIDE1-1:!4303.51N/01036.59E-LoRA Tracker 433.775MHz/BW125/SF10/CR5" //Siena

#define CFG_WIFI_SSID         "ap-stazione"
#define CFG_WIFI_KEY          "1111111111"

#define CFG_FREQ_CORR         false   // NB! incoming interrupts may stop working on frequent corrections when enabled
#define CFG_FREQ_CORR_DELTA   1000    //      test with your module before heavy usage

#define CFG_PERSISTENT_APRS   false
#define CFG_DIGIREPEAT        false
#define CFG_RF_TO_IS          false
#define CFG_IS_TO_RF          false
#define CFG_BEACON            true  //GG era false
#define CFG_KISS_EXTENSIONS   false  //GG era true

#define CFG_PTT_ENABLE        false
#define CFG_PTT_PIN           12
#define CFG_PTT_TX_DELAY_MS   50
#define CFG_PTT_TX_TAIL_MS    10

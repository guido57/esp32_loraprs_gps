#include <Arduino.h>

#include <arduino-timer.h>
#include "WiFi.h"
#include "loraprs_service.h"
#include "mygps.h"

#if __has_include("/tmp/esp32_loraprs_config.h")
#pragma message("Using external config")
#include "/tmp/esp32_loraprs_config.h"
#else
#pragma message("Using default config")
#include "config.h"
#endif

#if CFG_IS_CLIENT_MODE == true
#pragma message("Configured for client mode")
#else
#pragma message("Configured for server mode")
#endif


void initializeConfig(LoraPrs::Config &cfg) {
  
  // client/server mode switch
  cfg.IsClientMode = CFG_IS_CLIENT_MODE;
  cfg.UseDisplay = CFG_USE_DISPLAY;

  // lora parameters
  cfg.LoraFreq = CFG_LORA_FREQ;
  cfg.LoraBw = CFG_LORA_BW;
  cfg.LoraSf = CFG_LORA_SF;
  cfg.LoraCodingRate = CFG_LORA_CR;
  cfg.LoraSync = 0x34;
  cfg.LoraPower = CFG_LORA_PWR;
  cfg.LoraEnableCrc = CFG_LORA_ENABLE_CRC; // set to false for speech streaming data

  // lora pinouts
  cfg.LoraPinSs = CFG_LORA_PIN_SS;     // GPIO 18
  cfg.LoraPinRst = CFG_LORA_PIN_RST;   // GPIO 14 - TMS
  cfg.LoraPinDio0 = CFG_LORA_PIN_DIO0; // GPIO 26
  cfg.LoraUseIsr = CFG_LORA_USE_ISR;  // set to true for incoming packet ISR usage (stream mode, e.g. speech)

  // GPS pinout
  #define RXD2 15
  #define TXD2 12

  // aprs configuration
  cfg.AprsHost = "iz5oqo.duckdns.org";
  cfg.AprsPort = 14580;
  cfg.AprsLogin = CFG_APRS_LOGIN;
  cfg.AprsPass = CFG_APRS_PASS;
  cfg.AprsFilter = CFG_APRS_FILTER; // multiple filters are space separated
  cfg.AprsRawBeacon = CFG_APRS_RAW_BKN;
  cfg.AprsRawBeaconPeriodMinutes = 1; //GG it was 20

  // bluetooth device name
  cfg.BtName = CFG_BT_NAME;
  cfg.BtEnableBle = CFG_BT_USE_BLE;

  // server mode wifi paramaters
  cfg.WifiSsid = CFG_WIFI_SSID;
  cfg.WifiKey = CFG_WIFI_KEY;

  // frequency correction
  cfg.EnableAutoFreqCorrection = CFG_FREQ_CORR;  // automatic tune to any incoming packet frequency
  cfg.AutoFreqCorrectionDeltaHz = CFG_FREQ_CORR_DELTA;

  // configuration flags and features
  cfg.EnableSignalReport = true;  // signal report will be added to the comment sent to aprsis
  cfg.EnablePersistentAprsConnection = CFG_PERSISTENT_APRS; // keep aprsis connection open, otherwise connect on new data only
  cfg.EnableRfToIs = CFG_RF_TO_IS;  // send data from rf to aprsis
  cfg.EnableIsToRf = CFG_IS_TO_RF; // send data from aprsis to rf
  cfg.EnableRepeater = CFG_DIGIREPEAT; // digirepeat incoming packets
  cfg.EnableBeacon = CFG_BEACON;  // enable periodic AprsRawBeacon beacon to rf and aprsis if rf to aprsis is enabled
  cfg.EnableKissExtensions = CFG_KISS_EXTENSIONS; // radio control and signal reports

  // external ptt control
  cfg.PttEnable = CFG_PTT_ENABLE;
  cfg.PttPin = CFG_PTT_PIN;
  cfg.PttTxDelayMs = CFG_PTT_TX_DELAY_MS;
  cfg.PttTxTailMs = CFG_PTT_TX_TAIL_MS;

  // GPS 
  cfg.UseGPS = CFG_USE_GPS;     // if true, wait for GPS before sending beacon to LORA

}

LoraPrs::Service loraPrsService;

auto watchdogLedTimer = timer_create_default();

bool toggleWatchdogLed(void *) {
  digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
  return true;
}


void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 1);

  // Serial2 is used by GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial);
  
  LoraPrs::Config config;

  initializeConfig(config);
  
  
  loraPrsService.setup(config);

  //marco mod for sleep test
  //esp_sleep_enable_timer_wakeup(10 * 1000 * 1000);
  //esp_sleep_enable_ext0_wakeup((gpio_num_t)CFG_LORA_PIN_DIO0, 1);
  watchdogLedTimer.every(LED_TOGGLE_PERIOD, toggleWatchdogLed);

}

void loop() {
  loraPrsService.loop();
  //watchdogLedTimer.tick();
}


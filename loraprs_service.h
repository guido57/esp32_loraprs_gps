#ifndef LORAPRS_SEVICE_H
#define LORAPRS_SERVICE_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <cppQueue.h>

#include "BluetoothSerial.h"
#include "ax25_payload.h"
#include "kiss_processor.h"
#include "loraprs_config.h"

namespace LoraPrs {

class Service : public Kiss::Processor
{
public:
  Service();
  
  void setup(const Config &conf);
  void loop();

private:
  void setupWifi(const String &wifiName, const String &wifiKey);
  void setupLora(long loraFreq, long bw, int sf, int cr, int pwr, int sync, bool enableCrc);
  void setupBt(const String &btName);

  void reconnectWifi() const;
  bool reconnectAprsis();
  
  void onLoraDataAvailable(int packetSize);
  void onAprsisDataAvailable();

  void sendPeriodicBeacon();
  void sendToAprsis(const String &aprsMessage);
  bool sendAX25ToLora(const AX25::Payload &payload);
  void processIncomingRawPacketAsServer(const byte *packet, int packetLength);
  
  inline bool needsAprsis() const { 
    return !config_.IsClientMode && (config_.EnableRfToIs || config_.EnableIsToRf); 
  }
  inline bool needsWifi() const { return needsAprsis(); }
  inline bool needsBt() const { return config_.IsClientMode; }
  inline bool needsBeacon() const { return !config_.IsClientMode && config_.EnableBeacon; }

protected:
  virtual bool onRigTxBegin();
  virtual void onRigTx(byte b);
  virtual void onRigTxEnd();

  virtual void onSerialTx(byte b);
  virtual bool onSerialRxHasData();
  virtual bool onSerialRx(byte *b);

  virtual void onControlCommand(Cmd cmd, byte value);
  
private:
  const String CfgLoraprsVersion = "LoRAPRS 0.1";

  // module pinouts
  const byte CfgPinSs = 5;
  const byte CfgPinRst = 26;
  const byte CfgPinDio0 = 14;

  // processor config
  const int CfgConnRetryMs = 500;
  const int CfgPollDelayMs = 5;
  const int CfgWiFiConnRetryMaxTimes = 10;
  const int CfgMaxAX25PayloadSize = 512;

  // csma paramters, use lower value for high traffic, use 255 for real time
  const long CfgCsmaPersistence = 100;
  const long CfgCsmaSlotTimeMs = 500;
  
private:
  // config
  Config config_;
  String aprsLoginCommand_;
  AX25::Callsign ownCallsign_;

  // csma
  byte csmaP_;
  long csmaSlotTime_;
  long csmaSlotTimePrev_;

  // state
  long previousBeaconMs_;
    
  // peripherals
  BluetoothSerial serialBt_;
  WiFiClient aprsisConn_;
};

} // LoraPrs

#endif // LORAPRS_SERVICE_H

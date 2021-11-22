#include <captive_portal.h>
#include <config.h>
#include <handleHttp.h>

// the ConfigClass 
extern Config_ns::ConfigClass cc; 

//=== CaptivePortal stuff ================
String softAP_ssid;
String softAP_password;

/* hostname for mDNS. Should work at least on windows. Try http://esp32.local */
const char *myHostname = "esp32";

// NTP settings
const char* ntpServer = "europe.pool.ntp.org";
const long  gmtOffset_sec = 3600; // GMT + 1
//Change the Daylight offset in milliseconds. If your country observes Daylight saving time set it to 3600. Otherwise, set it to 0.
const int   daylightOffset_sec = 3600;

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

// Web server
WebServer web_server(80);

/* Setup the Access Point */
void AccessPointSetup();

/* Soft AP network parameters */
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
/** Should I connect to WLAN asap? */
boolean connect;

/** Last time I tried to connect to WLAN */
unsigned long lastConnectTry = 0;

/** Last time I tried to ping an external IP address (usually the gateway) */
unsigned long lastPing = 0;

/** Current WLAN status */
unsigned int status = WL_IDLE_STATUS;

// =====================================================
void connectWifi() {
    Serial.println("Connecting as wifi client...");
    //WiFi.forceSleepWake();
    //WiFi.disconnect();
    /*
    Serial.print("ssid=");Serial.println( espNow->settings.entries.ssid);
    Serial.print("password=");Serial.println(espNow->settings.entries.pwd);
    WiFi.begin(espNow->settings.entries.ssid, espNow->settings.entries.pwd);
    int connRes = WiFi.waitForConnectResult();
    Serial.print("connRes: ");
    Serial.println(connRes);
    */
}


void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

  tm timeinfo;
  bool got_local_time = false;

// =====================================================
// Callback for the TaskWIFI  
/**
   check WIFI conditions and try to connect to WIFI.
 * @return void
 */
void WiFi_loop(void){

  // Check if connected or not  
  unsigned int s = WiFi.status();

  // if WLAN status changed
  if (status != s)
  {
    Serial.printf("Status changed from %d to %d:\r\n",status,s);
    status = s;
    if (s == WL_CONNECTED){
      /* Just connected to WLAN */
      //Serial.printf("\r\nConnected to %s\r\n",espNow->settings.entries.ssid);
      Serial.print("IP address");
      Serial.println(WiFi.localIP());

      //init and get the time
      //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      #define TZ "CET-1CEST,M3.5.0/2,M10.5.0/3"
      configTzTime( TZ, ntpServer); //sets TZ and starts NTP sync
      printLocalTime();

      if(!getLocalTime(&timeinfo, 5000U)){ // 500msecs timeout
        Serial.println("Failed to obtain time");
        got_local_time = false;
     
      }else
        got_local_time = true;

      // Setup MDNS responder
      /*
          if (!MDNS.begin(myHostname)) {
            Serial.println("Error setting up MDNS responder!");
          } else {
            Serial.println("mDNS responder started");
            // Add service to MDNS-SD
            MDNS.addService("http", "tcp", 80);
          }
          */
      Serial.println("just connected");

    }
    /*
    else if (s == WL_NO_SSID_AVAIL){
      Serial.println("no SSID available -> turn on the Access Point");
      WiFi.disconnect();
      WiFi.mode(WIFI_MODE_APSTA);
    }
    else{
      Serial.println("not connected -> turn on the Access Point");
      WiFi.mode(WIFI_MODE_APSTA);

    }
    */
  }             
  // Check the Access Point button to turn on the Access Point
  if (digitalRead(cc["ApPin"]->val_int) == LOW){
    if(WiFi.getMode() == WIFI_MODE_STA){
      Serial.println("Turn on Access Point");
      WiFi.mode(WIFI_MODE_APSTA);
    }else if(WiFi.getMode()== WIFI_MODE_NULL) {
      Serial.println("Turn on Access Point");
      WiFi.mode(WIFI_MODE_AP);
    }
  }
 
  if (s == WL_CONNECTED){
    //MDNS.update();

  }
  // Do work:
  //DNS
  dnsServer.processNextRequest();
  //HTTP
  web_server.handleClient();

}

// Set the Access Point but let the WiFi off
void AccessPointSetup(){

  //softAP_ssid =  "ESP32_" + WiFi.macAddress();
  softAP_ssid     = cc["ApSsid"]->val_string; 
  softAP_password = cc["ApPwd"]->val_string;
  pinMode( cc["ApPin"]->val_int, INPUT_PULLUP);// set the pin as input
  delay(1000);
  // Access Point Setup
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(softAP_ssid.c_str(), softAP_password.c_str());
  
  WiFi.softAPConfig(apIP, apIP, netMsk);
  delay(100); // Without delay I've seen the IP address blank
  Serial.println("Access Point set:");
  Serial.println("Access Point already set:");
  
  Serial.printf("    SSID: %s\r\n", softAP_ssid.c_str());
  Serial.print("    IP address: ");
  Serial.println(WiFi.softAPIP());
  WiFi.mode(WIFI_MODE_NULL); // at the beginning turn off the WiFi
}

void CaptivePortalSetup(){

  AccessPointSetup();

  /* Setup the DNS web_server redirecting all the domains to the apIP */
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", apIP);

  /* Setup web pages: root, wifi config pages, SO captive portal detectors and not found. */
  web_server.on("/", handlehttp::handleRoot);
  web_server.on("/settingssave", handlehttp::handleSettingsSave);
  web_server.on("/restoresettings", handlehttp::handleRestoreSettings);
  //web_server.on("/wifi", handlehttp::handleWifi);
  //web_server.on("/wifisave", handlehttp::handleWifiSave);
  web_server.on("/generate_204", handlehttp::handleRoot);  //Android captive portal. Maybe not needed. Might be handled by notFound handler.
  web_server.on("/fwlink", handlehttp::handleRoot);  //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
  web_server.onNotFound(handlehttp::handleNotFound);
  web_server.begin(); // Web server start
  Serial.println("HTTP server started");
  //loadCredentials(); // Load WLAN credentials from network
  //connect = strlen(espNow->settings.entries.ssid) > 0; // Request WLAN connect if there is a SSID
}
 
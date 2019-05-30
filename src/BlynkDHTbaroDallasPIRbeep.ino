// BlynkDHTbaroDallasPIRbeepHttpOTA.ino
// 5-15-2019
// by: Truglodite
//
// Connects to Blynk server and uploads temp, humidity, and barometer data.
// Beeps and notify's if armed during PIR motion.

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <BlynkSimpleEsp8266_SSL.h>
#include <Wire.h>
#include <MS5611.h>
#include <MS5xxx.h> //PIO: just MS5xxx library, no MS5611 library
#include <DHT.h>  //PIO: Adafruit unified DHT and Adafruit unified sensor libraries
#include <OneWire.h>
#include <DallasTemperature.h>
extern "C" {
  #include "user_interface.h"
}
#define privacy
#include "configuration.h"
#ifdef privacy
  #include "privacy.h"
#endif

//#define debug                                   // Uncomment to enable serial debug prints
//*******************  Globals  *********************//
bool firmwareUp = 0;                 // Holder for firmware button position
bool isFirmwareUpSet = 0;            // Holder for sync finished/unfinished status
bool isArmButtonSet = 0;             // Holder for sync finished/unfinished status
bool isNtriggersSet = 0;        // Holder for sync finished/unfinished status
bool OTAnotificationSent = 0;        // Holder for OTA ready notification
bool alarmStatus = 0;                // Holder for alarm status
bool beeperActive = 0;               // Beeper status flag
unsigned long dhtReadTime = 0;       // Holder for DHT timeout timer
unsigned long dallasReadTime = 0;    // Holder for time of last dallas reading
unsigned long baroReadTime = 0;      // Holder for DHT timeout timer
unsigned long lastUploadTime = 0;    // Holder for last sample millis()
unsigned long lastConnectionCheck = 0;// Holder for time of last Blynk connection check
unsigned long otaStartTime = 0;      // Holder for OTA timeout timer
unsigned long lastPirHigh = 0;       // Holder for PIR timeout timer
unsigned long beepEventTime = 0;     // Holder for beep timer
unsigned long lastNotifyTime = 0;    // Holder for notification timer

float humidA = 0.0;                  // Holder for DHT humidity NAN check
float humid = 0.0;                   // Holder for DHT humidity
float heatIndex = 0.0;               // Holder for DHT heat index
float dallasTemp = 0.0;              // Holder for Dallas temp readings
float pressure = 0.0;                // Holder for barometer pressure
float dewPoint = 0.0;                // Holder for dew point
float dhtSamples = 0.0;              // Holder for number of DHT readings
float baroSamples = 0.0;             // Holder for number of baro readings
float dallasSamples=0.0;             // Holder for number of Dallas readings
int retriesDHT = 0;                  // Holder for DHT read retry count
int armButton = 0;                   // Holder for arm button state
int nTriggers = 0;                   // Holder for # of triggers (also on app)
int nBeeps = 0;                      // Holder for number of beeps
DeviceAddress dallasAddress;         // Holder for Dallas device address
IPAddress myIP;

MS5611 baro(&Wire);                  // Baro init
DHT dht(dhtPin,dhtType);             // DHT init
OneWire oneWire(dallasPin);          // OneWire init
DallasTemperature dallas(&oneWire);  // Dallas sensors init

ESP8266WebServer httpServer(80);  //init web update server
ESP8266HTTPUpdateServer httpUpdater;

char notifyOTAready[sizeof(notifyOTAreadyX) + 27 + sizeof(update_path) + 1] = {0};

//**************************  Setup  *************************//
////////////////////////////////////////////////////////////////
void setup() {
  pinMode(pirPin,INPUT);             // Setup PIR input pin
  pinMode(beepPin,OUTPUT);           // Setup beeper output pin
  digitalWrite(beepPin,LOW);         // Turn beeper off

  #ifdef debug
    Serial.begin(115200);
    Serial.println("Debug Enabled...");
  #endif

  WiFi.mode(WIFI_STA);               // Wifi mode config
  wifi_station_set_hostname(hostName);
  Blynk.begin(auth,ssid,pass);       // Blynk server setup
  checkConnection();                 // Connect to Wifi and Blynk

  // combine this string now that we have an IP
  sprintf(notifyOTAready, "OTA Waiting\nhttp://%s%s", myIP.toString().c_str(), update_path);

  //Initialize OTA server
  #ifdef debug
    Serial.print("Setting up OTA: ");
  #endif
  MDNS.begin(hostName);
  httpUpdater.setup(&httpServer, update_path, update_username, otaPassword);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  #ifdef debug
    Serial.println("Ready");
  #endif

  #ifdef debug                       // I2C setup
    Serial.println("Intializing i2c...");
  #endif
  Wire.begin(sdaPin,sclPin);

  #ifdef debug                       // Baro setup
    Serial.println("Intializing Baro...");
  #endif
  if(baro.connect()>0) {
    #ifdef debug
      Serial.println("No Baro Found!!!");
    #endif
    Blynk.notify(notifyBarofail);
  }

  #ifdef debug                       // DHT setup
    Serial.println("Intializing DHT Sensor...");
  #endif
  dht.begin();

  #ifdef debug                       // Dallas setup
    Serial.println("Intializing Dallas Sensor(s)...");
  #endif
  dallas.begin();
  #ifdef debug
    if (dallas.isParasitePowerMode()) Serial.println("Parasite Power: ON");
    else Serial.println("Parasite Power: OFF");
  #endif
  if (!dallas.getAddress(dallasAddress, 0)) Serial.println("Unable to find address for Device 0");
  dallas.setResolution(dallasAddress, dallasResolution);// set new resolution to tempResolution
  #ifdef debug                      // Report new resolution for each device
    Serial.print("Device 0 New resolution:");
    Serial.print(dallas.getResolution(dallasAddress), DEC);
    Serial.println(" bits");
  #endif
  dallas.setWaitForConversion(false);// Non-blocking, we'll handle timing manually
}
//********************  Lib Functions?  *********************//
///////////////////////////////////////////////////////////////
BLYNK_CONNECTED() {                  // Sync when Blynk is connected
  Blynk.syncAll();
}

BLYNK_WRITE(firmwareVpin) {          // Sync firmware button from app, and set the flag
  firmwareUp = param.asInt();
  isFirmwareUpSet = 1;
}
BLYNK_WRITE(armButtonVpin) {         // Read arm/disarm value from app Vpin
  armButton = param.asInt();
  #ifdef debug
    Serial.print(armButtonVpin);
    Serial.print(" button reads: ");
    Serial.println(armButton);
  #endif
  isArmButtonSet = 1;
}

BLYNK_WRITE(triggersVpin) {          // Read # of triggers from app Vpin
  nTriggers = param.asInt();
  #ifdef debug
    Serial.print(triggersVpin);
    Serial.print(" nTriggers reads: ");
    Serial.println(nTriggers);
  #endif
  isNtriggersSet = 1;
}
///////////////////////////////////////////////////////////////
//************************  LOOP  ***************************//
void loop() {
  Blynk.run();                       // Keeps V-pins up to date
  beeper();                          // Process horn beeps

  // Periodically check connection......................................
  if (millis() - lastConnectionCheck > checkConnPeriod) checkConnection();

  // FW button OFF: normal routine......................................
  if(isFirmwareUpSet && isArmButtonSet && isNtriggersSet && !firmwareUp){
    OTAnotificationSent = 0;         // reset OTA notification flag
    #ifdef debug
      Serial.print("Reading PIR: ");
    #endif
    bool pirStatus = digitalRead(pirPin); // Read PIR
    #ifdef debug
      Serial.println(pirStatus);
    #endif

    // PIR triggered, alarm is off
    if(pirStatus && !alarmStatus) {
      lastPirHigh = millis();
      #ifdef debug
        Serial.println("PIR triggered");
      #endif
      if(armButton) {                // Send alert if armed
        #ifdef debug
          Serial.println("Armed, sending alert.");
        #endif
        if(millis() - lastNotifyTime > minNotifyPeriod) {  // anti-spam
          lastNotifyTime = millis();
          Blynk.notify(alarmMessage);
        }
        nBeeps = 2;                  // Beep the horn twice
      }
      nTriggers++;
      if(nTriggers > triggersMax) nTriggers = 0; //Reset triggers if it's too big
      #ifdef debug
        Serial.println("Sending LED on and trigger++.");
      #endif
      Blynk.virtualWrite(alarmVpin, 255);// Turn on alarm LED
      Blynk.virtualWrite(triggersVpin, nTriggers);// Update trigger count
      alarmStatus = 1;               // Set alarm on flag
    }

    // PIR on, alarm status on
    if(pirStatus && alarmStatus) {
      lastPirHigh = millis();        // Update PIR timer
    }

    // PIR off, & alarm is started
    if(!pirStatus && alarmStatus) {
      if(millis() - lastPirHigh > 1000*pirTimeout) { // Check for PIR timeout
        #ifdef debug
          Serial.println();
          Serial.println("PIR timed out, resetting alarm.");
        #endif
        alarmStatus = 0;             // Set alarm off flag
        Blynk.virtualWrite(alarmVpin, 0);// Turn off alarm LED
      }
    }

    // Timed non-blocking read functions
    if(millis() - baroReadTime > baroPeriod)     readMS5611();
    if(millis() - dhtReadTime > dhtPeriod)       readDHT();
    if(millis() - dallasReadTime > dallasPeriod) readDallas();

    // Time to process, upload, and re-init...
    if(millis() - lastUploadTime > uploadPeriod) {
      #ifdef debug
        Serial.println("Calculating Averages...");
      #endif
      humid = humid / dhtSamples;
      pressure = pressure / baroSamples;
      dallasTemp = dallasTemp / dallasSamples;
                                     // Calculate derived values
      heatIndex = dht.computeHeatIndex(dallasTemp, humid,false);// Heat Index... false for C
      dewPoint = dewPointC(dallasTemp, humid);// Dew Point C
      //pressure = pressure / 1000.0;// Pa to kPa

      #ifdef unitsFarenheit          // Convert C's to F's if needed
        dallasTemp = celsiusToFarenheit(dallasTemp);
        heatIndex = celsiusToFarenheit(heatIndex);
        dewPoint = celsiusToFarenheit(dewPoint);
      #endif

      #ifdef debug
        Serial.print("R.H.= ");
        Serial.print(humid);
        Serial.println(" %");
        Serial.print("Dallas= ");
        Serial.print(dallasTemp);
        Serial.println(" F");
        Serial.print("Heat Index= ");
        Serial.print(heatIndex);
        Serial.println(" F");
        Serial.print("Dew Point= ");
        Serial.print(dewPoint);
        Serial.println(" F");
        Serial.print("Pressure= ");
        Serial.print(pressure);
        Serial.println(" Pa");
      #endif

      uploadData();                  // Upload data to blynk

      Serial.println("Resetting averaging buffers...");
      dhtSamples = 0.0;              // Reset the averaging buffers
      dallasSamples = 0.0;
      baroSamples = 0.0;
      humid = 0.0;
      dallasTemp = 0.0;
      pressure = 0.0;
    }
  }

  // FW button ON & Notification not sent: send OTA ready notification...
  else if(isFirmwareUpSet && firmwareUp && !OTAnotificationSent){
    #ifdef debug
      Serial.println("Firmware OTA ready...");
    #endif
    Blynk.notify(notifyOTAready);
    OTAnotificationSent = 1;         // Flag so we send just one
    otaStartTime = millis();
  }

  // FW button ON: handle ArduinoOTA calls...............................
  else if(isFirmwareUpSet && firmwareUp) {
    httpServer.handleClient();
    MDNS.update();
    if(millis() - otaStartTime > otaTimeout){// OTA timeout... reset so we send another notification
      #ifdef debug
        Serial.println("OTA timeout... restarting.");
      #endif
      ESP.restart();                 // Restart if failed to connect to baro
      delay(500);
    }
  }
}
///////////////////////////////////////////////////////////////
//*********************  Functions  *************************//
// Read Baro //////////////////////////////////////////////////
void readMS5611() {
  #ifdef debug
    Serial.println("Reading MS5611... ");
  #endif
  // Barometer read and print
  baro.ReadProm();
  baro.Readout();                   // Update MS5611 sensor
  pressure += baro.GetPres();       // Get pressure in Pa
  #ifdef debug
    Serial.print("Pressure Sum: ");
    Serial.print(pressure);
    Serial.println(" Pa");
  #endif
  baroReadTime = millis();
  baroSamples++;
  #ifdef debug
    Serial.print("Baro Samples/ReadTime: ");
    Serial.print(baroSamples);
    Serial.print(" samples / ");
    Serial.print(baroReadTime);
    Serial.println(" msec");
  #endif
}
// Read Humidity /////////////////////////////////////////
void readDHT()  {
  #ifdef debug
    Serial.println("Reading DHT Sensor...");
  #endif
  humidA = dht.readHumidity();        // RH %
  dhtReadTime = millis();
  if(isnan(humidA)) {// Drop the reading(s) if we get any NAN's
    retriesDHT ++;
    #ifdef debug
      Serial.print("DHT Read Fail... fails in a row: ");
      Serial.println(retriesDHT);
    #endif
    if(retriesDHT > dhtRetriesMax)  { // Too many NAN's in a row, reboot... (very rare)
      #ifdef debug
        Serial.println("Too many NANs, sending notification and restarting...");
      #endif
      Blynk.notify(notifyDHTfail);
      ESP.restart();
      delay(500);
    }
    return;
  }
  else {                              // Cool, no NAN's, count it...
    retriesDHT = 0;
    humid += humidA;
    dhtSamples++;
    #ifdef debug
      Serial.print("DHT ReadTime: ");
      Serial.print(dhtReadTime);
      Serial.println(" msec");
      Serial.print("DHT Samples: ");
      Serial.println(dhtSamples);
      Serial.print("R.H. Sum: ");
      Serial.print(humid);
      Serial.println(" %");
    #endif
  }
}
// Non-blocking process for updating Dallas temp readings////////////////
void readDallas()  {
  #ifdef debug
    Serial.println("Reading Dallas temp...");
  #endif
  dallasTemp += dallas.getTempCByIndex(0);// Read temp conversion
  dallas.requestTemperatures();      // Send command to start next reading & conversion
  dallasReadTime = millis();
  dallasSamples++;
  #ifdef debug
    Serial.print("Dallas ReadTime: ");
    Serial.print(dallasReadTime);
    Serial.println(" msec");
    Serial.print("Dallas Samples: ");
    Serial.println(dallasSamples);
    Serial.print("Dallas Sum: ");
    Serial.print(dallasTemp);
    Serial.println(" C");
  #endif
}
// Upload data to server ///////////////////////////////////////////
void uploadData() {
  #ifdef debug
    Serial.print("Sending data to Blynk Server");
  #endif

  Blynk.virtualWrite(humidVpin, humid);
  #ifdef debug
    Serial.print(".");
  #endif
  Blynk.virtualWrite(heatIndexVpin, heatIndex);
  #ifdef debug
    Serial.print(".");
  #endif
  Blynk.virtualWrite(baroVpin, pressure);
  #ifdef debug
    Serial.print(".");
  #endif
  Blynk.virtualWrite(dewPointVpin, dewPoint);
  #ifdef debug
    Serial.print(".");
  #endif
  Blynk.virtualWrite(temp1Vpin, dallasTemp);
  yield(); //make sure values get sent before buffers are zeroed
  lastUploadTime = millis();
  #ifdef debug
    Serial.println("done.");
  #endif
}
// Check connection to Blynk Server ////////////////////////////////
void checkConnection() {
  lastConnectionCheck = millis();     // Done checking, reset check timer
  if(!Blynk.connected()) {            // We aren't connected
    #ifdef debug
      Serial.print("Checking via: ");
      Serial.println(ssid);
    #endif
    while (!Blynk.connect()) {       // Loop here until connected (pointless to continue)
      if(millis() - lastConnectionCheck > restartPeriod) {// Reboot if not connected before timeout
        #ifdef debug
          Serial.println("Failed to connect... restarting.");
        #endif
        ESP.restart();
        delay(500);
      }
    }
    //we only get past here if we are connected
    myIP = WiFi.localIP();
    #ifdef debug
      Serial.print("Connected through: ");
      //Serial.println(WiFi.localIP());
      Serial.println(myIP);
    #endif
  }
  myIP = WiFi.localIP();
}
// Takes Celsius & RH%, and returns dewpoint in C/////////////////////
float dewPointC(float celsius, float rhPercent)  {
  float a = 17.271;
  float b = 237.7;
  float c = (a * celsius) / (b + celsius) + log(rhPercent*0.01);
  float TdC = (b * c) / (a - c);
  return TdC;
}
// Obvious by name...////////////////////////////////////////////////
float celsiusToFarenheit(float celsius)  {
  float farenheit = 1.8*celsius + 32.0;
  return farenheit;
}
// Non-blocking Beeper/////////////////////////////////////////////////////
void beeper(void) {
  if(nBeeps)  { //we have sme beeping left to do
    unsigned long msecs = millis(); //grab time once per loop should be good enough
    if(msecs - beepEventTime > beepDelay) { //time to do something with the horn
      if(!beeperActive)  { //horn is off
        beepEventTime = msecs; //reset timer
        digitalWrite(beepPin,HIGH); //sound the horn
        beeperActive = 1;
      }
      else if(beeperActive) { //horn is on
        beepEventTime = msecs; //reset timer
        digitalWrite(beepPin,LOW); //kill the horn
        beeperActive = 0;
        nBeeps--; //done with one beep
      }
    }
  }
}

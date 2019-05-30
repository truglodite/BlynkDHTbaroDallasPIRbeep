//////////////////////////////////////////////////////////
// configuration.h
// by: Truglodite
// updated: 5/28/2019
//
// General configuration for BlynkDHTbaroDallasPIRbeep.
//
//////////////////////////////////////////////////////////
#pragma once

// Github users can add a /src/privacy.h with #define privacy, etc.
#ifndef privacy
char auth[] = "MyBlynkAuthToken";                 // Blynk App auth token
char ssid[] = "MyWifiSSID";                       // Wifi SSID
char pass[]=  "MyWifiPassword";                   // Wifi WPA2 password
//IPAddress staticIP          (192,168,0,3);      // Static local IP (setup your router accordingly)
//byte mac[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0xED};   // Wifi MAC
char hostName[] =     "ESP-myNode";               // OTA and wifihostname, default "esp8266-[ChipID]"
const char* update_path =   "/firmware";          // OTA webserver update directory
const char* update_username = "admin";            // OTA username
const char otaPassword[] =  "password";            // OTA password
char notifyOTAreadyX[] =     "OTA Waiting\nhttp://";// OTA ready notification template (holds characters)
char notifyDHTfail[] =      "myNode: DHT read error";// Failed DHT read notification text
char notifyBarofail[] =     "myNode: Baro init failed";// Failed baro init notification text
char alarmMessage[] =       "myNode: Motion!";// Message to e-mail/notify when alarm is tripped
#endif

#define unitsFarenheit                            // Uncomment to enable Farenheit output
#define firmwareVpin        V0                    // Firmware Upload Button, 0= Normal, 1= Firmware OTA
#define armButtonVpin       V1                    // Arm/Disarm button
#define alarmVpin           V2                    // Alarm status LED
#define triggersVpin        V3                    // Pin to indicate # of triggers
#define temp1Vpin           V4                    // Dallas temperature [F]
#define humidVpin           V5                    // Relative Humidity [%]
#define baroVpin            V6                    // Barometeric Pressure [Pa]
#define heatIndexVpin       V7                    // Heat Index [F]
#define dewPointVpin        V8                    // Dew Point [F]
#define pirPin              13                    // Physical pin: PIR output
#define dhtPin              12                    // DHT sensor data pin (default io12)
#define dallasPin           14                    // Dallas sensor data pin
#define sclPin              4                     // i2c clock pin
#define sdaPin              5                     // i2c data pin
#define dallasResolution    12                    // temperature resolution bits (9 default, up to 12)
#define beepPin             16                    // Beeper signal
#define dhtType             DHT22                 // DHT22, DHT11, etc...
#define dhtRetriesMax       5                     // Number of DHT "NAN" reads before rebooting
#define dallasPeriod        800                   // millis to wait between start of conversion and reading (>760 for 12bit)
#define dhtPeriod           2000                  // milliseconds between DHT sensor reads
#define baroPeriod          500                   // milliseconds between baro reads
#define pirTimeout          5                     // Seconds between last PIR high and trigger reset
#define triggersMax         50                    // Reset trigger count to 0 when > this value
#define checkConnPeriod     11000                 // milliseconds between active connection checks
#define uploadPeriod        30000                 // milliseconds between uploads, data is averaged between uploads (don't flood Blynk... "10 values per sec")
#define minNotifyPeriod     300000                // milliseconds notificaitons will be delayed if they repeat faster than this
#define restartPeriod       60000                 // milliseconds between reboots when connection fails
#define otaTimeout          300000                // milliseconds to wait for OTA upload before reboot (ie. period between OTA notifications)
#define beepDelay           100                   // milliseconds length of and delay between beeps

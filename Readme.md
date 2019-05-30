# BlynkDHTbaroDallasPIRbeep

This code is written for use with an ESP8266, an MS5611 (barometer), an DHTXX (humidity), a Dallas DS18B01 digital sensor (temp), a digital output PIR sensor (motion), and a 3v3 level beeper. It connects to Blynk server and periodically uploads temp, humidity, and barometer readings. It continuously observes PIR state, and beeps+sends a notification during PIR motion if armed with a Blynk App button. Browser firmware OTA upgrade is also initiated by a Blynk app button. This code is written to be as non-blocking as possible, however no modifications were done to the included libraries (which may contain some lines of blocking code).

Added bonus: It's also an internet connection monitor! Since this code resets when failing to connect to Blynk, it emits a short/periodic beep while internet is down.

## Blynk Virtual Pins
Vpin | Data Output (F.W. and Arm buttons are required... BLYNK_WRITE()'s use them)
--- | ---------------------
V0 | Firmware Upload Button [0 = normal, 1 = upload]
V1 | Alarm Arm Button
V2 | Alarm Status LED
V3 | Alarm Trigger Count
V4 | Temperature 1 - Dallas [F]
V5 | Relative Humidity [%]
V6 | Barometer [Pa]
V7 | Heat Index [F]
V8 | Dew Point [F]

## ESP8266 Pinout
ESPpin | Description
------ | -------------------
io4 | Baro SCL pins
io5 | Baro SDA pins
io13 | PIR output pin
io12 | DHT sensor data pin
io14 | Dallas temp sensor (non-parasitic power)
io0 | 4k7 Low
io2 | 4k7 High
io15 | 4k7 Low
EN | 4k7 High
RST | 4k7 High

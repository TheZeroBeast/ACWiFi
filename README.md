# ACWiFi

This firmware was written using the Arduino IDE and allows an ESP8266 module (Wemos D1 R1 mini in this instance) to communicate with a Mitsubishi Heavy Industries
split system air conditioner via the CNS connector on the indoor unit control PCB. 

This allows control and monitoring of the attached air conditioner via MQTT and Domoticz or Home Assistant.

The "master" branch is suitable for use with Home Assistant and the "domoticz" branch is suitable for use with Domoticz.

An open-source PCB design that supports this firmware can be found here https://github.com/dvisser/MHI-ESP-PCB

Credit to GitHub user rjdekker for their work documenting the MHI SPI protocol, their work can be found here https://github.com/rjdekker/MHI2MQTT

NOTE: A fix for Arduino OTA for Windows 10 based systems is included in the batch file within this repository.

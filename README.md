###Arduino-based PID controller for Rancilio Silvia boiler temperature.

Maintain stable boiler temperature by reading the temp from a MAX6675
thermocouple amplifier and switching a solid state relay on/off.

LCD display and buttons to display actual temp, update the set point temp,
and change the P I D values to tune the PID controller.

Right now, we just print out what we would do to the serial port,
although it's ready to control a real LCD and relay.

For testing, this includes a "fake boiler" that heats up and
cools down at a certain rate.

<pre>
TODO:
- control real LCD instead of FakeLCD
- actually change PIN_RELAY_CONTROL in update_relay()
- initialize settings from eeprom
- update changed settings to eeprom
- replace update_faketemp() with data from real thermocouple
- remove debugging Serial.print() statements
</pre>

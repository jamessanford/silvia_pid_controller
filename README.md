###Arduino-based PID controller for Rancilio Silvia boiler temperature.

Maintain stable boiler temperature by reading the temp from a MAX6675
thermocouple amplifier and switching a solid state relay on/off.

LCD display and buttons to display actual temp, update the set point temp,
and change the P I D values to tune the PID controller.

For testing, this includes a "fake boiler" that heats up and
cools down at a certain rate.  Just set FAKE_BOILER to 1.

<pre>
TODO:
- initialize settings from eeprom
- update changed settings to eeprom
- remove debugging Serial.print() statements
</pre>

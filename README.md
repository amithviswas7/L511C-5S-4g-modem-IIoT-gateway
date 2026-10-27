# L511C-5S-4g-modem-IIoT-gateway
4g modem used Modbus-RTU repo library, acts as a master which forwards or generates the Modbus RTU requests and forwards it to slave and validate slave response, transfers to the client using mqtt.
FC1_8.c and modbus_mqtt_raw_binary.py are one set of program, .py will acts as modbus request frame builder and publish it using mqtt and prints received response from the modem (FC1_8.c)

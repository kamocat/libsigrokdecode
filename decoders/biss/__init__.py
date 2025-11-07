'''
BiSS (Bidirectional/Serial/Synchronous) is a digital, serial interface protocol for fast and safe isochronous process data transmission, particularly used in motor feedback systems. Simultaneous to the reception of sensor process data and the transmission of actuator process data in real-time, the BiSS protocol is capable of register data transfers without interrupting the process data stream.


## Point-to-Point Connection ##
In the point-to-point connection only one sensor device is connected to the master device using one clock line (MA) and one data output line (SL). Within the sensor device, several sensors can be daisy-chained, e.g. to enable redundant data transmission for safety sytems, smart sensor applications or condition monitoring.


## Bus Connection ##
In the bus connection only one sensor device is connected to the master device using one clock line (MA) and two data lines (SL for slave-to-master and MO for master-to-slave). The SL of the first device is fed into the MO of the second device, which will buffer the incoming data until it is ready to send, and then append its own data after the first slave. Thus multiple devices behave the same as a single device with multiple sensors.
'''

from .pd import Decoder

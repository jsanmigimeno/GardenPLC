# Garden PLC
Arduino controller (designed for the Arduino Nano) to control the irrigation and swimming pool of my garden.
A control circuit has been built with the following specifications:
- 6 Output relays @220VAC
- 6 Digital inputs (for switches) @24VDC
- 8 Latching solenoid valves @9VDC
- RTC clock
- MAX485 serial interface to communicate with the GardenPLCWirelesInterface (which implements an API server).

<br />

# Dependencies
ArduinoThread - Ivan Seidel - 2.1.1
RTClib - Adafruit - 2.0.3
LinkedList - Ivan Seidel - 1.3.3

jsanmigimeno/MAX485


# Structure
## Threads
Task Scheduler Thread
- Handles the execution of the **Irrigation** and **Swimming Pool Controller**.

Communication Thread
- Handles requests made from the GardenPLCWirelessInterface via serial communication.

Electrovalves Controller Thread
- Handles the logic for the irrigation jobs.
- Turns on/off the DC latching electrovalves of the irrigation zones with pulses via a multiplexer (uses 4 select pins + a signal enable pin).
- Implemented as a separate thread as accurate timing is required for the pulses that control the valves' latching solendoids.

## Helper Classes
Data Saver
- Handles data read/writes from/to EEPROM memory.

<br />


# Logic
## Setup
- The **Irrigation** and **Swimming Pool Controllers** are initialised, which themselves initialise the state of the logic pins.
- The **Datasaver** helper class is used to load the state of the controllers.
- The **Electrovalves Control Thread** resets (turns off) all valves upon initialisation. Note that latching solenoid valves do not turn off until a turn-off pulse is sent; if power is lost whilst a solenoid valve is open, it will remain open indefinitely. As a precaution, the mains cut-off solenoid valve is NOT a DC latching one, and hence will close after a power loss.
- The **Task Scheduler Thread** is initialised and the controllers are added to it.
## Main Loop
- The **Task Scheduler Thread** will regularly call the **'runTask()'** method of the irrigation and swimming pool controllers, passing as argumante the state of the PLC (clock timestamp + auto mode state).



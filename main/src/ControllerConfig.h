/*
    ControllerConfig.h

    Configuration of the controller.
*/

#ifndef ControllerConfig_h
#define ControllerConfig_h

#include "PinDefinitions.h"

#define DEBUG_SERIAL Serial

// Outputs
#define SWIMMING_POOL_RECIRCULATION_PUMP_PIN OUTPUT_RELAY_PIN_0
#define UV_DISINFECT_LIGHT_PIN               OUTPUT_RELAY_PIN_1
#define MAINS_WATER_INLET_VALVE_PIN          OUTPUT_RELAY_PIN_4
#define SWIMMING_POOL_IRRIGATION_PUMP_PIN    OUTPUT_RELAY_PIN_5

//Inputs
#define AUTO_MODE_ENABLE_INPUT_PIN                     INPUT_SIGNAL_PIN_1
#define SWIMMING_POOL_PUMP_ENABLE_INPUT_PIN            INPUT_SIGNAL_PIN_2
#define UV_ENABLE_INPUT_PIN                            INPUT_SIGNAL_PIN_3
#define IRRIGATION_FROM_SWIMMING_POOL_ENABLE_INPUT_PIN INPUT_SIGNAL_PIN_4
#define RECIRCULATION_SENSOR_INPUT_PIN                 INPUT_SIGNAL_PIN_5
#define IRRIGATION_PRESSURE_SENSOR_INPUT_PIN           INPUT_SIGNAL_PIN_6

// Irrigation Configuration
#define IRRIGATION_ZONES_COUNT   3   // Up to 4 (expandable to 8)
#define IRRIGATION_SOURCES_COUNT 2   // Up to 2
#define IRRIGATION_GROUPS_COUNT  10  // Subject to EEPROM memory size

// Communication Configuration
#define TIMEOUT_PER_PACKET 100       // ms

#endif

/*
    PinDefinitions.h

    Maps the Arduino Nano's pins to the controller's inputs/outputs.
*/

#define OUTPUT_RELAY_PIN_0 12
#define OUTPUT_RELAY_PIN_1 11
#define OUTPUT_RELAY_PIN_2 10
#define OUTPUT_RELAY_PIN_3 9
#define OUTPUT_RELAY_PIN_4 8
#define OUTPUT_RELAY_PIN_5 7

#define INPUT_SIGNAL_PIN_1 A0
#define INPUT_SIGNAL_PIN_2 A1
#define INPUT_SIGNAL_PIN_3 A2
#define INPUT_SIGNAL_PIN_4 A3
#define INPUT_SIGNAL_PIN_5 A6
#define INPUT_SIGNAL_PIN_6 A7

#define MULTIPLEXER_SELECT_PIN_0 5
#define MULTIPLEXER_SELECT_PIN_1 4
#define MULTIPLEXER_SELECT_PIN_2 3
#define MULTIPLEXER_SELECT_PIN_3 2
#define MULTIPLEXER_SIGNAL_PIN   6

#define COMM_SERIAL Serial // i.e. pins Tx1 and Rx1
#define COMM_TRANSMISSION_ENABLE_PIN 13
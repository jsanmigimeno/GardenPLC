/*
  InterfaceUtils.h
*/
#ifndef InterfaceUtils_h
#define InterfaceUtils_h

#include <Arduino.h>

#define ANALOG_PIN_HIGH_THRESHOLD 800 // TODO CHECK VALUE

class InputSignal
{
    public:
        InputSignal(const uint8_t pinRef) : pinRef(pinRef) {
            setPinMode();
        }

        boolean value() {
            return analogRead(pinRef) >= ANALOG_PIN_HIGH_THRESHOLD;
        }
    private:
        const uint8_t pinRef;

        void setPinMode() {
            pinMode(pinRef, INPUT);
        }

};

class OutputRelay
{
    public:
        OutputRelay(const uint8_t pinRef) : pinRef(pinRef) {
            turnOff();
            setPinMode();
        }

        void turnOn() {            
            digitalWrite(pinRef, LOW); // Relays are active low
            state = true;
        }

        void turnOff() {            
            digitalWrite(pinRef, HIGH); // Relays are active low
            state = false;
        }

        bool getState() {
            return state;
        }

    private:
        const uint8_t pinRef;
        bool state = false;

        void setPinMode() {
            pinMode(pinRef, OUTPUT);
        }

};


#endif
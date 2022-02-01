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
            setPinMode();
        }

        void turnOn() {
            if (state) return;
            
            digitalWrite(pinRef, HIGH);
            state = true;
        }

        void turnOff() {
            if (!state) return;
            
            digitalWrite(pinRef, LOW);
            state = false;
        }

        bool getState() {
            return state;
        }

    private:
        const uint8_t pinRef;
        bool state      = false;

        void setPinMode() {
            pinMode(pinRef, OUTPUT);
        }

};


#endif
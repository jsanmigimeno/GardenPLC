/*
  InterfaceUtils.h
*/
#ifndef InterfaceUtils_h
#define InterfaceUtils_h

#include <Arduino.h>

#define ANALOG_PIN_HIGH_THRESHOLD 800 // TODO CHECK VALUE
#define DEBOUNCE_TIME_MILLIS 200

class InputSignal
{
    public:
        InputSignal(const uint8_t pinRef) : pinRef(pinRef) {
            setPinMode();

            lastUpdateTime = millis();
            _value         = analogRead(pinRef) >= ANALOG_PIN_HIGH_THRESHOLD;
        }

        bool value() {
            uint32_t time = millis();

            bool read_value = analogRead(pinRef) >= ANALOG_PIN_HIGH_THRESHOLD;

            if (read_value == _value) {
                lastUpdateTime = time;
            }
            else if (time - lastUpdateTime >= DEBOUNCE_TIME_MILLIS) {
                lastUpdateTime = time;
                _value         = read_value;
            }

            return _value;
        }

    private:
        const uint8_t pinRef;

        uint32_t lastUpdateTime;
        bool     _value;

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
            digitalWrite(pinRef, HIGH);
            state = true;
        }

        void turnOff() {            
            digitalWrite(pinRef, LOW);
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
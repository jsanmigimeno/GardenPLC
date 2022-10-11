/*
  TaskSchedulerThread.h
*/
#ifndef TaskSchedulerThread_h
#define TaskSchedulerThread_h

#include <Arduino.h>
#include <Thread.h>
#include <RTClib.h>

#include "../Utils/InterfaceUtils.h"


struct PLCState {
    uint32_t time;
    bool autoModeState;
};


class Task
{
    public:
        virtual void runTask(const PLCState& state);
};


static const uint32_t defaultRTCTime = 1640991600;

template <uint8_t T>
class TaskSchedulerThread: public Thread
{
  public:
    TaskSchedulerThread(Task* tasks[T], RTC_DS3231 & rtcClock)
        : clock(rtcClock)
    {
        // Check time
        DateTime date = clock.now();
        const uint32_t rtcTime = date.unixtime();
        if (rtcTime < defaultRTCTime) {
            clock.adjust(DateTime(defaultRTCTime));
        }

        // Save tasks
        for (uint8_t i = 0; i < T; i++) {
            _tasks[i] = tasks[i];
        }
    }

    void run() {
        state.time = clock.now().unixtime();
        if (state.autoModeState != autoEnableSignal->value()) {
            state.autoModeState = autoEnableSignal->value();
            lastChangeTimestamp = state.time;
        }

        for (uint8_t i = 0; i < T; i++) {
            _tasks[i]->runTask(state);
        }

        runned();
    }

    uint32_t getTime() {
        return clock.now().unixtime();
    }

    void setTime(uint32_t time) {
        clock.adjust(DateTime(time));
        lastChangeTimestamp = getTime();
    }

    uint32_t getLastChangeTimestamp() {
        return lastChangeTimestamp;
    }

    bool getAutoModeState() {
        return autoEnableSignal->value();
    }

  
  private:
        RTC_DS3231& clock;
        Task* _tasks[T];

        InputSignal* autoEnableSignal = new InputSignal(AUTO_MODE_ENABLE_INPUT_PIN);

        PLCState state;

        uint32_t lastChangeTimestamp = 0;
};

#endif
/*
  SwimmingPoolController.h
  Implementation of the Swimming Pool Controller logic.

  The function 'runTask' is the main loop of the controller, which is called regularly by the TaskScheduler thread.

  There are two pump turn on modes: manual and automatic.
  Manual Mode:
    - Turned on via a switch on the PLC control panel.
    - If the manual mode is turned on whilst the scheduled mode is ongoing, the manual mode gets
      temporarily disabled until it gets turned off. This is to prevent manual mode from turning on 
      the pump indefinitely after the scheduled mode completes.

  Scheduled Mode:
    - Has to be enabled on the PLC control panel (auto mode).
    - Has to be enabled via the PLC API/Android App.
    - Schedule configuration:
        -- Next start date and time
        -- Schedule Duration
        -- Schedule Period
    - The controller periodically checks if the schedule is due, and once it happens it will turn on
      the swimming pool pump.


  The controller can be in different states, each of which will result in a different instruction loop being triggered
  whenever the main loop of the controller gets triggered.

  IDLE state:
    1. If manual mode is turned on, there is no schedule onging, and the manual mode is not disabled, turn on the manual 
       mode and change the state of the controller to 'MANUAL_JOB'.
    2. If auto mode is enabled on the PLC control panel, and the schedule is enabled (via the API/Android App),
       check the schedlue. If it is due, change the state of the controller to 'SCHEDULED_JOB'.
  
  MANUAL_JOB state:
    1. If manual mode is turned off, turn off the pump and revert the state of the controller to 'IDLE'.
  
  SCHEDULED_JOB state:
    1. If auto mode gets disabled on the PLC control panel or if the schedule gets disabled via the API/Android App, turn 
       off the pump and revert the state of the controller to 'IDLE'.

*/
#ifndef SwimmingPoolController_h
#define SwimmingPoolController_h
#include <Arduino.h>

#include "SwimmingPoolControllerTypes.h"

#include "../Utils/DataSaver.h"
#include "../Utils/InterfaceUtils.h"

#include "../TaskScheduler/TaskSchedulerThread.h"

enum class SwimmingPoolControllerState {
    IDLE = 0,
    MANUAL_JOB,
    SCHEDULED_JOB,
};

class SwimmingPoolController: public Task
{
    public:
        SwimmingPoolController(
            DataSaver*& dataSaver
        );

        const InputSignal* manualOverride                = new InputSignal(SWIMMING_POOL_PUMP_ENABLE_INPUT_PIN);
        const InputSignal* UVEnable                      = new InputSignal(UV_ENABLE_INPUT_PIN);
        const InputSignal* recirculationSensor           = new InputSignal(RECIRCULATION_SENSOR_INPUT_PIN);

        const OutputRelay* swimmingPoolRecirculationPump = new OutputRelay(SWIMMING_POOL_RECIRCULATION_PUMP_PIN);
        const OutputRelay* uvDisinfectLight              = new OutputRelay(UV_DISINFECT_LIGHT_PIN);

        // Reset Method
        void reset();
        void stopJob();

        // Controller Loops
        void runTask(const PLCState& state);
        void idleLoop(const PLCState& state);
        void manualLoop(const PLCState& state);
        void scheduledLoop(const PLCState& state);

        // Controller State Public API
        uint32_t getLastChangeTimestamp();
        uint8_t  getControllerState();
        bool     getRecirculationPumpManualOverrideLockState();

        // Schedule Config Public API
        void enableSchedule();
        void disableSchedule();
        bool isScheduleEnabled();

        uint32_t getNextTurnOnTime();
        void     setNextTurnOnTime(uint32_t nextTurnOnTime);

        uint16_t getDuration();
        void     setDuration(uint16_t duration);

        uint8_t  getPeriodDays();
        void     setPeriodDays(uint8_t periodDays);

    private:
        DataSaver*& dataSaver;

        SwimmingPoolControllerState state;
        uint32_t lastChangeTimestamp = 0;
        bool recirculationPumpManualOverrideLock = true; // Prevents rec. pump turn on if manual override is set whilst in automatic mode        
        
        uint32_t turnOnTime;
        uint32_t nextTurnOffTime;
        bool     recirculationState;
        uint32_t recirculationFlowStartDetectionTime;
        uint32_t recirculationFlowStopDetectionTime;
        bool     UVEnableState = false;

        SwimmingPoolConfig   config;
        SwimmingPoolSchedule schedule;

        void initialise();

        // Helper Methods
        void turnPumpOn(const uint32_t time);
        void turnPumpOff();

        void turnUVOn();
        void turnUVOff();

        // Data Management Methods
        void loadSchedule();
        void saveSchedule();
        void loadConfig();
        void saveConfig();
};

#endif
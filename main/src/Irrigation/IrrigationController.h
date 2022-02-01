/*
  IrrigationController.h
  Implementation of the Irrigation Controller logic.

  The function 'runTask' is the main loop of the controller, which is called regularly by the TaskScheduler thread.

  There are two irrigation modes: manual irrigation and scheduled irrigation:
  Manual Irrigation:
    - Turned on via a switch on the PLC control panel. 
    - The irrigation source and irrigation zones are configured via the PLC API/Android App.
    - If manual irrigation is turned on whilst a scheduled irrigation is ongoing, manual irrigation gets
      temporarily disabled until it gets turned off. This is to prevent manual irrigation from running 
      indefinitely after the scheduled irrigation completes.
    - Note that manual irrigation will run a maximum of 2^16 seconds unless manually disabled.

  Scheduled Irrigation:
    - Has to be enabled on the PLC control panel (auto mode).
    - Has to be enabled via the PLC API/Android App.
    - Up to 10 irrigation groups can be set up, each composed of:
      -- Group Name
      -- Irrigation Source
      -- Irrigation Zones
      -- Irrigation Period
      -- Irrigation Duration
      -- Irrigation Start Time
    - The controller periodically checks if a scheduled irrigation is due, and once it happens it will create
      a new irrigation job via the ElectrovalvesControlThread. Multiple jobs can be scheduled at the same time,
      which will be executed sequentially.
    - A group can also be manually triggered at any time via the PLC API/Android App.

  The controller can be in different states, each of which will result in a different instruction loop being triggered
  whenever the main loop of the controller gets triggered.

  IDLE state:
    1. If manual mode is turned on, there is no scheduled irrigation onging, and the manual mode is not disabled, 
       turn on manual irrigation and change the state of the controller to 'MANUAL_JOB'.
    2. If auto mode is enabled on the PLC control panel, check whether an irrigation group has been manually triggered
       via the API/Android App. If that's the case, turn on that irrigation group and change the state of the controller 
       to 'SCHDULED_JOB'.
    3. If auto mode is enabled on the PLC control panel, and the irrigation schedule is enabled (via the API/Android App),
       check every irrigation group, and turn them on if necessary. If any group is due, change the state of the controller
       to 'SCHEDULED_JOB'.
  
  MANUAL_JOB state:
    1. If manual mode is turned off, or the ElectrovalvesControlThread indicates that there is no job ongoing (e.g. timeout),
       revert the state of the controller to 'IDLE'.
  
  SCHEDULED_JOB state:
    1. If the ElectrovalvesControlThread indicates that there is no job ongoing (i.e. all jobs completed) revert the state 
       of the controller to 'IDLE'.
    2. If auto mode gets disabled on the PLC control panel, cancel all active/pending irrigations and revert the state of 
       the controller to 'IDLE'.
*/

#ifndef IrrigationController_h
#define IrrigationController_h

#include <Arduino.h>
#include <RTClib.h>
#include <LinkedList.h>

#include "ElectrovalvesControlThread.h"
#include "IrrigationControllerTypes.h"

#include "../ControllerConfig.h"
#include "../Utils/DataSaver.h"
#include "../Utils/InterfaceUtils.h"
#include "../TaskScheduler/TaskSchedulerThread.h"

enum class IrrigationControllerState {
    IDLE = 0,
    MANUAL_JOB,
    SCHEDULED_JOB,
};

class IrrigationController: public Task
{
    public:
        IrrigationController(
          ElectrovalvesControlThread*& valvesControllerPtr,
          DataSaver*&                  dataSaver,
          RTC_DS3231&                  rtcClock
        );

        const InputSignal* manualIrrigationEnable = new InputSignal(IRRIGATION_FROM_SWIMMING_POOL_ENABLE_INPUT_PIN);
        const InputSignal* irrigationPressureSensor         = new InputSignal(IRRIGATION_PRESSURE_SENSOR_INPUT_PIN);

        // Reset Methods
        void resetIrrigationManualConfig();
        void resetGroup(uint8_t groupIdx);
        void reset();

        // Controller Loops
        void runTask(const PLCState& state);
        void idleLoop(const PLCState& state);
        void manualLoop(const PLCState& state);
        void scheduledLoop(const PLCState& state);

        // Controller State Methods
        uint32_t getLastChangeTimestamp();
        uint8_t  getControllerState();
        bool     getManualOverrideLockState();
        uint16_t getZonesState();

        // Manual Irrigation Config Public API
        uint16_t getIrrigationManualZones();
        void     setIrrigationManualZones(uint16_t zones);

        uint8_t  getIrrigationManualSource();
        void     setIrrigationManualSource(uint8_t sourceIndex);

        // Irrigation Schedule Config Public API
        void     enableSchedule();
        void     disableSchedule();
        bool     isScheduleEnabled();

        bool     isSchedulePaused();
        uint32_t getScheduleResumeTime();
        void     pauseSchedule(uint32_t resumeTimestamp);
        void     resumeSchedule();
        uint32_t getNextIrrigationTime();

        // Irrigation Groups Public API
        void     enableGroup(uint8_t groupIdx);
        void     disableGroup(uint8_t groupIdx);
        bool     isGroupEnabled(uint8_t groupIdx);
        uint16_t getGroupsEnableState();

        void     getGroupName(uint8_t groupIdx, IrrigationGroupName& groupName);
        void     setGroupName(uint8_t groupIdx, IrrigationGroupName& groupName);

        uint16_t getGroupZones(uint8_t groupIdx);
        void     setGroupZones(uint8_t groupIdx, uint16_t zones);

        uint8_t  getGroupSource(uint8_t groupIdx);
        void     setGroupSource(uint8_t groupIdx, uint8_t sourceIdx);

        uint8_t  getGroupPeriod(uint8_t groupIdx);
        void     setGroupPeriod(uint8_t groupIdx, uint8_t period);

        uint16_t getGroupDuration(uint8_t groupIdx);
        void     setGroupDuration(uint8_t groupIdx, uint16_t duration);

        uint16_t getGroupInitTime(uint8_t groupIdx);
        void     setGroupInitTime(uint8_t groupIdx, uint16_t time);

        uint32_t getGroupNextIrrigationTime(uint8_t groupIdx);

        void     getGroup(uint8_t groupIdx, IrrigationGroup& irrGroup);
        void     updateGroup(uint8_t groupIdx, IrrigationGroup& data);

        void     scheduleGroupNow(uint8_t groupIdx);

    private:
        ElectrovalvesControlThread*& valvesController;
        DataSaver*&                  dataSaver;
        RTC_DS3231&                  clock;

        IrrigationManualConfig   irrigationManualConfig;
        IrrigationScheduleConfig irrigationScheduleConfig;
        IrrigationGroups         irrigationGroups;

        IrrigationControllerState state = IrrigationControllerState::IDLE;
        uint32_t lastChangeTimestamp = 0;
        bool manualIrrigationDisableLock = true; // Prevents manual irrigation turn on if it is set whilst in automatic mode
        LinkedList<uint8_t> manualScheduleQueue = LinkedList<uint8_t>();

        // Irrigation Schedule Functions
        void updateNextIrrigationTime(uint8_t groupIdx);
        bool isPeriodValid(const uint8_t period);

        // Data Management Methods
        void loadData();

        void saveIrrigationScheduleConfig();
        void saveIrrigationManualConfig();
        void saveIrrigationGroups();
        void saveIrrigationGroup(const uint8_t groupIdx);

};

#endif
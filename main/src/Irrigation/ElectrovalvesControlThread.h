/*
  ElectrovalvesControlThread.h

  Handles the logic for the irrigation jobs; i.e. turning on/off the irrigation sources and the irrigation zones.
  - The sources are enabled via the controller's output relays.
  - The irrigation zones are controlled by turning on/off the DC latching-solenoid electrovalves via a multiplexer.

  As multiple irrigation job requests can be triggered at the same time, these are stored in a 'jobQueue'.

  1. When a job is received, the controller will go into the 'startingLoop', which will enable the sources and irrigation
  zones of that job (the irrigation zones are always enabled/disabled one at a time, according to the turn on/off pulse parameters).

  2. Once everything is set, the controller will go into the 'runningLoop', which will wait for the duration of the job
  to ellapse.

  3. Once the job finishes, if there is another job pending with the same irrigation source, the controller will go into the
  'transitionLoop', which will update the irrigation zones without disabling the irrigation source and then go back to the 
  'runningLoop'. Otherwise, the controller goes into the 'stoppingLoop', which will disable the active irrigation zones and source.

  The 'loop' functions are called everytime the 'run' method of this thread is called. However, if a pulse is being triggered,
  the pulse control logic will take precedence over the active 'loop'.

  Note that to turn on/off the electrovalve i, a pulse is sent via the multiplexer's output 2*i / 2*i+1 respectively.
  
*/
#ifndef ElectrovalvesControlThread_h
#define ElectrovalvesControlThread_h

#include <Arduino.h>
#include <Thread.h>
#include <LinkedList.h>

#include "../ControllerConfig.h"
#include "../Utils/InterfaceUtils.h"


enum CancelType {
    CANCEL_CURRENT_JOB = 0,
    CANCEL_ALL_JOBS
};

enum ElectrovalvesControlThreadState {
    IDLE = 0,
    STARTING_JOB,
    RUNNING_JOB,
    TRANS_JOB,
    STOPPING_JOB
};

enum TransitionState {
    TRANS_IDLE = 0,
    OPENING_NEXT,
    CLOSING_CURRENT
};

struct JobConfig {
    uint16_t zones;
    uint8_t  sourceIndex;
    uint16_t duration;
    int8_t   nextPendingZone;
    uint32_t startTimestamp;
};

class ElectrovalvesControlThread: public Thread
{
    public:
        ElectrovalvesControlThread();

        bool addJob(uint16_t electrovalveIndexes, uint8_t sourceIndex, uint16_t duration);
        void cancelCurrentJob();
        void cancelAllJobs();

        bool     isBusy();
        bool     checkChanges();
        uint16_t getValvesState();

        void run();

        const OutputRelay* mainsWaterInletValve       = new OutputRelay(MAINS_WATER_INLET_VALVE_PIN);
        const OutputRelay* swimmingPoolIrrigationPump = new OutputRelay(SWIMMING_POOL_IRRIGATION_PUMP_PIN);
    
    private:
        LinkedList<JobConfig*> jobQueue    = LinkedList<JobConfig*>();
        LinkedList<CancelType> cancelQueue = LinkedList<CancelType>();

        ElectrovalvesControlThreadState _state;

        bool     _pulseActive;
        uint32_t _pulseStartTimestamp;

        uint32_t _sourceEndTimestamp;

        bool changed = false;   // Flag that indicates whether the state of the valves/sources has changed

        TransitionState _transState;

        void initialisePins();

        // Main loops
        void idleLoop();
        void startingLoop();
        void runningLoop();
        void transitionLoop();
        void stoppingLoop();

        // Transition functions
        void trOpenNextJobZones();
        void trCloseCurrentJobZones();

        // Source functions
        void turnOnSource(const uint8_t sourceIndex);
        void turnOffSource(const uint8_t sourceIndex);
        void setSourceState(const uint8_t sourceIndex, const bool state);

        // Irrigation zones functions
        bool setJobZonesState(JobConfig* config, const bool state, uint16_t ignoreZones = 0);
        int8_t getNextZone(uint16_t selectedZones, int8_t currentZoneIndex);
        void turnOnZone(const uint8_t zoneIndex);
        void turnOffZone(const uint8_t zoneIndex);
        void setZonePulse(const uint8_t pulseOutputIndex);
        void unsetZonePulse();
        void setMultInputPins(const uint8_t inputIndex);
        void setMultSignalState(const bool state);

        // Reset/Cancel functions
        void reset();
        void removeCurrentJobFromQueue();
        void removeAllJobsFromQueue();

};

#endif
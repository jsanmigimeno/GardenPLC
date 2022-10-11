/*
  ElectrovalvesControlThread.cpp
*/
#include "ElectrovalvesControlThread.h"

// Time in ms
const uint16_t PULSE_DURATION           = 100;
const uint16_t BETWEEN_PULSES_DURATION  = 100;
const uint16_t BETWEEN_SOURCES_DURATION = 3000;

// Time in us
const uint16_t MULTIPLEXER_SIGNAL_DELAY = 100;


ElectrovalvesControlThread::ElectrovalvesControlThread()
{
    initialisePins();

    // IMPORTANT: Make sure all electrovalves are turned off, as the DC latching solenoid valves will remain
    // indefinitely in the 'on' state until an 'off' pulse is sent; after a power loss, any open electrovalve
    // will not close if the reset method is not called.
    reset();
}

void ElectrovalvesControlThread::initialisePins() {
    // Multiplexer - Zones
    pinMode(MULTIPLEXER_SELECT_PIN_0, OUTPUT);
    pinMode(MULTIPLEXER_SELECT_PIN_1, OUTPUT);
    pinMode(MULTIPLEXER_SELECT_PIN_2, OUTPUT);
    pinMode(MULTIPLEXER_SELECT_PIN_3, OUTPUT);
    pinMode(MULTIPLEXER_SIGNAL_PIN, OUTPUT);
}



// Control functions ************************************************************************************************************

bool ElectrovalvesControlThread::addJob(uint16_t electrovalveIndexes, uint8_t sourceIndex, uint16_t duration) {

    // Validate job parameters are within range
    if (
        (((0xFFFF >> (16-IRRIGATION_ZONES_COUNT)) & electrovalveIndexes) == 0) ||  // Make sure no electrovalves outside the available ones are selected
        (sourceIndex >= IRRIGATION_SOURCES_COUNT)
    ) return false; //TODO NOTE ERROR?

    // Save job
    JobConfig* newJob = new JobConfig();

    newJob->zones           = electrovalveIndexes;
    newJob->sourceIndex     = sourceIndex;
    newJob->duration        = duration;
    newJob->nextPendingZone = -1;

    jobQueue.add(newJob);

    return true;
}

void ElectrovalvesControlThread::cancelCurrentJob(){
    cancelQueue.add(CANCEL_CURRENT_JOB);
}

void ElectrovalvesControlThread::cancelAllJobs(){
    cancelQueue.add(CANCEL_ALL_JOBS);
}



// State functions **************************************************************************************************************

bool ElectrovalvesControlThread::isBusy() {
    return jobQueue.size() > 0;
}

bool ElectrovalvesControlThread::checkChanges(){
    bool tempChanged = changed;
    changed = false;
    return tempChanged;
}

uint16_t ElectrovalvesControlThread::getValvesState(){
    return jobQueue.size() == 0 ? 0 : jobQueue.get(0)->zones;
}


// Thread run function **********************************************************************************************************

void ElectrovalvesControlThread::run() {
    // If a pulse is being triggered, do nothing else until completed
    if (_pulseActive) {
        uint32_t pulseEllapsedTime = millis() - _pulseStartTimestamp;
        if (pulseEllapsedTime >= PULSE_DURATION) unsetZonePulse();

        return runned();
    }

    // If a cancel request has been triggered
    if (cancelQueue.size() > 0) {
        // If starting, transitioning or stopping, wait until complete
        // If running, force stopping
        if (_state == ElectrovalvesControlThreadState::RUNNING_JOB) {
            // Stop current job
            _state = ElectrovalvesControlThreadState::STOPPING_JOB;
        }
        else if (_state == ElectrovalvesControlThreadState::IDLE) {
            // Once on idle state, cancel job(s)
            CancelType cancelRequest = cancelQueue.get(0);
            if (cancelRequest == CANCEL_ALL_JOBS) {
                // Cancel all jobs
                removeAllJobsFromQueue();
                cancelQueue.clear();
            }
            else {
                cancelQueue.remove(0);
            }
            return runned();
        }
    }

    // Trigger the active state loop function
    switch (_state) {
        case ElectrovalvesControlThreadState::IDLE:
            idleLoop();
            break;
        case ElectrovalvesControlThreadState::STARTING_JOB:
            startingLoop();
            break;
        case ElectrovalvesControlThreadState::RUNNING_JOB:
            runningLoop();
            break;
        case ElectrovalvesControlThreadState::TRANS_JOB:
            transitionLoop();
            break;
        case ElectrovalvesControlThreadState::STOPPING_JOB:
            stoppingLoop();
            break;
        default:
            reset(); // Undefined state
        break;
    }
    runned();
}



// Loops ************************************************************************************************************************

void ElectrovalvesControlThread::idleLoop() {
    // Check job queue
    if (jobQueue.size() > 0) {
        _state = ElectrovalvesControlThreadState::STARTING_JOB;
    }
}

void ElectrovalvesControlThread::startingLoop() {

    // Get the job
    JobConfig* activeJobPtr = jobQueue.get(0);

    // First time checks
    if (activeJobPtr->nextPendingZone == -1) {

        // Wait since last source active
        const uint32_t ellapsedSinceLastSource = millis() - _sourceEndTimestamp;
        if (ellapsedSinceLastSource < BETWEEN_SOURCES_DURATION) {
            return;
        }

        activeJobPtr->nextPendingZone = getNextZone(activeJobPtr->zones, -1);
    }

    // Turn on zones
    const bool allZonesTurnedOn = setJobZonesState(activeJobPtr, true);

    // If all zones have been turned on
    if (allZonesTurnedOn) {
        // Turn on the source and register the start timestamp
        turnOnSource(activeJobPtr->sourceIndex);
        activeJobPtr->startTimestamp = millis();

        // Change job state
        _state = ElectrovalvesControlThreadState::RUNNING_JOB;
        changed = true;
    }

}

void ElectrovalvesControlThread::runningLoop() {

    // If the job duration has been reached, change the job state
    JobConfig* activeJobPtr = jobQueue.get(0);
    uint32_t ellapsedTime = (millis() - activeJobPtr->startTimestamp) / 1000;  //TODO IMPLEMENT FAILSAFE IN CASE REMAINING TIME IS TOO LONG?
    
    if (ellapsedTime >= activeJobPtr->duration) {
        // If a next job is set, and it has the same source as the current job, transition
        if (jobQueue.size() > 1 && activeJobPtr->sourceIndex == jobQueue.get(1)->sourceIndex) {
            _state = ElectrovalvesControlThreadState::TRANS_JOB;
        }
        else { // Otherwise stop
            _state = ElectrovalvesControlThreadState::STOPPING_JOB;
        }
    }
}

void ElectrovalvesControlThread::transitionLoop() {
    switch(_transState) {
        case TransitionState::TRANS_IDLE:
            _transState = TransitionState::OPENING_NEXT;
            // Note that there is intentionally no 'break;' at this point

        case TransitionState::OPENING_NEXT:
            trOpenNextJobZones();
            break;

        case TransitionState::CLOSING_CURRENT:
            trCloseCurrentJobZones();
            break;

        default:
            reset(); // Undefined state
            break;
    }
}

void ElectrovalvesControlThread::stoppingLoop() {

    JobConfig* activeJobPtr = jobQueue.get(0);

    // Stop the source
    if (activeJobPtr->nextPendingZone == -1) {
        turnOffSource(activeJobPtr->sourceIndex);
        _sourceEndTimestamp = millis();
        activeJobPtr->nextPendingZone = getNextZone(activeJobPtr->zones, -1);
    }

    const bool allZonesTurnedOff = setJobZonesState(activeJobPtr, false);
    if (allZonesTurnedOff) {
        // Remove job from the queue
        removeCurrentJobFromQueue();
        // Set thread to idle
        _state = ElectrovalvesControlThreadState::IDLE;
        changed = true;
    }

}



// Transition functions *********************************************************************************************************

void ElectrovalvesControlThread::trOpenNextJobZones() {

    JobConfig* currJobPtr = jobQueue.get(0);
    JobConfig* nextJobPtr = jobQueue.get(1);

    // Set start condition
    if (nextJobPtr->nextPendingZone == -1) {
        nextJobPtr->nextPendingZone = getNextZone(nextJobPtr->zones, -1);
    }

    // Turn on the zones of the next job ignoring the zones that are already opened by the current job
    const bool allZonesTurnedOn = setJobZonesState(nextJobPtr, true, currJobPtr->zones);

    // If all zones have been opened
    if (allZonesTurnedOn) {
        nextJobPtr->startTimestamp = millis();              // Set the start time
        _transState = TransitionState::CLOSING_CURRENT;     // Change transition state
    };
}

void ElectrovalvesControlThread::trCloseCurrentJobZones() {

    JobConfig* currJobPtr = jobQueue.get(0);
    JobConfig* nextJobPtr = jobQueue.get(1);

    // Set start condition
    if (currJobPtr->nextPendingZone == -1) {
        currJobPtr->nextPendingZone = getNextZone(currJobPtr->zones, -1);
    }

    // Turn off the zones of the current job ignoring the zones that are required by the next job
    const bool allZonesTurnedOff = setJobZonesState(currJobPtr, false, nextJobPtr->zones);
    
    // If all zones have been closed
    if (allZonesTurnedOff) {
        // Remove job from the queue
        removeCurrentJobFromQueue();
        // Change transition and controller state
        _transState = TransitionState::TRANS_IDLE;
        _state      = ElectrovalvesControlThreadState::RUNNING_JOB;
        changed = true;
    };

}



// Source functions *************************************************************************************************************

void ElectrovalvesControlThread::turnOnSource(const uint8_t sourceIndex) {
    setSourceState(sourceIndex, true);
}

void ElectrovalvesControlThread::turnOffSource(const uint8_t sourceIndex) {
    setSourceState(sourceIndex, false);
}

void ElectrovalvesControlThread::setSourceState(const uint8_t sourceIndex, const bool state) {
    switch (sourceIndex) {
        case 0: state ? mainsWaterInletValve->turnOn() : mainsWaterInletValve->turnOff(); break;
        case 1: state ? swimmingPoolIrrigationPump->turnOn() : swimmingPoolIrrigationPump->turnOff(); break;
    }
}



// Irrigation zones functions ***************************************************************************************************

bool ElectrovalvesControlThread::setJobZonesState(JobConfig* config, const bool state, uint16_t ignoreZones /* = 0 */) {

    uint16_t zones           = config->zones;
    int8_t   nextPendingZone = config->nextPendingZone;

    if (nextPendingZone == IRRIGATION_ZONES_COUNT) {  // This check is here and not at the end of the function to ensure the electrovalve pulse has completed before modifying the source state
        config->nextPendingZone = -1;
        return true; // Job completed
    }

    // Get the time since the last turn on pulse
    uint32_t timeSinceLastPulseStart = millis() - _pulseStartTimestamp;

    // Start a new pulse once BETWEEN_PULSES_DURATION ms has ellapsed
    if (timeSinceLastPulseStart >= PULSE_DURATION + BETWEEN_PULSES_DURATION) {
        // If the zone index is not in the ignore zones variable, send pulse
        if ((ignoreZones & (1 << nextPendingZone)) == 0) { 
            if (state) turnOnZone(nextPendingZone);
            else       turnOffZone(nextPendingZone);
        }

        config->nextPendingZone = getNextZone(zones, nextPendingZone);
    }

    return false;
}


int8_t ElectrovalvesControlThread::getNextZone(uint16_t selectedZones, int8_t currentZoneIndex) {
    // Return the index of the next selected zone

    // The zones configuration (selectedZones) represents which zones are requested by setting the bits corresponding to 
    // the zones indexes to 1: the ith zone is selected if the ith bit of selectedZones is a 1.
    do {
        currentZoneIndex++;
    } while(currentZoneIndex < IRRIGATION_ZONES_COUNT && ((1 << currentZoneIndex) & selectedZones) == 0);

    return currentZoneIndex;
}

void ElectrovalvesControlThread::turnOnZone(const uint8_t zoneIndex) {
    setZonePulse(2 * zoneIndex);
}

void ElectrovalvesControlThread::turnOffZone(const uint8_t zoneIndex) {
    setZonePulse(2 * zoneIndex + 1);
}

void ElectrovalvesControlThread::setZonePulse(const uint8_t pulseOutputIndex) {
    setMultInputPins(pulseOutputIndex);             // Set multiplexer address
    delayMicroseconds(MULTIPLEXER_SIGNAL_DELAY);    // Wait for multiplexer to be set
    setMultSignalState(true);                       // Set signal high

    // Save pulse info
    _pulseActive = true;
    _pulseStartTimestamp = millis();
}

void ElectrovalvesControlThread::unsetZonePulse() {
    setMultSignalState(false);  // Set signal low
    _pulseActive = false;       // Reset pulse info
}

// Set the address of the multiplexer
void ElectrovalvesControlThread::setMultInputPins(const uint8_t inputIndex) {
    digitalWrite(MULTIPLEXER_SELECT_PIN_0, (inputIndex & 1) != 0);
    digitalWrite(MULTIPLEXER_SELECT_PIN_1, (inputIndex & 2) != 0);
    digitalWrite(MULTIPLEXER_SELECT_PIN_2, (inputIndex & 4) != 0);
    digitalWrite(MULTIPLEXER_SELECT_PIN_3, (inputIndex & 8) != 0);
}

// Set the state of the signal going into the multiplexer
void ElectrovalvesControlThread::setMultSignalState(const bool state) {
    digitalWrite(MULTIPLEXER_SIGNAL_PIN, state);
}



// Reset/Cancel functions *******************************************************************************************************

void ElectrovalvesControlThread::reset() {

    // Turn of multiplexer signal
    setMultSignalState(false);
    _pulseActive = false;

    // Reset state variables
    _state = ElectrovalvesControlThreadState::IDLE;
    _transState = TransitionState::TRANS_IDLE;
    _pulseStartTimestamp = millis();
    _sourceEndTimestamp  = millis();

    // Turn off all sources
    for (uint8_t i = 0; i < IRRIGATION_SOURCES_COUNT; i++) {
        turnOffSource(i);
    }

    // Turn off all zones
    for (uint8_t i = 0; i < IRRIGATION_ZONES_COUNT; i++) {
        turnOffZone(i);
        // IMPORTANT - WAIT PULSE_DURATION AND DISABLE THE PULSE MANUALLY, AS THE RUN THREAD FUNCTION IS NOT BEING CALLED
        delay(PULSE_DURATION);
        unsetZonePulse();
        delay(BETWEEN_PULSES_DURATION);
    }

    // Clear queues
    while (jobQueue.size() > 0) {
        JobConfig* jobPtr = jobQueue.get(0);
        delete jobPtr;
        jobQueue.remove(0);
    }

    cancelQueue.clear();
}

void ElectrovalvesControlThread::removeCurrentJobFromQueue() {
    JobConfig* currJobPtr = jobQueue.get(0);
    delete currJobPtr;
    jobQueue.remove(0);
}

void ElectrovalvesControlThread::removeAllJobsFromQueue() {
    // TODO Make sure in idle state?
    while (jobQueue.size() != 0) {
        removeCurrentJobFromQueue();
    }
}
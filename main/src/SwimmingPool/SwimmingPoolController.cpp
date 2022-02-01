#include "SwimmingPoolController.h"
/*
  SwimmingPoolController.h
  Implementation of the Swimming Pool Controller logic.
*/

SwimmingPoolController::SwimmingPoolController(
    DataSaver*& dataSaver
) : dataSaver(dataSaver)
{
    initialise();
    loadConfig();
    loadSchedule();
}

void SwimmingPoolController::initialise() {
    state = SwimmingPoolControllerState::IDLE;

    recirculationPumpManualOverrideLock = true;

    turnOnTime                          = 0;
    nextTurnOffTime                     = 0;
    recirculationFlowStartDetectionTime = 0;
    recirculationFlowStopDetectionTime  = 0;
}

void SwimmingPoolController::reset() {
    turnPumpOff();
    turnUVOff();

    config.maxScheduledTurnOnTimeout         = 3600;
    config.minScheduledDuration              = 5*60;
    config.maxScheduledDuration              = 12*3600;
    config.recirculationMaxTurnOnTimeout     = 30;
    config.recirculationStopDetectionTimeout = 5;
    config.uvTurnOnOffDelay                  = 5;
    saveConfig();

    schedule.scheduleEnable = false;
    schedule.nextTurnOnTime = 4294967295; // Set to max value 2^32-1
    schedule.duration       = 0;          // Set to min value
    schedule.periodDays     = 255;        // Set to max value 2^8-1
    saveSchedule();
    
    initialise();
}



// Controller Loops *************************************************************************************************************

void SwimmingPoolController::runTask(const PLCState& plcState) {

    // Read recirculation sensor
    recirculationState = recirculationSensor->value();
    if (recirculationState) {
        if (recirculationFlowStartDetectionTime == 0) {
            recirculationFlowStartDetectionTime = plcState.time;
            recirculationFlowStopDetectionTime  = 0;
        }
    }
    else {
        if (recirculationFlowStopDetectionTime == 0) {
            recirculationFlowStartDetectionTime = 0;
            recirculationFlowStopDetectionTime  = plcState.time;
        }
    }

    switch(state) {
        case SwimmingPoolControllerState::IDLE:
            idleLoop(plcState);
            break;
    
        case SwimmingPoolControllerState::MANUAL_JOB:
            manualLoop(plcState);
            break;

        case SwimmingPoolControllerState::SCHEDULED_JOB:
            scheduledLoop(plcState);
            break;
    }


    if (UVEnable->value() != UVEnableState) {
        UVEnableState = UVEnable->value();
        lastChangeTimestamp = plcState.time;
    }

    // The UV disinfector logic is independent of the operational mode of the controller
    // Turn on UV disinfector
    if (
        !uvDisinfectLight->getState() &&                                                 // The UV-C light is off AND
        UVEnable->value() &&                                                             // the UV-C light is enabled AND
        swimmingPoolRecirculationPump->getState() &&                                     // the recirculation pump is turned on AND
        recirculationState &&                                                            // there is a recirculation flow detected AND
        (plcState.time - recirculationFlowStartDetectionTime) >= config.uvTurnOnOffDelay // $(config.uvTurnOnOffDelay) have passed since a flow has been detected
    ) {
        turnUVOn();
        lastChangeTimestamp = plcState.time;
    }
    // Turn off the UV disinfector
    else if (
        uvDisinfectLight->getState() && (                                                                        // The UV-C light is on AND
            !UVEnable->value() ||                                                                                     // the UV-C light is disabled OR
            !swimmingPoolRecirculationPump->getState() ||                                                            // the recirculation pump is turned off OR
            (!recirculationState && ((plcState.time - recirculationFlowStopDetectionTime) >= config.uvTurnOnOffDelay))    // $(config.uvTurnOnOffDelay) have passed since a flow stop has been detected
        )
    ) { // Turn off UV disinfector
        turnUVOff();
        lastChangeTimestamp = plcState.time;
    }
}

void SwimmingPoolController::idleLoop(const PLCState& plcState) {
    // Manual mode check
    if (manualOverride->value()) {
        if (!recirculationPumpManualOverrideLock) { // Make sure manual mode isn't disabled
            // Turn pump on
            turnPumpOn(plcState.time);
            state = SwimmingPoolControllerState::MANUAL_JOB;
            lastChangeTimestamp = plcState.time;
            return;
        }
        // Manual override is disabled, continue
    }
    else {
        if (recirculationPumpManualOverrideLock) {
            recirculationPumpManualOverrideLock = false;
            lastChangeTimestamp = plcState.time;
        }
    }

    // Scheduled mode check
    if (
        plcState.autoModeState && 
        schedule.scheduleEnable &&
        plcState.time >= schedule.nextTurnOnTime
    ) {
        // Check turn on timeout (i.e. not too much time has passed since the scheduled turn on time), and job duration
        if (
            (plcState.time - schedule.nextTurnOnTime) <= config.maxScheduledTurnOnTimeout &&
            schedule.duration*60 >= config.minScheduledDuration
        ) {
            turnPumpOn(plcState.time);
            nextTurnOffTime = plcState.time + schedule.duration*60;
            state = SwimmingPoolControllerState::SCHEDULED_JOB;
        }
        else {
            // TODO NOTE ERROR SOMEHOW?
        }

        // Compute next turn on time
        const uint32_t periodInSeconds = (schedule.periodDays == 0 ? 1 : schedule.periodDays)*86400;
        schedule.nextTurnOnTime += (floor( (plcState.time - schedule.nextTurnOnTime)/periodInSeconds ) + 1)*periodInSeconds;

        saveSchedule();
        lastChangeTimestamp = plcState.time;
    }
}

void SwimmingPoolController::manualLoop(const PLCState& plcState) {
    if (!manualOverride->value()) {
        turnPumpOff();
        state = SwimmingPoolControllerState::IDLE;
        lastChangeTimestamp = plcState.time;
    }
}

void SwimmingPoolController::scheduledLoop(const PLCState& plcState) {
    
    // If the manual pump turn on override is set whilst the pump is in auto mode, disable manual override
    // (avoid the pump turning on indefinately after the scheduled timer finishes)
    if (manualOverride->value()) {
        if (!recirculationPumpManualOverrideLock) {
            recirculationPumpManualOverrideLock = true;
            lastChangeTimestamp = plcState.time;
        }
    }
    else {
        if (recirculationPumpManualOverrideLock) {
            recirculationPumpManualOverrideLock = false;
            lastChangeTimestamp = plcState.time;
        }
    }

    // Turn off the pump if auto mode gets disabled, if the schedule gets disabled, or if the scheduled timer completes
    if (
        !plcState.autoModeState || 
        !schedule.scheduleEnable ||
        plcState.time >= nextTurnOffTime
    ) {
        turnPumpOff();
        state = SwimmingPoolControllerState::IDLE;
        lastChangeTimestamp = plcState.time;
        return;
    }

    // Failsafe - nextTurnOffTime is too far away (rtc time change?)
    if ((nextTurnOffTime - plcState.time) >= config.maxScheduledDuration) {
        turnPumpOff();
        state = SwimmingPoolControllerState::IDLE;
        lastChangeTimestamp = plcState.time;
        return;
        // TODO note error?
    }

    // Turn off the pump automatically if the flow sensor stops detecting a recirculation flow
    if (
        !recirculationState &&
        (plcState.time - turnOnTime) >= config.recirculationMaxTurnOnTimeout &&
        (plcState.time - recirculationFlowStopDetectionTime) >= config.recirculationStopDetectionTimeout
    ) {
        turnPumpOff();
        state = SwimmingPoolControllerState::IDLE;
        lastChangeTimestamp = plcState.time;
        // TODO note error?
    }

}



// Controller State Public API **************************************************************************************************

uint32_t SwimmingPoolController::getLastChangeTimestamp() {
    return lastChangeTimestamp;
}

uint8_t SwimmingPoolController::getControllerState() {
    return (uint8_t) state;
}

bool SwimmingPoolController::getRecirculationPumpManualOverrideLockState() {
    return recirculationPumpManualOverrideLock;
}



// Schedule Config Public API ***************************************************************************************************
void SwimmingPoolController::enableSchedule() {
    schedule.scheduleEnable = true;
    lastChangeTimestamp++;
    saveSchedule();
}

void SwimmingPoolController::disableSchedule() {
    schedule.scheduleEnable = false;
    lastChangeTimestamp++;
    saveSchedule();
}

bool SwimmingPoolController::isScheduleEnabled() {
    return schedule.scheduleEnable;
}


uint32_t SwimmingPoolController::getNextTurnOnTime() {
    return schedule.nextTurnOnTime;
}

void SwimmingPoolController::setNextTurnOnTime(uint32_t nextTurnOnTime) {
    schedule.nextTurnOnTime = nextTurnOnTime;
    lastChangeTimestamp++;
    saveSchedule();
}


uint16_t SwimmingPoolController::getDuration() {
    return schedule.duration;
}

void SwimmingPoolController::setDuration(uint16_t duration) {
    schedule.duration = duration;
    lastChangeTimestamp++;
    saveSchedule();
}


uint8_t SwimmingPoolController::getPeriodDays() {
    return schedule.periodDays;
}

void SwimmingPoolController::setPeriodDays(uint8_t periodDays) {
    if (periodDays == 0) return;
    schedule.periodDays = periodDays;
    lastChangeTimestamp++;
    saveSchedule();
}

void SwimmingPoolController::stopJob() {
    nextTurnOffTime = 0;
}



// Helper Methods ***************************************************************************************************************

void SwimmingPoolController::turnPumpOn(const uint32_t time) {
    swimmingPoolRecirculationPump->turnOn();
    turnOnTime = time;
}

void SwimmingPoolController::turnPumpOff() {
    swimmingPoolRecirculationPump->turnOff();
    turnOnTime = 0;
}

void SwimmingPoolController::turnUVOn() {
    uvDisinfectLight->turnOn();
}

void SwimmingPoolController::turnUVOff() {
    uvDisinfectLight->turnOff();
}



// Data Management Methods ******************************************************************************************************

void SwimmingPoolController::loadSchedule() {
    dataSaver->getSwimmingPoolSchedule(schedule);
    // TODO CHECK LOADED DATA
}

void SwimmingPoolController::saveSchedule() {
    dataSaver->saveSwimmingPoolSchedule(schedule);
}

void SwimmingPoolController::loadConfig() {
    dataSaver->getSwimmingPoolConfig(config);
    // TODO CHECK LOADED DATA
}

void SwimmingPoolController::saveConfig() {
    dataSaver->saveSwimmingPoolConfig(config);
}
/*
  IrrigationController.cpp

  Implementation of the Irrigation Controller logic. 
*/

#include "IrrigationController.h"


IrrigationController::IrrigationController(
  ElectrovalvesControlThread*& valvesControllerPtr,
  DataSaver*&                  dataSaver,
  RTC_DS3231 &                 rtcClock
) : dataSaver(dataSaver), valvesController(valvesControllerPtr), clock(rtcClock)
{
  loadData();
}



// Reset Methods ****************************************************************************************************************

void IrrigationController::resetIrrigationManualConfig() {
  irrigationManualConfig.zones = 0;
  irrigationManualConfig.sourceIndex = 0;
  saveIrrigationManualConfig();
}

void IrrigationController::resetGroup(uint8_t groupIdx) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?
  irrigationGroups[groupIdx].enabled          = false;
  irrigationGroups[groupIdx].zones            = 0;
  irrigationGroups[groupIdx].source           = 0;
  irrigationGroups[groupIdx].period           = 24;
  irrigationGroups[groupIdx].duration         = 0;
  irrigationGroups[groupIdx].time             = 0;
  irrigationGroups[groupIdx].nextTimestamp    = 0;

  memset(irrigationGroups[groupIdx].name, 0, IRRIGATION_GROUP_NAME_LENGTH);
  saveIrrigationGroup(groupIdx);
}

void IrrigationController::reset() {
  irrigationScheduleConfig.state = false;
  irrigationScheduleConfig.disabledUntilTimestamp = 0;
  irrigationScheduleConfig.maxScheduledTurnOnTimeout = 60*60*6;
  irrigationScheduleConfig.minScheduledDuration = 10;
  irrigationScheduleConfig.maxScheduledDuration = 30*60;
  saveIrrigationScheduleConfig();

  resetIrrigationManualConfig();

  for (uint8_t i = 0; i < IRRIGATION_GROUPS_COUNT; i++) {
    resetGroup(i);
  }
}



// Controller Loops *************************************************************************************************************

void IrrigationController::runTask(const PLCState& plcState) {

  switch(state) {
    case IrrigationControllerState::IDLE:
      idleLoop(plcState);
      break;
  
    case IrrigationControllerState::MANUAL_JOB:
      manualLoop(plcState);
      break;

    case IrrigationControllerState::SCHEDULED_JOB:
      scheduledLoop(plcState);
      break;
  }

  if (valvesController->checkChanges()) {
    lastChangeTimestamp = plcState.time;
  }

}

void IrrigationController::idleLoop(const PLCState& plcState) {
  // Turn on manual irrigation if the manual switch is turned on, there is no active job, and manual irrigation is not disabled
  if (manualIrrigationEnable->value() && !valvesController->isBusy()) {
    if (!manualIrrigationDisableLock) {
      // Turn on manual irrigation
      const bool success = valvesController->addJob(
        irrigationManualConfig.zones,
        irrigationManualConfig.sourceIndex,
        0xFFFF // Manual irrigation will run 2^16 seconds unless manually disabled //TODO decrease time?
      );

      if (!success) manualIrrigationDisableLock = true;   // Disable manual irrigation if the job failed to prevent an infinite loop (e.g. the manual zones configuration is invalid)
      else state = IrrigationControllerState::MANUAL_JOB;

      return;
    }

    // Manual override is disabled, continue
  }
  else {
    // If manual irrigation is disabled, and the manual switch is turned off, re-enable the manual irrigation
    if (manualIrrigationDisableLock) {
      manualIrrigationDisableLock = false;
      lastChangeTimestamp = plcState.time;
    }
  }

  // If auto mode is enabled
  if (plcState.autoModeState) {

    // Check the manual schedule queue - that is, jobs that have been manually scheduled via the PLC communication API
    if (manualScheduleQueue.size() > 0) {
      const uint8_t groupIdx = manualScheduleQueue.shift();
      if (groupIdx < IRRIGATION_GROUPS_COUNT) {
        IrrigationGroup& irrGroup = irrigationGroups[groupIdx];
        if (
          (irrGroup.duration < irrigationScheduleConfig.maxScheduledDuration) &&
          (irrGroup.duration > irrigationScheduleConfig.minScheduledDuration)
        ) {
          valvesController->addJob(
            irrGroup.zones,
            irrGroup.source,
            irrGroup.duration
          );
          state = IrrigationControllerState::SCHEDULED_JOB;
        }
      }
      return;
    }

    // Check the scheduled irrigation (if it is enabled)
    if (irrigationScheduleConfig.state) {

      // Check if the schedule is paused
      if (irrigationScheduleConfig.disabledUntilTimestamp != 0) {
        if (plcState.time < irrigationScheduleConfig.disabledUntilTimestamp) return;
        else {
          // Resume time reached
          irrigationScheduleConfig.disabledUntilTimestamp = 0;
          saveIrrigationScheduleConfig();
          lastChangeTimestamp = plcState.time;
        }
      }

      // Loop through irrigation groups
      for (uint8_t i = 0; i < IRRIGATION_GROUPS_COUNT; i++) {

        IrrigationGroup& irrGroup = irrigationGroups[i];

        // Check if disabled
        if (!irrGroup.enabled) continue;

        // Check if timestamp has been reached
        if (irrGroup.nextTimestamp <= plcState.time) {

          bool scheduleMissed = plcState.time - irrGroup.nextTimestamp >= irrigationScheduleConfig.maxScheduledTurnOnTimeout;

          if (
            !scheduleMissed &&
            (irrGroup.duration < irrigationScheduleConfig.maxScheduledDuration) &&
            (irrGroup.duration > irrigationScheduleConfig.minScheduledDuration)
          ) {
            valvesController->addJob(
              irrGroup.zones,
              irrGroup.source,
              irrGroup.duration
            );
            state = IrrigationControllerState::SCHEDULED_JOB;
          }

          // Update nextTimestamp
          const uint32_t periodSeconds = (
            (uint32_t) min((irrGroup.period == 0 ? 24 : irrGroup.period), scheduleMissed ? 24 : 0xFFFFFFFF) // If the schedule was missed, set the period to max 24h (ensure irrigation before the next day)
          ) * 60 * 60;

          const uint32_t currentTimestamp = irrGroup.nextTimestamp;
          const uint32_t steps            = floor( (plcState.time - currentTimestamp) / ((float) periodSeconds) ) + 1;

          irrGroup.nextTimestamp = currentTimestamp + steps * periodSeconds;

          saveIrrigationGroup(i);

          lastChangeTimestamp = plcState.time;
        }

      }
    }
  }
  else {
    // If auto mode is not enabled in the control panel, clear the manual schedule queue (prevent jobs from being
    // indefinately scheduled until auto mode gets enabled).
    if (manualScheduleQueue.size() > 0) manualScheduleQueue.clear();
  }
}

void IrrigationController::manualLoop(const PLCState& plcState) {
  if (!manualIrrigationEnable->value() || !valvesController->isBusy()) {
    valvesController->cancelCurrentJob();
    state = IrrigationControllerState::IDLE;
    lastChangeTimestamp = plcState.time;
  }
}

void IrrigationController::scheduledLoop(const PLCState& plcState) {
  // Lock manual irrigation if it is switched on whilst a scheduled irrigation is active.
  if (manualIrrigationEnable->value() && !manualIrrigationDisableLock) {
    manualIrrigationDisableLock = true;
    lastChangeTimestamp = plcState.time;
  }
  else if (!manualIrrigationEnable->value() && manualIrrigationDisableLock) {
    manualIrrigationDisableLock = false;
    lastChangeTimestamp = plcState.time;
  }
  
  // Change state to idle if either the scheduled irrigation completes or auto mode gets disabled
  if (!valvesController->isBusy()) state = IrrigationControllerState::IDLE;
  else if (!plcState.autoModeState) {
    valvesController->cancelAllJobs();
    state = IrrigationControllerState::IDLE;
  }

  // TODO add irrigation pressure sensor safety cut-off? (as in swimming pool controller)

}


// Controller State Methods *****************************************************************************************************

uint32_t IrrigationController::getLastChangeTimestamp() {
  return lastChangeTimestamp;
}

uint8_t IrrigationController::getControllerState() {
  return (uint8_t) state;
}

bool IrrigationController::getManualOverrideLockState() {
  return manualIrrigationDisableLock;
}

uint16_t IrrigationController::getZonesState() {
  return valvesController->getValvesState();
}



// Manual Irrigation Config Public API ******************************************************************************************

uint16_t IrrigationController::getIrrigationManualZones() {
  return irrigationManualConfig.zones;
}

void IrrigationController::setIrrigationManualZones(uint16_t zones) {
  irrigationManualConfig.zones = zones;
  lastChangeTimestamp++;
  saveIrrigationManualConfig();
}


uint8_t IrrigationController::getIrrigationManualSource() {
  return irrigationManualConfig.sourceIndex;
}

void IrrigationController::setIrrigationManualSource(uint8_t sourceIndex) {
  irrigationManualConfig.sourceIndex = sourceIndex;
  lastChangeTimestamp++;
  saveIrrigationManualConfig();
}



// Irrigation Schedule Config Public API ****************************************************************************************

void IrrigationController::enableSchedule() {
  irrigationScheduleConfig.state = true;
  lastChangeTimestamp++;
  saveIrrigationScheduleConfig();
}

void IrrigationController::disableSchedule() {
  irrigationScheduleConfig.state = false;
  lastChangeTimestamp++;
  saveIrrigationScheduleConfig();
}

bool IrrigationController::isScheduleEnabled() {
  return irrigationScheduleConfig.state;
}


bool IrrigationController::isSchedulePaused() {
  return irrigationScheduleConfig.disabledUntilTimestamp != 0;
}

uint32_t IrrigationController::getScheduleResumeTime() {
  return irrigationScheduleConfig.disabledUntilTimestamp;
}

void IrrigationController::pauseSchedule(uint32_t resumeTimestamp) {
  irrigationScheduleConfig.disabledUntilTimestamp = resumeTimestamp;
  saveIrrigationScheduleConfig();
  lastChangeTimestamp++;
}

void IrrigationController::resumeSchedule() {
  irrigationScheduleConfig.disabledUntilTimestamp = 0;
  saveIrrigationScheduleConfig();
  lastChangeTimestamp++;
}

uint32_t IrrigationController::getNextIrrigationTime() {
  uint32_t nextTimestamp = 0xFFFFFFFF;
  if (!isScheduleEnabled()) return nextTimestamp;
  // TODO correct timestamp if irrigation is paused
  for (uint8_t i = 0; i < IRRIGATION_GROUPS_COUNT; i++) {
    if (isGroupEnabled(i)) {
      if (irrigationGroups[i].nextTimestamp < nextTimestamp) nextTimestamp = irrigationGroups[i].nextTimestamp;
    }
  }
  return nextTimestamp;
}


// Irrigation Groups Public API *************************************************************************************************

void IrrigationController::enableGroup(uint8_t groupIdx) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?

  updateNextIrrigationTime(groupIdx);

  irrigationGroups[groupIdx].enabled = true;
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}
        
void IrrigationController::disableGroup(uint8_t groupIdx) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?

  irrigationGroups[groupIdx].enabled = false;
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);  
}
        
bool IrrigationController::isGroupEnabled(uint8_t groupIdx) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return false; // TODO NOTE ERROR?
  return irrigationGroups[groupIdx].enabled;
}


uint16_t IrrigationController::getGroupsEnableState() {
  uint16_t state = 0;

  for (uint8_t i = 0; i < IRRIGATION_GROUPS_COUNT; i++) {
    state = state | (isGroupEnabled(i) << i);
  }
  
  return state;
}
        
void IrrigationController::getGroupName(uint8_t groupIdx, IrrigationGroupName& groupName) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?
  memcpy(groupName, &(irrigationGroups[groupIdx].name), IRRIGATION_GROUP_NAME_LENGTH);
  saveIrrigationGroup(groupIdx);
}

void IrrigationController::setGroupName(uint8_t groupIdx, IrrigationGroupName& groupName) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?
  memcpy(&(irrigationGroups[groupIdx].name), groupName, IRRIGATION_GROUP_NAME_LENGTH);
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}
        
uint16_t IrrigationController::getGroupZones(uint8_t groupIdx) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return 0; // TODO NOTE ERROR?
  return irrigationGroups[groupIdx].zones;
}

void IrrigationController::setGroupZones(uint8_t groupIdx, uint16_t zones) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?
  irrigationGroups[groupIdx].zones = zones;
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}
        
uint8_t IrrigationController::getGroupSource(uint8_t groupIdx) {
  return irrigationGroups[groupIdx].source;
}

void IrrigationController::setGroupSource(uint8_t groupIdx, uint8_t sourceIdx) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?
  irrigationGroups[groupIdx].source = sourceIdx;
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}
        
uint8_t IrrigationController::getGroupPeriod(uint8_t groupIdx) {
  return irrigationGroups[groupIdx].period;
}

void IrrigationController::setGroupPeriod(uint8_t groupIdx, uint8_t period) {
  if (!isPeriodValid(period)) return; //TODO note error?
  irrigationGroups[groupIdx].period = period;
  updateNextIrrigationTime(groupIdx);
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}
        
uint16_t IrrigationController::getGroupDuration(uint8_t groupIdx) {
  return irrigationGroups[groupIdx].duration;
}

void IrrigationController::setGroupDuration(uint8_t groupIdx, uint16_t duration) {
  if (duration < irrigationScheduleConfig.minScheduledDuration) return;
  irrigationGroups[groupIdx].duration = duration;
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}
        
uint16_t IrrigationController::getGroupInitTime(uint8_t groupIdx) {
  return irrigationGroups[groupIdx].time;
}

void IrrigationController::setGroupInitTime(uint8_t groupIdx, uint16_t time) {
  irrigationGroups[groupIdx].time = time;
  updateNextIrrigationTime(groupIdx);
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}

uint32_t IrrigationController::getGroupNextIrrigationTime(uint8_t groupIdx) {
  return irrigationGroups[groupIdx].nextTimestamp;
}
void IrrigationController::getGroup(uint8_t groupIdx, IrrigationGroup& irrGroup) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?
  memcpy(&irrGroup, &irrigationGroups[groupIdx], sizeof(IrrigationGroup));
}

void IrrigationController::updateGroup(uint8_t groupIdx, IrrigationGroup& data) {
  if (groupIdx >= IRRIGATION_GROUPS_COUNT) return; // TODO NOTE ERROR?
  memcpy(&(irrigationGroups[groupIdx]), &data, sizeof(data));
  lastChangeTimestamp++;
  saveIrrigationGroup(groupIdx);
}

void IrrigationController::scheduleGroupNow(uint8_t groupIdx) {
  manualScheduleQueue.add(groupIdx);
}



// Irrigation Schedule Functions ************************************************************************************************

void IrrigationController::updateNextIrrigationTime(uint8_t groupIdx) {

  // Update next timestamp
  IrrigationGroup& groupData = irrigationGroups[groupIdx];

  const uint16_t hours = groupData.time/60;
  const uint16_t minutes = groupData.time - hours*60;
  DateTime now = clock.now();
  DateTime nextIrr = DateTime(now.year(), now.month(), now.day(), hours, minutes);
  if (nextIrr < now) nextIrr = nextIrr + TimeSpan(1, 0, 0, 0);

  groupData.nextTimestamp = nextIrr.unixtime();
  saveIrrigationGroup(groupIdx);
}


bool IrrigationController::isPeriodValid(const uint8_t period) {
  // If the period is less than 24 h, it must divide 24h without a remainder
  // Otherwise it must be a multiple of 24
  if (period == 0) return false;
  if (period < 24) return 24 % period == 0;
  return period % 24 == 0;
}



// Data Management Methods ******************************************************************************************************

void IrrigationController::loadData() {
  dataSaver->getIrrigationManualConfig(irrigationManualConfig);
  dataSaver->getIrrigationScheduleConfig(irrigationScheduleConfig);
  dataSaver->getGroups(irrigationGroups);
}

void IrrigationController::saveIrrigationScheduleConfig() {
  dataSaver->saveIrrigationScheduleConfig(irrigationScheduleConfig);
}

void IrrigationController::saveIrrigationManualConfig() {
  dataSaver->saveIrrigationManualConfig(irrigationManualConfig);
}

void IrrigationController::saveIrrigationGroups() {
  dataSaver->saveIrrigationGroups(irrigationGroups);
}

void IrrigationController::saveIrrigationGroup(const uint8_t groupIdx) {
  dataSaver->saveIrrigationGroup(groupIdx, irrigationGroups[groupIdx]);
}

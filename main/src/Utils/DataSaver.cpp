/*
  DataSaver.cpp
*/
#include "DataSaver.h"

const uint16_t ENABLED_OFFSET           = 16;
const uint16_t ZONES_OFFSET             = ENABLED_OFFSET           + 1;
const uint16_t IRRIGATION_SOURCE_OFFSET = ZONES_OFFSET             + 2;
const uint16_t MIN_PERIOD_OFFSET        = IRRIGATION_SOURCE_OFFSET + 1;
const uint16_t MAX_PERIOD_OFFSET        = MIN_PERIOD_OFFSET        + 1;
const uint16_t MIN_DURATION_OFFSET      = MAX_PERIOD_OFFSET        + 1;
const uint16_t MAX_DURATION_OFFSET      = MIN_DURATION_OFFSET      + 2;
const uint16_t TIME_OFFSET              = MAX_DURATION_OFFSET      + 2;
const uint16_t NEXT_TIMESTAMP_OFFSET    = TIME_OFFSET              + 2;


bool DataSaver::isInitialised() {
  int initFlag;
  EEPROM.get(INITIALISED_ADDR, initFlag);
  return initFlag == INITIALISED_FLAG_VALUE;
}

void DataSaver::setInitialisedFlag() {
  EEPROM.put(INITIALISED_ADDR, INITIALISED_FLAG_VALUE);
}

void DataSaver::resetInitialisedFlag() {
  EEPROM.put(INITIALISED_ADDR, UNINITIALISED_FLAG_VALUE);
}



// Swimming Pool ****************************************************************************************************************

void DataSaver::getSwimmingPoolConfig(SwimmingPoolConfig& config){
  EEPROM.get(SwimmingPoolConfig_ADDR, config);
}

void DataSaver::saveSwimmingPoolConfig(const SwimmingPoolConfig& config){
  EEPROM.put(SwimmingPoolConfig_ADDR, config);
}


void DataSaver::getSwimmingPoolSchedule(SwimmingPoolSchedule& schedule){
  EEPROM.get(SwimmingPoolSchedule_ADDR, schedule);
}

void DataSaver::saveSwimmingPoolSchedule(const SwimmingPoolSchedule& schedule){
  EEPROM.put(SwimmingPoolSchedule_ADDR, schedule);
}



// Irrigation *******************************************************************************************************************

void DataSaver::getIrrigationManualConfig(IrrigationManualConfig& config){
  EEPROM.get(IRRIGATION_MANUAL_CONFIG_ADDR, config);
}

void DataSaver::saveIrrigationManualConfig(IrrigationManualConfig& config) {
  EEPROM.put(IRRIGATION_MANUAL_CONFIG_ADDR, config);
}


void DataSaver::getIrrigationScheduleConfig(IrrigationScheduleConfig& config){
  EEPROM.get(IRRIGATION_SCHEDULE_CONFIG_ADDR, config);  
}

void DataSaver::saveIrrigationScheduleConfig(IrrigationScheduleConfig& config) {
  EEPROM.put(IRRIGATION_SCHEDULE_CONFIG_ADDR, config);
}


void DataSaver::getGroups(IrrigationGroups& irrigationGroupsConfig){
  EEPROM.get(IRRIGATION_GROUPS_ADDR, irrigationGroupsConfig);
}
void DataSaver::saveIrrigationGroups(IrrigationGroups& irrigationGroupsConfig) {
  EEPROM.put(IRRIGATION_GROUPS_ADDR, irrigationGroupsConfig);
}


void DataSaver::getGroup(const uint8_t groupIdx, IrrigationGroup& irrigationGroup) {
  const uint16_t groupAddrOffset = groupIdx * sizeof(IrrigationGroup);
  EEPROM.get(IRRIGATION_GROUPS_ADDR + groupAddrOffset, irrigationGroup);
}

void DataSaver::saveIrrigationGroup(const uint8_t groupIdx, IrrigationGroup& irrigationGroup) {
  const uint16_t saveAddrOffset = groupIdx * sizeof(IrrigationGroup);
  EEPROM.put(IRRIGATION_GROUPS_ADDR + saveAddrOffset, irrigationGroup);
}


void DataSaver::saveIrrigationGroupNextTimestamp(const uint8_t groupIdx, long nextTimestamp) {
  const uint16_t saveAddrOffset = groupIdx * sizeof(IrrigationGroup) + NEXT_TIMESTAMP_OFFSET;
  EEPROM.put(IRRIGATION_GROUPS_ADDR + saveAddrOffset, nextTimestamp);
}

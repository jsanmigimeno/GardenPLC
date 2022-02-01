/*
  DataSaver.h
*/
#ifndef DataSaver_h
#define DataSaver_h

#include <Arduino.h>
#include <EEPROM.h>

#include "../ControllerConfig.h"
#include "../Irrigation/IrrigationControllerTypes.h"
#include "../Irrigation/ElectrovalvesControlThread.h"
#include "../SwimmingPool/SwimmingPoolControllerTypes.h"

#define UNINITIALISED_FLAG_VALUE 0
#define INITIALISED_FLAG_VALUE 1 // Cannot be 255! (Default memory value)

const int INITIALISED_ADDR                = 0;
const int SwimmingPoolConfig_ADDR         = INITIALISED_ADDR + 1;
const int SwimmingPoolSchedule_ADDR       = SwimmingPoolConfig_ADDR + sizeof(SwimmingPoolConfig);
const int IRRIGATION_MANUAL_CONFIG_ADDR   = SwimmingPoolSchedule_ADDR + sizeof(SwimmingPoolSchedule);
const int IRRIGATION_SCHEDULE_CONFIG_ADDR = IRRIGATION_MANUAL_CONFIG_ADDR + sizeof(IrrigationManualConfig);
const int IRRIGATION_GROUPS_ADDR          = IRRIGATION_SCHEDULE_CONFIG_ADDR + sizeof(IrrigationScheduleConfig);

class DataSaver
{
    public:
        bool isInitialised();
        void setInitialisedFlag();
        void resetInitialisedFlag();

        // Swimming Pool
        void getSwimmingPoolConfig(SwimmingPoolConfig& config);
        void saveSwimmingPoolConfig(const SwimmingPoolConfig& config);

        void getSwimmingPoolSchedule(SwimmingPoolSchedule& schedule);
        void saveSwimmingPoolSchedule(const SwimmingPoolSchedule& schedule);

        // Irrigation
        void getIrrigationManualConfig(IrrigationManualConfig& config);
        void saveIrrigationManualConfig(IrrigationManualConfig& config);

        void getIrrigationScheduleConfig(IrrigationScheduleConfig& config);
        void saveIrrigationScheduleConfig(IrrigationScheduleConfig& config);

        void getGroups(IrrigationGroups& irrigationGroupsConfig);
        void saveIrrigationGroups(IrrigationGroups& irrigationGroupsConfig);

        void getGroup(const uint8_t groupIdx, IrrigationGroup& irrigationGroup);
        void saveIrrigationGroup(const uint8_t groupIdx, IrrigationGroup& irrigationGroup);

        void saveIrrigationGroupNextTimestamp(const uint8_t groupIdx, long nextTimestamp);
};

#endif
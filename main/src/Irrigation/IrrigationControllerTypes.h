
#ifndef IrrigationControllerTypes_h
#define IrrigationControllerTypes_h

#include "../ControllerConfig.h"
#include "ElectrovalvesControlThread.h"

#define IRRIGATION_GROUP_NAME_LENGTH 16

using IrrigationGroupName = char[IRRIGATION_GROUP_NAME_LENGTH];

struct IrrigationManualConfig {
    uint16_t zones;
    uint8_t  sourceIndex;
};

struct IrrigationScheduleConfig {
    bool     state;                       // Enable/disable the entire irrigation controller
    uint32_t disabledUntilTimestamp;      // Pause irrigation until this timestamp (UNIX timestamp)
    uint16_t maxScheduledTurnOnTimeout;   // Maximum allowed time to turn on a scheduled irrigation (e.g. after power loss)
    uint16_t minScheduledDuration;        // Minimum irrigation duration
    uint16_t maxScheduledDuration;        // Maximum irrigation duration
};

struct IrrigationGroup {
    bool enabled;               // Enabled state of the group

    IrrigationGroupName name;   // Group name

    uint16_t zones;             // Zones that are part of this group (stored as booleans in the number's bits)
    int8_t   source;            // Source index of the irrigation group

    uint8_t  period;            // Irrigation period in hours - Min 1 hour - max 7*24 hours
    uint16_t duration;          // Irrigation duration in seconds - Min 15 seconds - max 60*60 seconds

    uint16_t time;              // Irrigation time - minutes since 00:00
    uint32_t nextTimestamp;     // Next irrigation timestamp (UNIX timestamp)
};

using IrrigationGroups = IrrigationGroup[IRRIGATION_GROUPS_COUNT];

#endif
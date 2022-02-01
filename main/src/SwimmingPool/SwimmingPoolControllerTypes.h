
#ifndef SwimmingPoolControllerTypes_h
#define SwimmingPoolControllerTypes_h

struct SwimmingPoolConfig {
    uint16_t maxScheduledTurnOnTimeout;             // Maximum delay for triggering the swimming pool schedule (else skip schedule)
    uint16_t minScheduledDuration;                  // Minimum schedule duration (protect pump)
    uint16_t maxScheduledDuration;                  // Maximum schedule duration (prevent glitches)
    uint8_t  recirculationMaxTurnOnTimeout;         // Maximum timeout for detecting a recirculation flow after turning on the pump
    uint8_t  recirculationStopDetectionTimeout;     // Timeout for stopping the pump after failing to detect a recirculation flow
    uint8_t  uvTurnOnOffDelay;                      // Delay for turning on/off the UV-C disinfector
};

struct SwimmingPoolSchedule {
    bool     scheduleEnable;    // Schedule enable
    uint32_t nextTurnOnTime;    // Next turn on timestamp (UNIX timestamp)
    uint16_t duration;          // Filtration duration - in minutes
    uint8_t  periodDays;        // Period between filtrations - in days
};

#endif
#ifndef ProtocolDefinition_h
#define ProtocolDefinition_h
/*
    Define the instruction addresses for the serial communication between the
    API server and the PLC.
*/


// Global
#define GET_PLC_LAST_CHANGE_ADDR   0x1
#define GET_AUTO_VALUE_ADDR        0x2

#define GET_CLOCK_ADDR             0x5
#define SET_CLOCK_ADDR             0x6



// Swimming Pool
#define SP_GET_LAST_CHANGE_ADDR          0x32
#define SP_GET_CONTROLLER_STATE_ADDR     0x33
#define SP_GET_PUMP_STATE_ADDR           0x34
#define SP_GET_UV_STATE_ADDR             0x35
#define SP_GET_PUMP_MANUAL_VALUE_ADDR    0x36
#define SP_GET_UV_ENABLE_VALUE_ADDR      0x37
#define SP_GET_FLOW_SENSOR_VALUE_ADDR    0x38

#define SP_GET_PUMP_MANUAL_DISABLE_ADDR  0x39

#define SP_GET_SCHEDULE_ENABLE_ADDR      0x3A
#define SP_SET_SCHEDULE_ENABLE_ADDR      0x3B

#define SP_GET_SCHEDULE_NEXT_ADDR        0x3C
#define SP_SET_SCHEDULE_NEXT_ADDR        0x3D

#define SP_GET_SCHEDULE_DURATION_ADDR    0x3E
#define SP_SET_SCHEDULE_DURATION_ADDR    0x3F

#define SP_GET_SCHEDULE_PERIOD_ADDR      0x40
#define SP_SET_SCHEDULE_PERIOD_ADDR      0x41

#define SP_REQ_SCHEDULE_RESET_ADDR       0x42



// Irrigation
#define IRR_GET_LAST_CHANGE_ADDR                0x64 
#define IRR_GET_CONTROLLER_STATE_ADDR           0x65 
#define IRR_GET_PUMP_STATE_ADDR                 0x66
#define IRR_GET_MAINS_INLET_STATE_ADDR          0x67
#define IRR_GET_MANUAL_VALUE_ADDR               0x68
#define IRR_GET_PRESSURE_SENSOR_VALUE_ADDR      0x69

#define IRR_GET_MANUAL_DISABLE_STATE_ADDR       0x6A

#define IRR_GET_ZONES_STATE_ADDR                0x6B 

#define IRR_GET_MANUAL_ZONES_ADDR               0x6C 
#define IRR_SET_MANUAL_ZONES_ADDR               0x6D 

#define IRR_GET_MANUAL_SOURCE_ADDR              0x6E 
#define IRR_SET_MANUAL_SOURCE_ADDR              0x6F 

#define IRR_GET_SCHEDULE_ENABLE_ADDR            0x70
#define IRR_SET_SCHEDULE_ENABLE_ADDR            0x71

#define IRR_GET_SCHEDULE_PAUSED_STATE_ADDR      0x72
#define IRR_SET_SCHEDULE_PAUSE_TIMESTAMP_ADDR   0x73 

#define IRR_REQ_SCHEDULE_RESUME_ADDR            0x74
#define IRR_GET_SCHEDULE_RESUME_TIME_ADDR       0x75

#define IRR_GET_NEXT_IRRIGATION_TIME_ADDR       0x76

#define IRR_GET_SCHEDULE_GROUPS_STATE_ADDR      0x77 
#define IRR_GET_SCHEDULE_GROUP_STATE_ADDR       0x78
#define IRR_SET_SCHEDULE_GROUP_STATE_ADDR       0x79

#define IRR_GET_SCHEDULE_GROUP_NAME_ADDR        0x7A 
#define IRR_SET_SCHEDULE_GROUP_NAME_ADDR        0x7B 

#define IRR_GET_SCHEDULE_GROUP_ZONES_ADDR       0x7C 
#define IRR_SET_SCHEDULE_GROUP_ZONES_ADDR       0x7D 

#define IRR_GET_SCHEDULE_GROUP_SOURCE_ADDR      0x7E 
#define IRR_SET_SCHEDULE_GROUP_SOURCE_ADDR      0x7F 

#define IRR_GET_SCHEDULE_GROUP_PERIOD_ADDR      0x80 
#define IRR_SET_SCHEDULE_GROUP_PERIOD_ADDR      0x81 

#define IRR_GET_SCHEDULE_GROUP_DURATION_ADDR    0x82 
#define IRR_SET_SCHEDULE_GROUP_DURATION_ADDR    0x83 

#define IRR_GET_SCHEDULE_GROUP_INIT_TIME_ADDR   0x84 
#define IRR_SET_SCHEDULE_GROUP_INIT_TIME_ADDR   0x85 

#define IRR_GET_SCHEDULE_GROUP_NEXT_TIME_ADDR   0x86 

#define IRR_REQ_SCHEDULE_GROUP_NOW_ADDR         0x87 

#define IRR_REQ_CANCEL_CURRENT_JOB_ADDR         0x88 
#define IRR_REQ_CANCEL_ALL_JOBS_ADDR            0x89 

#define IRR_REQ_SCHEDULE_GROUP_RESET_ADDR       0x8A
#define IRR_REQ_SCHEDULE_RESET_ADDR             0x8B 


#endif
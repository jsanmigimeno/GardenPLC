#include "CommunicationsThread.h"

#include "ProtocolDefinition.h"
#include "../PinDefinitions.h"

static uint8_t payloadAndParity;

CommunicationsThread::CommunicationsThread(
  ElectrovalvesControlThread*& electrovavlesThread,
  TaskSchedulerThread<2>*&     taskSchedulerThread,
  IrrigationController*&       irrigationController,
  SwimmingPoolController*&     swimmingPoolController
) :
  electrovavlesThread(electrovavlesThread),
  taskSchedulerThread(taskSchedulerThread),
  irrigationController(irrigationController),
  swimmingPoolController(swimmingPoolController),
  max485(new MAX485(COMM_SERIAL, COMM_TRANSMISSION_ENABLE_PIN, 19200, SERIAL_8N1, 50, 50))
{
    max485->begin();
}



void CommunicationsThread::run() {

  // If no request is being actively handled, check for new requests
  if (requestCode == 0) {
    if (max485->available() > 0) { 
      // If a new request is received, wait for the first 2 bytes (instruction code + payload/parity)
      if (requestTimestamp == 0) requestTimestamp = millis();
      else if ((millis() - requestTimestamp) > TIMEOUT_PER_PACKET) { // Timed out receiveing the instruction payload+parity bit
        requestCode      = 0;
        requestTimestamp = 0;
        while (max485->available()) max485->read(); // Clear receive buffer
      }

      // Once the instruction, payload size and parity check have been received, read the data
      if (max485->available() >= 2) {
        requestCode        = max485->read();
        payloadAndParity   = max485->read();

        requestParityBit   = (payloadAndParity & 0x80) != 0;                      // Get the first bit (parity bit)
        requestPayloadSize = payloadAndParity & 0x7F;                             // Ignore the first bit (parity bit)
        requestDataTimeout = ((uint32_t) requestPayloadSize)*TIMEOUT_PER_PACKET;  // Calculate the timeout for the entire request
      }
    }
  }
  else { // If a request is being actively handled, wait for the request payload
    bool allPacketsReceived = max485->available() >= requestPayloadSize + 1; // Expect an extra null character at the end of the response
    bool timedOut           = !allPacketsReceived && ((millis() - requestTimestamp) >= requestDataTimeout);

    if (allPacketsReceived) {
      // Write received packets to the rxPayloadBuffer     //TODO make sure requestPayloadSize is no larger than the buffer size
      for (uint8_t i = 0; i < requestPayloadSize; i++) {
        rxPayloadBuffer[i] = max485->read();
      }

      // Expect extra null character at the end of the payload
      if (max485->read() != 0) {
        //TODO ERROR?
      }
      
      // Check parity - If the request parity (checkRequestParity()) is even (true), the parity check bit (requestParityBit) should be 0 (false)
      if (!checkRequestParity() == requestParityBit) handleRequest(requestCode);
    }

    if (timedOut) {
      //TODO THROW ERROR? NOTE ERROR SOMEWHERE?

      // Clear received data
      while (max485->available() > 0) max485->read();
    }

    if (allPacketsReceived || timedOut) {
      // Reset the variables state after request complition/timeout
      requestTimestamp    = 0;
      requestCode         = 0;
      requestParityBit  = false;
      requestPayloadSize  = 0;
      requestDataTimeout  = 0;
      responsePayloadSize = 0;
    }}

  runned();
}



// Request handling functions ***************************************************************************************************

void CommunicationsThread::handleRequest(uint8_t requestCode) {
  uint8_t groupIdx;
  IrrigationGroupName tempGroupNameBuff;

  // Reset the read pointers to the Rx/Tx buffers
  rxPayloadBufferNextPtr = rxPayloadBuffer;
  txPayloadBufferNextPtr = txPayloadBuffer;

  //TODO whatif groupIdx > number of groups?

  switch(requestCode) {

    // Global Instructions
    case GET_PLC_LAST_CHANGE_ADDR:
      writeResponsePayload(taskSchedulerThread->getLastChangeTimestamp());
      break;
    case GET_AUTO_VALUE_ADDR: //Get auto mode enable state
      writeResponsePayload(taskSchedulerThread->getAutoModeState());
      break;
    case 0x3:
      break;
    case 0x4:
      break;
    case GET_CLOCK_ADDR: //Get clock
      writeResponsePayload(taskSchedulerThread->getTime());
      break;
    case SET_CLOCK_ADDR: //Set clock
      taskSchedulerThread->setTime(readRequestPayloadInt(4));

      // Cancel all active jobs after clock change, as the finish timestamps will be corrupted
      swimmingPoolController->stopJob();
      electrovavlesThread->cancelAllJobs();
      break;


    // Swimming Pool Instructions
    case SP_GET_LAST_CHANGE_ADDR: // Swimming Pool Controller last change timestamp
      writeResponsePayload(swimmingPoolController->getLastChangeTimestamp());
      break;
    case SP_GET_CONTROLLER_STATE_ADDR: //Swimming Pool Controller State
      writeResponsePayload(swimmingPoolController->getControllerState());
	    break;
    case SP_GET_PUMP_STATE_ADDR: //Recirculation pump state
      writeResponsePayload(swimmingPoolController->swimmingPoolRecirculationPump->getState());
      break;
    case SP_GET_UV_STATE_ADDR: //UV state
      writeResponsePayload(swimmingPoolController->uvDisinfectLight->getState());
      break;
    case SP_GET_PUMP_MANUAL_VALUE_ADDR: //Recirculation pump manual override input value
      writeResponsePayload(swimmingPoolController->manualOverride->value());
      break;
    case SP_GET_UV_ENABLE_VALUE_ADDR: //UV enable input value
      writeResponsePayload(swimmingPoolController->UVEnable->value());
      break;
    case SP_GET_FLOW_SENSOR_VALUE_ADDR: //Recirculation flow sensor input value
      writeResponsePayload(swimmingPoolController->recirculationSensor->value());
      break;
    case SP_GET_PUMP_MANUAL_DISABLE_ADDR: //Recirculation pump manual override disable state
      writeResponsePayload(swimmingPoolController->getRecirculationPumpManualOverrideLockState());
      break;
    case SP_GET_SCHEDULE_ENABLE_ADDR: //Get schedule enable state
      writeResponsePayload(swimmingPoolController->isScheduleEnabled());
      break;
    case SP_SET_SCHEDULE_ENABLE_ADDR: //Set schedule enable state
      if (readRequestPayloadInt(1) == 0) swimmingPoolController->disableSchedule();
      else                        swimmingPoolController->enableSchedule();
      break;
    case SP_GET_SCHEDULE_NEXT_ADDR: //Get next scheduled turn on time
      writeResponsePayload(swimmingPoolController->getNextTurnOnTime());
      break;
    case SP_SET_SCHEDULE_NEXT_ADDR: //Set next scheduled turn on time
      swimmingPoolController->setNextTurnOnTime(readRequestPayloadInt(4));
      break;
    case SP_GET_SCHEDULE_DURATION_ADDR: //Get scheduled recirculation duration
      writeResponsePayload(swimmingPoolController->getDuration());
      break;
    case SP_SET_SCHEDULE_DURATION_ADDR: //Set scheduled recirculation duration
      swimmingPoolController->setDuration(readRequestPayloadInt(2));
      break;
    case SP_GET_SCHEDULE_PERIOD_ADDR: //Get scheduled period
      writeResponsePayload(swimmingPoolController->getPeriodDays());
      break;
    case SP_SET_SCHEDULE_PERIOD_ADDR: //Set scheduled period
      swimmingPoolController->setPeriodDays(readRequestPayloadInt(1));
      break;
    case SP_REQ_SCHEDULE_RESET_ADDR: //Reset
      if (readRequestPayloadInt(2) == 0xAA00) swimmingPoolController->reset();
      break;
    

    // Irrigation Instructions
    case IRR_GET_LAST_CHANGE_ADDR:
      writeResponsePayload(irrigationController->getLastChangeTimestamp());
      break;
    case IRR_GET_CONTROLLER_STATE_ADDR:
      writeResponsePayload(irrigationController->getControllerState());
      break;
    case IRR_GET_PUMP_STATE_ADDR: //Irrigation pump state
      writeResponsePayload(electrovavlesThread->swimmingPoolIrrigationPump->getState());
      break;
    case IRR_GET_MAINS_INLET_STATE_ADDR: //Mains water inlet state
      writeResponsePayload(electrovavlesThread->mainsWaterInletValve->getState());
      break;
    case IRR_GET_MANUAL_VALUE_ADDR: //Irrigation from swimming pool manual override input value
      writeResponsePayload(irrigationController->manualIrrigationEnable->value());
      break;
    case IRR_GET_PRESSURE_SENSOR_VALUE_ADDR: //Irrigation pressure sensor input value
      writeResponsePayload(irrigationController->irrigationPressureSensor->value());
      break;
    case IRR_GET_MANUAL_DISABLE_STATE_ADDR: //Manual irrigation disable state
      writeResponsePayload(irrigationController->getManualOverrideLockState());
      break;
    case IRR_GET_ZONES_STATE_ADDR: //Zones state
      writeResponsePayload(irrigationController->getZonesState());
      break;
    case IRR_GET_MANUAL_ZONES_ADDR: //Get manual irrigation zones
      writeResponsePayload(irrigationController->getIrrigationManualZones());
      break;
    case IRR_SET_MANUAL_ZONES_ADDR: //Set manual irrigation zones
      irrigationController->setIrrigationManualZones(readRequestPayloadInt(2));
      break;
    case IRR_GET_MANUAL_SOURCE_ADDR: //Get manual irrigation source
      writeResponsePayload(irrigationController->getIrrigationManualSource());
      break;
    case IRR_SET_MANUAL_SOURCE_ADDR: //Set manual irrigation source
      irrigationController->setIrrigationManualSource(readRequestPayloadInt(1));
      break;
    case IRR_GET_SCHEDULE_ENABLE_ADDR: //Get irrigation enable state
      writeResponsePayload(irrigationController->isScheduleEnabled());
      break;
    case IRR_SET_SCHEDULE_ENABLE_ADDR: //Set irrigation enable state
      if (readRequestPayloadInt(1) == 0) irrigationController->disableSchedule();
      else                        irrigationController->enableSchedule();
      break;
    case IRR_GET_SCHEDULE_PAUSED_STATE_ADDR: //Get irrigation paused state
      writeResponsePayload(irrigationController->isSchedulePaused());
      break;
    case IRR_SET_SCHEDULE_PAUSE_TIMESTAMP_ADDR: //Pause irrigation
      irrigationController->pauseSchedule(readRequestPayloadInt(4));
      break;
    case IRR_REQ_SCHEDULE_RESUME_ADDR: //Resume irrigation
      if (readRequestPayloadInt(1) != 0) irrigationController->resumeSchedule();
      break;
    case IRR_GET_SCHEDULE_RESUME_TIME_ADDR: //Get irrigation scheduled resume time
      writeResponsePayload(irrigationController->getScheduleResumeTime());
      break;
    case IRR_GET_NEXT_IRRIGATION_TIME_ADDR: //Get next irrigation time
      writeResponsePayload(irrigationController->getNextIrrigationTime());
      break;
    case IRR_GET_SCHEDULE_GROUPS_STATE_ADDR: //Get irrigation groups enable state
      writeResponsePayload(irrigationController->getGroupsEnableState());
      break;
    case IRR_GET_SCHEDULE_GROUP_STATE_ADDR: //Get irrigation group enable state
      writeResponsePayload(irrigationController->isGroupEnabled(readRequestPayloadInt(1)));
      break;
    case IRR_SET_SCHEDULE_GROUP_STATE_ADDR: //Set irrigation group enable state
      groupIdx = readRequestPayloadInt(1);
      if (readRequestPayloadInt(1) == 0) irrigationController->disableGroup(groupIdx);
      else                        irrigationController->enableGroup(groupIdx);
      break;
    case IRR_GET_SCHEDULE_GROUP_NAME_ADDR: //Get irrigation group name
      irrigationController->getGroupName(readRequestPayloadInt(1), tempGroupNameBuff);
      writeResponsePayload((uint8_t*) &tempGroupNameBuff, IRRIGATION_GROUP_NAME_LENGTH);
      break;
    case IRR_SET_SCHEDULE_GROUP_NAME_ADDR: //Set irrigation group name
      groupIdx = readRequestPayloadInt(1);
      readRequestPayload((uint8_t*) &tempGroupNameBuff, IRRIGATION_GROUP_NAME_LENGTH);
      irrigationController->setGroupName(groupIdx, tempGroupNameBuff);
      break;
    case IRR_GET_SCHEDULE_GROUP_ZONES_ADDR: //Get irrigation group zones
      writeResponsePayload(irrigationController->getGroupZones(readRequestPayloadInt(1)));
      break;
    case IRR_SET_SCHEDULE_GROUP_ZONES_ADDR: //Set irrigation group zones
      groupIdx = readRequestPayloadInt(1);
      irrigationController->setGroupZones(groupIdx, readRequestPayloadInt(2));
      break;
    case IRR_GET_SCHEDULE_GROUP_SOURCE_ADDR: //Get irrigation group source
      writeResponsePayload(irrigationController->getGroupSource(readRequestPayloadInt(1)));
      break;
    case IRR_SET_SCHEDULE_GROUP_SOURCE_ADDR: //Set irrigation group source
      groupIdx = readRequestPayloadInt(1);
      irrigationController->setGroupSource(groupIdx, readRequestPayloadInt(1));
      break;
    case IRR_GET_SCHEDULE_GROUP_PERIOD_ADDR: //Get irrigation group period
      writeResponsePayload(irrigationController->getGroupPeriod(readRequestPayloadInt(1)));
      break;
    case IRR_SET_SCHEDULE_GROUP_PERIOD_ADDR: //Set irrigation group period
      groupIdx = readRequestPayloadInt(1);
      irrigationController->setGroupPeriod(groupIdx, readRequestPayloadInt(1));
      break;
    case IRR_GET_SCHEDULE_GROUP_DURATION_ADDR: //Get irrigation group duration
      writeResponsePayload(irrigationController->getGroupDuration(readRequestPayloadInt(1)));
      break;
    case IRR_SET_SCHEDULE_GROUP_DURATION_ADDR: //Set irrigation group duration
      groupIdx = readRequestPayloadInt(1);
      irrigationController->setGroupDuration(groupIdx, readRequestPayloadInt(2));
      break;
    case IRR_GET_SCHEDULE_GROUP_INIT_TIME_ADDR: //Get irrigation group init time
      writeResponsePayload(irrigationController->getGroupInitTime(readRequestPayloadInt(1)));
      break;
    case IRR_SET_SCHEDULE_GROUP_INIT_TIME_ADDR: //Set irrigation group init time
      groupIdx = readRequestPayloadInt(1);
      irrigationController->setGroupInitTime(groupIdx, readRequestPayloadInt(2));
      break;
    case IRR_GET_SCHEDULE_GROUP_NEXT_TIME_ADDR: //Get irrigation group next init time
      writeResponsePayload(irrigationController->getGroupNextIrrigationTime(readRequestPayloadInt(1)));
      break;
    case IRR_REQ_SCHEDULE_GROUP_NOW_ADDR:
      irrigationController->scheduleGroupNow(readRequestPayloadInt(1));
      break;
    case IRR_REQ_CANCEL_CURRENT_JOB_ADDR:
      if (readRequestPayloadInt(1) != 0) electrovavlesThread->cancelCurrentJob();
      break;
    case IRR_REQ_CANCEL_ALL_JOBS_ADDR:
      if (readRequestPayloadInt(1) != 0) electrovavlesThread->cancelAllJobs();
      break;
    case IRR_REQ_SCHEDULE_GROUP_RESET_ADDR: //Reset irrigation group
      groupIdx = readRequestPayloadInt(1);
      if (readRequestPayloadInt(2) == 0xBB01) irrigationController->resetGroup(groupIdx);
      break;
    case IRR_REQ_SCHEDULE_RESET_ADDR: //Reset
      if (readRequestPayloadInt(2) == 0xBA00) irrigationController->reset();
      break;

    default:
      // Unknown instruction. Do not respond
      return;

  }

  sendResponse();

}



void CommunicationsThread::sendResponse() {
  //TODO make sure responsePayloadSize is no larger than the buffer size

  txPayloadBufferNextPtr = txPayloadBuffer; // Reset the Tx buffer read pointer to the start of the buffer

  max485->beginTransmission();

  max485->write(requestCode);
  max485->write(responsePayloadSize | ((checkResponseParity() ? 0 : 1) << 7)); // Parity bit - make the number of 1s in the response even 

  for (int8_t i = 0; i < responsePayloadSize; i++) {
    max485->write(*(txPayloadBufferNextPtr++));
  }

  // Serial.available() treats 0xFF as EOL and will skip it if it's the last byte on the buffer.
  // Always transmit 0x0 at the end of the response as a workaround
  max485->write(0x0);

  max485->endTransmission();
}



// Rx/Tx payload buffer read/write functions ************************************************************************************

// Read up to 4 bytes from the Rx buffer and cast it to an int
uint32_t CommunicationsThread::readRequestPayloadInt(uint8_t bytesCount) {
  uint32_t response = 0;

  uint8_t* responsePtr = (uint8_t*) &response;
  uint8_t packet;

  for (int8_t i = 0; i < bytesCount; i++) {
    responsePtr[i] = *(rxPayloadBufferNextPtr++);
  }

  return response;
}

// Copy data from the Rx buffer to the supplied buffer
void CommunicationsThread::readRequestPayload(uint8_t* bufferPtr, uint8_t bytesCount) {
  for (int8_t i = 0; i < bytesCount; i++) {
    bufferPtr[i] = *(rxPayloadBufferNextPtr++);
  }
}

// Write data to Tx buffer
void CommunicationsThread::writeResponsePayload(bool response) {
  writeResponsePayload((uint8_t) (response ? 1 : 0));
}

void CommunicationsThread::writeResponsePayload(uint8_t response) {
  writeResponsePayload((uint8_t*) &response, 1);
}

void CommunicationsThread::writeResponsePayload(uint16_t response) {
  writeResponsePayload((uint8_t*) &response, 2);
}

void CommunicationsThread::writeResponsePayload(uint32_t response) {
  writeResponsePayload((uint8_t*) &response, 4);
}

void CommunicationsThread::writeResponsePayload(uint8_t* responsePtr, uint8_t bytesCount) {
  responsePayloadSize += bytesCount;
  for (int8_t i = 0; i < bytesCount; i++) {
    *txPayloadBufferNextPtr++ = responsePtr[i];
  }
}



// Parity check functions *******************************************************************************************************

bool CommunicationsThread::checkRequestParity() {
  bool parity = checkParity(&requestCode);
  parity = parity == checkParity(&requestPayloadSize);
  parity = parity == checkParity(rxPayloadBuffer, requestPayloadSize);

  return parity;
}

bool CommunicationsThread::checkResponseParity() {
  bool parity = checkParity(&requestCode);
  parity = parity == checkParity(&responsePayloadSize);
  parity = parity == checkParity(txPayloadBuffer, responsePayloadSize);

  return parity;
}

bool CommunicationsThread::checkParity(uint8_t* bytePtr) {
  uint8_t t = *bytePtr ^ (*bytePtr >> 1);
  t ^= t >> 2;
  t ^= t >> 4;
  return (~t) & 1; // 'True' if the number of 1s is even
}

bool CommunicationsThread::checkParity(uint8_t* startPtr, uint8_t size) {
  bool parity = true;
  for (uint8_t i = 0; i < size; i++) {
    parity = parity == checkParity(startPtr + i);
  }
  return parity; // 'True' if the number of 1s is even
}
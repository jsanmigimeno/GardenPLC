/*
  CommunicationsThread.h

  Handles communication between the controller and the GardenPLCWirelessInterface using serial communication
  via the MAX485 component.

  A custom communication protocol is implemented, each transmission is formed as:
    1 Byte  - Instruction Code
    2 Bytes - Parity Bit + Payload Size (First bit (MSB) is the parity bit)
    N Bytes - Payload
    NULL character
  
  When a request is received, once the parity check is successful, the request payload is written to the
  Rx payload buffer. 
  The 'readRequestPayload' functions are then used to read data sequentially from this buffer; each call
  increments the 'rxPayloadBufferNextPtr'.
  Response data is written to the Tx payload buffer in a similar manner, using the 'writeResponsePayload' 
  functions and the 'txPayloadBufferNextPtr'.
  Last, the response is sent using the protocol defined above. A response is always sent, even if there 
  is no response payload.
*/

#ifndef CommunicationsThread_h
#define CommunicationsThread_h

#include <Arduino.h>
#include <Thread.h>
#include <MAX485.h>

#include "../ControllerConfig.h"
#include "../TaskScheduler/TaskSchedulerThread.h"
#include "../Irrigation/IrrigationController.h"
#include "../SwimmingPool/SwimmingPoolController.h"

const uint8_t payloadBufferSize = IRRIGATION_GROUP_NAME_LENGTH + 1; // Set to the largest possible request payload

class CommunicationsThread: public Thread
{
  public:
    CommunicationsThread(
      ElectrovalvesControlThread*& electrovavlesThread,
      TaskSchedulerThread<2>*&     taskSchedulerThread,
      IrrigationController*&       irrigationController,
      SwimmingPoolController*&     swimmingPoolController
    );
    void run();
  
  private:
    ElectrovalvesControlThread*& electrovavlesThread;
    TaskSchedulerThread<2>*&     taskSchedulerThread;
    IrrigationController*&       irrigationController;
    SwimmingPoolController*&     swimmingPoolController;

    MAX485* max485;

    uint8_t  rxPayloadBuffer[payloadBufferSize] = {0};
    uint8_t  txPayloadBuffer[payloadBufferSize] = {0};
    uint8_t* rxPayloadBufferNextPtr = rxPayloadBuffer;
    uint8_t* txPayloadBufferNextPtr = txPayloadBuffer;

    uint32_t requestTimestamp   = 0;
    uint8_t  requestCode        = 0;
    bool     requestParityBit = false;
    uint8_t  requestPayloadSize = 0;
    uint32_t requestDataTimeout = 0;

    uint8_t  responsePayloadSize = 0;

    void handleRequest(uint8_t requestCode);
    void sendResponse();

    uint32_t readRequestPayloadInt(uint8_t bytesCount);                  // Parse ${bytesCount} bytes of the rx payload buffer as an int
    void     readRequestPayload(uint8_t* bufferPtr, uint8_t bytesCount); // Copy ${bytesCount} bytes of the rx payload buffer to the supplied buffer (bufferPtr)
  
    void writeResponsePayload(bool response);
    void writeResponsePayload(uint8_t response);
    void writeResponsePayload(uint16_t response);
    void writeResponsePayload(uint32_t response);
    void writeResponsePayload(uint8_t* responsePtr, uint8_t bytesCount);

    bool checkParity(uint8_t* bytePtr);
    bool checkParity(uint8_t* startPtr, uint8_t size);

    bool checkRequestParity();
    bool checkResponseParity();
};

#endif
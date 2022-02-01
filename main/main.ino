#include <ThreadController.h>
#include <StaticThreadController.h>
#include <RTClib.h>

#include "src/ControllerConfig.h"
#include "src/Utils/DataSaver.h"
#include "src/Irrigation/IrrigationController.h"
#include "src/Irrigation/ElectrovalvesControlThread.h"
#include "src/SwimmingPool/SwimmingPoolController.h"
#include "src/TaskScheduler/TaskSchedulerThread.h"
#include "src/Communication/CommunicationsThread.h"

// Initialise thread controller
ThreadController threadController = ThreadController();

// Clock
RTC_DS3231 rtc;

void setup() {
  // NOTE: serial debugging messages CANNOT be enabled whilst using the serial interface to communicate
  // with the GardenPLCWirelessInterface. Workaround: use a board with extra Serial channels and set COMM_SERIAL to 
  // a channel other than the default one (e.g. Serial1).

  // Wait for rtc to be ready
  while (!rtc.begin()) {
    delay(50);
  }

  // Shared objects
  DataSaver* dataSaver = new DataSaver();  //TODO VALIDATE LOADED DATA

  ElectrovalvesControlThread* electrovavlesThread = new ElectrovalvesControlThread();

  // Initialise controllers and task scheduler
  IrrigationController*   irrigationController   = new IrrigationController(electrovavlesThread, dataSaver, rtc);
  SwimmingPoolController* swimmingPoolController = new SwimmingPoolController(dataSaver);

  const Task* tasks[2] = {irrigationController, swimmingPoolController};
  TaskSchedulerThread<2>* taskSchedulerThread = new TaskSchedulerThread<2>(tasks, rtc);

  // Communications Thread
  CommunicationsThread* communicationsThread = new CommunicationsThread(
    electrovavlesThread,
    taskSchedulerThread,
    irrigationController,
    swimmingPoolController
  );

  // Initialise memory if not initialised
  if (!(dataSaver->isInitialised())) {
    irrigationController->reset();
    swimmingPoolController->reset();

    dataSaver->setInitialisedFlag();
  }

  // Start threads and set intervals
	threadController.add(electrovavlesThread);
	threadController.add(taskSchedulerThread);
	threadController.add(communicationsThread);

  electrovavlesThread->setInterval(1);
  taskSchedulerThread->setInterval(1);
  communicationsThread->setInterval(1);

  delay(1000); // Wait for pinmodes to establish
}


void loop() {
  // Run threads
  threadController.run();
}

#include <Arduino.h>
#include "KuDAQ.h"

// freeRTOS task handles
TaskHandle_t Task1;
TaskHandle_t Task2;

KuDAQ DAQ; // global instance of KuDAQ

void taskCore0(void * parameter) { // SENSOR READING THREAD
  for(;;) {
    DAQ.updateCore0(); // loop function for core 0
  }
}

void taskCore1(void * parameter) { // EXTERNAL COMMUNICATION THREAD
  for(;;) {
    DAQ.updateCore1(); // loop function for core 1
  }
}

void setup() {
  DAQ.initialize_all(); // Initialize all components of the system
  xTaskCreatePinnedToCore(taskCore0, "Task0", 10000, NULL, 1, &Task1, 1); // Core 1
  xTaskCreatePinnedToCore(taskCore1, "Task1", 10000, NULL, 1, &Task2, 0); // Core 0
}

void loop() {
  // nothing to see here, get out !
}
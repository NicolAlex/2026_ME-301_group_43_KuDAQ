#include <Arduino.h>
#include "KuDAQ.h"

// mutlicore
TaskHandle_t Task1;
TaskHandle_t Task2;

void taskCore0(void * parameter) { // SENSOR READING THREAD
  for(;;) {
    // variable definiton
    static orientation_buffer_t orientation1_sender;
    // data acquisition and processing
    sensor1.aquisition();
    orientation1_sender = sensor1.getRawOrientation();
    // send only if there's new data
    if (orientation1_sender.newData) {
      // For now, send orientation data over the queue (instead of accel) for testing
      xQueueOverwrite(orientationQueue, &orientation1_sender);
      orientation1_sender.newData = false; // Mark data as sent

      // compute and send aquisition frequency
      static unsigned long last_acquisition_ms = 0;
      static int acquisition_counter = 0;
      acquisition_counter++;
      if (millis() - last_acquisition_ms >= 1000) {
          last_acquisition_ms = millis();
          Serial.print("Acquisition frequency: ");
          Serial.println(acquisition_counter);
          acquisition_counter = 0;
      }

    }
    //vTaskDelay(1 / portTICK_PERIOD_MS);
    
  }
}

void taskCore1(void * parameter) { // EXTERNAL COMMUNICATION THREAD
  for(;;) {
    // variable definition
    static orientation_buffer_t orientation1_receiver;
    static unsigned long last_wifi_send_ms = 0;
    static int wifi_send_counter = 0;
    // WiFi client handling
    if (!client || !client.connected()) {
      client = server.available();
    }
    // wait up to 100 ms for data
    if (xQueueReceive(orientationQueue, &orientation1_receiver, pdMS_TO_TICKS(100)) == pdTRUE) {

        if (client && client.connected()) {
            client.print("PITCH:");
            client.println(orientation1_receiver.rpy_buffer[0], 3);
            client.print("ROLL:");
            client.println(orientation1_receiver.rpy_buffer[1], 3);
            client.print("YAW:");
            client.println(orientation1_receiver.rpy_buffer[2], 3);
            //Serial.println("send data over wifi !");

            // sending frequency counter (for debug)
            wifi_send_counter++;
            unsigned long current_ms = millis();
            if (current_ms - last_wifi_send_ms >= 1000) {
                last_wifi_send_ms = current_ms;
                client.print("SENDING_FREQUENCY:");
                client.println(wifi_send_counter);
                wifi_send_counter = 0;
            }
          }

      #ifdef SERIAL_STREAM
      // Optional USB debug output.
      Serial.print("Pitch : ");
      Serial.println(orientation1_receiver.rpy_buffer[0], 3);
      Serial.print("Roll : ");
      Serial.println(orientation1_receiver.rpy_buffer[1], 3);
      Serial.print("Yaw : ");
      Serial.println(orientation1_receiver.rpy_buffer[2], 3);
      Serial.println("---------------------------------------------------------------------------------");
      #endif
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  

  // Keep sensor work away from core 0, where BT stack activity is concentrated.
  xTaskCreatePinnedToCore(taskCore0, "Task0", 10000, NULL, 1, &Task1, 1); // Core 1
  xTaskCreatePinnedToCore(taskCore1, "Task1", 10000, NULL, 1, &Task2, 0); // Core 0
}

void loop() {
  // nothing to see here, get out !
}
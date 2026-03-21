#include <Arduino.h>
#include "KuDAQ.h"
// FreeRTOS queue for safe inter-core data transfer
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// mutlicore
TaskHandle_t Task1;
TaskHandle_t Task2;

// sensor definition (create at runtime in setup to avoid construction before Arduino core inits)
Sensor sensor1(ACCEL1_ADDR, GYRO1_ADDR);
Sensor sensor2(ACCEL2_ADDR, GYRO2_ADDR);

// Queue used to pass accel samples safely from core 0 to core 1
QueueHandle_t accelQueue = nullptr;

void taskCore0(void * parameter) {
  for(;;) {
    // variable definiton
    static sensor_buffer_t accel1_sender;
    // data acquisition and processing
    sensor1.aquisition();
    accel1_sender = sensor1.getFilteredAccel();
    // send only if there's new data
    if (accel1_sender.newData) {
      // enqueue, do not block long (0 ticks)
      xQueueOverwrite(accelQueue, &accel1_sender);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void taskCore1(void * parameter) {
  for(;;) {
    // variable definition
    static sensor_buffer_t accel1_receiver;
    // wait up to 100 ms for data
    if (xQueueReceive(accelQueue, &accel1_receiver, pdMS_TO_TICKS(0)) == pdTRUE) {
      //Serial.println("--------------------------------------------------------------");
      //Serial.println("New data available:");
      //Serial.println("Sensor 1 - Accel: ");
      Serial.print(">accel_x:");
      Serial.println(accel1_receiver.xyz_buffer[0]);
      Serial.print(">accel_y:");
      Serial.println(accel1_receiver.xyz_buffer[1]);
      Serial.print(">accel_z:");
      Serial.println(accel1_receiver.xyz_buffer[2]);
      // Serial.print("Timestamp: ");
      // Serial.println(accel1_receiver.timestamp);

      // compute acquisition frequency for debug
      static unsigned long last_timestamp = 0;
      static int sample_count = 0;
      static float acquisition_freq = 0.0f;
      sample_count++;
      unsigned long current_time = micros();
      if (current_time - last_timestamp >= 1000000) { // every 1 second
        acquisition_freq = sample_count;
        last_timestamp = current_time;
        sample_count = 0;
      }
      Serial.print(">freq:");
      Serial.println(acquisition_freq);
    } else {
#ifdef DEBUG
      Serial.println("No new data available.");
#endif
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);

    
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // wait for serial monitor to initialize

  // initialize I2C before creating sensor objects
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // initialize sensors
  sensor1.init();
  sensor2.init();

  delay(100); // wait for sensors to initialize

  // create queue to pass 1 sensor sample bwteen cores
  accelQueue = xQueueCreate(1, sizeof(sensor_buffer_t));
  if (accelQueue == NULL) {
    Serial.println("Failed to create accel queue!");
    while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  xTaskCreatePinnedToCore(taskCore0, "Task0", 10000, NULL, 1, &Task1, 0); // Core 0
  xTaskCreatePinnedToCore(taskCore1, "Task1", 10000, NULL, 1, &Task2, 1); // Core 1
}

void loop() {
  // nothing to see here, get out !
}

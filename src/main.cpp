#include <Arduino.h>
#include "KuDAQ.h"

// mutlicore
TaskHandle_t Task1;
TaskHandle_t Task2;

// sensor definition (create at runtime in setup to avoid construction before Arduino core inits)
Sensor sensor1(ACCEL1_ADDR, GYRO1_ADDR);
Sensor sensor2(ACCEL2_ADDR, GYRO2_ADDR);

// common structs
static sensor_buffer_t accel_data_1;
static sensor_buffer_t gyro_data_1;
static sensor_buffer_t accel_data_2;
static sensor_buffer_t gyro_data_2;
static volatile bool data_ready = false;

// void taskCore0(void * parameter) {
//   for(;;) {
//     sensor1.aquisition();
//     sensor2.aquisition();
//     data_ready = sensor1.newDataReady() || sensor2.newDataReady();
//     accel_data_1 = sensor1.getFilteredAccel();
//     gyro_data_1 = sensor1.getFilteredGyro();
//     accel_data_2 = sensor2.getFilteredAccel();
//     gyro_data_2 = sensor2.getFilteredGyro();
//     delay(1000);
//     vTaskDelay(1 / portTICK_PERIOD_MS);
//   }
// }

// void taskCore1(void * parameter) {
//   for(;;) {
//     if (data_ready) {
//       Serial.println("New data available:");
//       Serial.print("Sensor 1 - Accel: ");
//       Serial.print(accel_data_1.xyz_buffer[0]);
//       Serial.print(", ");
//       Serial.print(accel_data_1.xyz_buffer[1]);
//       Serial.print(", ");
//       Serial.print(accel_data_1.xyz_buffer[2]);
//       Serial.print(" | Gyro: ");
//       Serial.print(gyro_data_1.xyz_buffer[0]);
//       Serial.print(", ");
//       Serial.print(gyro_data_1.xyz_buffer[1]);
//       Serial.print(", ");
//       Serial.println(gyro_data_1.xyz_buffer[2]);

//       Serial.print("Sensor 2 - Accel: ");
//       Serial.print(accel_data_2.xyz_buffer[0]);
//       Serial.print(", ");
//       Serial.print(accel_data_2.xyz_buffer[1]);
//       Serial.print(", ");
//       Serial.print(accel_data_2.xyz_buffer[2]);
//       Serial.print(" | Gyro: ");
//       Serial.print(gyro_data_2.xyz_buffer[0]);
//       Serial.print(", ");
//       Serial.print(gyro_data_2.xyz_buffer[1]);
//       Serial.print(", ");
//       Serial.println(gyro_data_2.xyz_buffer[2]);

//       data_ready = false; // reset the flag
//     }
//     else {
//       Serial.println("No new data available.");
//     }
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//   }
// }

void setup() {
  Serial.begin(115200);
  delay(1000); // wait for serial monitor to initialize

  // initialize I2C before creating sensor objects
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // initialize sensors
  sensor1.init();
  sensor2.init();

  delay(1000); // wait for sensors to initialize


  // xTaskCreatePinnedToCore(
  //   taskCore0,     // Function
  //   "Task0",       // Name
  //   10000,         // Stack size
  //   NULL,          // Parameter
  //   1,             // Priority
  //   &Task1,        // Task handle
  //   0);            // Core 0

  // xTaskCreatePinnedToCore(
  //   taskCore1,
  //   "Task1",
  //   10000,
  //   NULL,
  //   1,
  //   &Task2,
  //   1);            // Core 1


}

void loop() {
    sensor1.aquisition();

    accel_data_1 = sensor1.getFilteredAccel();
    Serial.print("Sensor 1 - Accel: ");
    Serial.println(accel_data_1.xyz_buffer[0]);
    Serial.print(", ");
    Serial.print(accel_data_1.xyz_buffer[1]);
    Serial.print(", ");
    Serial.println(accel_data_1.xyz_buffer[2]);
    delay(100);
}


// accessing data of a std::vector:
// std::vector<float> myVector = {1.0f, 2.0f, 3.0f};
// float firstElement = myVector[0]; // Access the first element
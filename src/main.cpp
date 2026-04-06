#include <Arduino.h>
#include "KuDAQ.h"
// FreeRTOS queue for safe inter-core data transfer
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <WiFi.h>

// mutlicore
TaskHandle_t Task1;
TaskHandle_t Task2;

// sensor definition (create at runtime in setup to avoid construction before Arduino core inits)
Sensor sensor1(ACCEL1_ADDR, GYRO1_ADDR);
Sensor sensor2(ACCEL2_ADDR, GYRO2_ADDR);

// Queue used to pass accel samples safely from core 0 to core 1
QueueHandle_t accelQueue = nullptr;

// WiFi credentials
const char* ssid = "ESP32_NET"; // IP : 192.168.4.1
const char* password = "12345678";

// Wifi server creation
WiFiServer server(23);
WiFiClient client;


void taskCore0(void * parameter) { // SENSOR READING THREAD
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

void taskCore1(void * parameter) { // EXTERNAL COMMUNICATION THREAD
  for(;;) {
    // variable definition
    static sensor_buffer_t accel1_receiver;
    static unsigned long last_wifi_send_ms = 0;
    static int wifi_send_counter = 0;
    // WiFi client handling
    if (!client || !client.connected()) {
      client = server.available();
    }
    // wait up to 100 ms for data
    if (xQueueReceive(accelQueue, &accel1_receiver, pdMS_TO_TICKS(0)) == pdTRUE) {

        if (client && client.connected()) {
            client.print("TIMESTAMP:");
            client.println(accel1_receiver.timestamp); // microsecond timestamp
            client.print("ACCEL_X:");
            client.println(accel1_receiver.xyz_buffer[0], 3);
            client.print("ACCEL_Y:");
            client.println(accel1_receiver.xyz_buffer[1], 3);
            client.print("ACCEL_Z:");
            client.println(accel1_receiver.xyz_buffer[2], 3);
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
        else {
          Serial.println("no client");
        }

      #ifdef DEBUG
      // Optional USB debug output.
      Serial.print(">accel_x:");
      Serial.println(accel1_receiver.xyz_buffer[0], 3);
      Serial.print(">accel_y:");
      Serial.println(accel1_receiver.xyz_buffer[1], 3);
      Serial.print(">accel_z:");
      Serial.println(accel1_receiver.xyz_buffer[2], 3);
      #endif
    } else {
    #ifdef DEBUG
          Serial.println("No new data available.");
    #endif
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100); // wait for serial monitor to initialize

  // WiFi setup
  WiFi.softAP(ssid, password); // Start WiFi in Access Point mode with given SSID and password
  Serial.println("Access Point started!");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());  // usually 192.168.4.1
  server.begin(); // Start the TCP server on port 23
  delay(100); // wait for WiFi to initialize

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

  // Keep sensor work away from core 0, where BT stack activity is concentrated.
  xTaskCreatePinnedToCore(taskCore0, "Task0", 10000, NULL, 1, &Task1, 1); // Core 1
  xTaskCreatePinnedToCore(taskCore1, "Task1", 10000, NULL, 1, &Task2, 0); // Core 0
}

void loop() {
  // nothing to see here, get out !
}
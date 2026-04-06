#include <Arduino.h>
#include "KuDAQ.h"
// FreeRTOS queue for safe inter-core data transfer
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "BluetoothSerial.h"
#include "esp_spp_api.h"

// mutlicore
TaskHandle_t Task1;
TaskHandle_t Task2;

// sensor definition (create at runtime in setup to avoid construction before Arduino core inits)
Sensor sensor1(ACCEL1_ADDR, GYRO1_ADDR);
Sensor sensor2(ACCEL2_ADDR, GYRO2_ADDR);

// Queue used to pass accel samples safely from core 0 to core 1
QueueHandle_t accelQueue = nullptr;

// bluetooth serial
BluetoothSerial SerialBT;
volatile bool bt_client_connected = false;

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
    case ESP_SPP_SRV_OPEN_EVT:
      bt_client_connected = true;
      Serial.println("BT client connected");
      break;
    case ESP_SPP_CLOSE_EVT:
      bt_client_connected = false;
      Serial.println("BT client disconnected");
      break;
    default:
      break;
  }
}

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
    static unsigned long last_bt_send_ms = 0;
    // wait up to 100 ms for data
    if (xQueueReceive(accelQueue, &accel1_receiver, pdMS_TO_TICKS(0)) == pdTRUE) {
      // Stream BT at a controlled rate to avoid SPP buffer congestion.
      if (bt_client_connected && SerialBT.hasClient()) {
        const unsigned long now_ms = millis();
        if (now_ms - last_bt_send_ms >= 100) { // 10 Hz BT stream for stability
          last_bt_send_ms = now_ms;
          SerialBT.printf("%.4f,%.4f,%.4f,%lu\n",
                          accel1_receiver.xyz_buffer[0],
                          accel1_receiver.xyz_buffer[1],
                          accel1_receiver.xyz_buffer[2],
                          accel1_receiver.timestamp);
        }
      }

      #ifdef DEBUG
      // Optional USB debug output.
      Serial.print(">accel_x:");
      Serial.println(accel1_receiver.xyz_buffer[0]);
      Serial.print(">accel_y:");
      Serial.println(accel1_receiver.xyz_buffer[1]);
      Serial.print(">accel_z:");
      Serial.println(accel1_receiver.xyz_buffer[2]);
      #endif
    } else {
    #ifdef DEBUG
          Serial.println("No new data available.");
    #endif
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);

    
  }
}

// void setup() {
//   Serial.begin(115200);
//   delay(1000); // wait for serial monitor to initialize
//   SerialBT.register_callback(btCallback);
//   SerialBT.setPin("1234");
//   SerialBT.begin("KuDAQ_BT"); // Bluetooth device name
//   Serial.println("Bluetooth SPP started. Pair, then open the COM port on PC.");
//   // initialize I2C before creating sensor objects
//   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
//   // initialize sensors
//   sensor1.init();
//   sensor2.init();

//   delay(100); // wait for sensors to initialize

//   // create queue to pass 1 sensor sample bwteen cores
//   accelQueue = xQueueCreate(1, sizeof(sensor_buffer_t));
//   if (accelQueue == NULL) {
//     Serial.println("Failed to create accel queue!");
//     while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
//   }

//   // Keep sensor work away from core 0, where BT stack activity is concentrated.
//   xTaskCreatePinnedToCore(taskCore0, "Task0", 10000, NULL, 1, &Task1, 1); // Core 1
//   xTaskCreatePinnedToCore(taskCore1, "Task1", 10000, NULL, 1, &Task2, 0); // Core 0
// }

// void loop() {
//   // nothing to see here, get out !
// }


void setup() {
  Serial.begin(115200);
  delay(1000); // wait for serial monitor to initialize
  SerialBT.begin("KuDAQ_BT"); // Bluetooth device name
  Serial.println("Bluetooth SPP started. Pair, then open the COM port on PC.");

  delay(100); // wait for sensors to initialize
}

void loop() {
  SerialBT.println("Hello from loop!");
  delay(1000);
}
#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include <array>
#include <vector>
#include <string>
#include "constants.h"
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct sensor_data_t { // stores 10 samples of data for filtering purposes
    int index = 0; // index for circular buffer
    float x[10];
    float y[10];
    float z[10];
    unsigned long timestamp[10];

    int unacked_data;
} sensor_data_t;

typedef struct orientation_data_t { // stores 10 samples of orientation data for filtering purposes
    int index = 0; // index for circular buffer
    float roll[10];
    float pitch[10];
    float yaw[10];
    unsigned long timestamp[10];

    int unacked_data;
} orientation_data_t;

typedef struct sensor_buffer_t { // for data transmission of 1 sample
    std::array<float, 3> xyz_buffer = {20.0f, 20.0f, 20.0f}; // x,y,z data
    unsigned long timestamp = 0; // timestamp of the data
    bool newData = false; // flag to indicate if it's new data
} sensor_buffer_t;

typedef struct orientation_buffer_t { // for data transmission of 1 sample
    std::array<float, 3> rpy_buffer = {0.0f, 0.0f, 0.0f}; // roll, pitch, yaw data
    unsigned long timestamp = 0; // timestamp of the data
    bool newData = false; // flag to indicate if it's new data
} orientation_buffer_t;


class Sensor {
    public:
        Sensor(uint8_t accel_addr, uint8_t gyro_addr);
        ~Sensor();
        void init();
        void aquisition();
        sensor_buffer_t getFilteredAccel();
        sensor_buffer_t getFilteredGyro();
        orientation_buffer_t getRawOrientation();
        orientation_buffer_t getFilteredOrientation();
        float getSamplingRate() { return sampling_rate; }
        float getCutoffFreq() { return cutoff_freq; }
        void setCutoffFreq(float fc);
        void setSamplingRate(float fs);

    private:
        // Data buffers
        sensor_data_t raw_accel_data;
        sensor_data_t raw_gyro_data;
        sensor_data_t filtered_accel_data;
        sensor_data_t filtered_gyro_data;
        orientation_data_t raw_orientation_data;
        orientation_data_t filtered_orientation_data;
        // external class wrapper (owns accel+gyro)
        Bmi088 *bmi = nullptr;
        Madgwick *ahrs = nullptr;
        // I2C addresses
        uint8_t accel_addr;
        uint8_t gyro_addr;
        // error string vector
        std::vector<std::string> errorMsg;
        // functions and variables
        bool read_data();
        void filter_data_1stOdr();
        void filter_data_2ndOdr();
        int sensor_status;
        float firstOrder_LPF(float x, float x_prev);
        float secondOrder_LPF(float x, float x_prev1, float x_prev2);
        void AHRS_update();
        float sampling_rate; // in Hz
        float cutoff_freq; // in Hz
        float a; // Coefficient for the low-pass filter
        float b; // Coefficient for the low-pass filter
        inline void compute_a();
        inline void compute_b();
};
class KuDAQ {
    public:
        KuDAQ();
        ~KuDAQ();
        void initialize_all();
        void updateCore0();
        void updateCore1();

    private:
        // Private methods
        void WiFi_setup();
        void WiFi_wellness();
        void core0_setupInterrupt(bool trigger);
        void cmdHandler(std::vector<std::string> cmd_tokens);
        std::vector<std::string> readMessage();
        // Private members
        CORE0_FSM core0_state;
        CORE1_FSM core1_state;
        Sensor *sensor1 = nullptr;
        Sensor *sensor2 = nullptr;
        QueueHandle_t orientationQueue = nullptr;
        QueueHandle_t trigger_core0_setup = nullptr;
        // Private variables
        const char* ssid = "KuDAQ_stream"; // IP : 192.168.4.1
        const char* password = "12345678";



};
    
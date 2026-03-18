#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include <math.h>
#include <array>
#include "constants.h"

typedef struct sensor_data_t { // stores 10 samples of data for filtering purposes
    float x[10];
    float y[10];
    float z[10];
    unsigned long timestamp[10];
    int unacked_data;
} sensor_data_t;

typedef struct sensor_buffer_t { // for data transmission of 1 sample
    std::array<float, 3> xyz_buffer; // x,y,z data
    unsigned long timestamp_buffer;
    bool newData; // flag to indicate if it's new data
} sensor_buffer_t;


class Sensor {
    public:
        Sensor(int accel_addr, int gyro_addr);
        ~Sensor();
        void aquisition();
        inline bool newDataReady();
        sensor_buffer_t getFilteredAccel();
        sensor_buffer_t getFilteredGyro();
        void setCutoffFreq(float fc);
        void setSamplingRate(float fs);

    private:
        sensor_data_t raw_accel_data;
        sensor_data_t raw_gyro_data;
        sensor_data_t filtered_accel_data;
        sensor_data_t filtered_gyro_data;
        void read_data();
        void filter_data_1stOdr();
        void filter_data_2ndOdr();
        int sensor_status;
        float firstOrder_LPF(float x, float x_prev);
        float secondOrder_LPF(float x, float x_prev1, float x_prev2);
        float sampling_rate = 100.0f; // in Hz
        float cutoff_freq = 50.0f; // in Hz
        float a; // Coefficient for the low-pass filter
        float b; // Coefficient for the low-pass filter
        inline void compute_a();
        inline void compute_b();
    };

/*class KuDAQ {
    public:
        KuDAQ();
        ~KuDAQ();
        void update_loop();
        void read_sensors();
        void filter_data();
        void merge_data();

    private:



        

}*/
    
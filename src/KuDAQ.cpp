#include "KuDAQ.h"

Sensor::Sensor(int accel_addr, int gyro_addr) { // Constructor
    // Initialize variables
    sensor_status = 0;
    a = 0.0f;
    b = 0.0f;
    for (int i = 0; i < 10; i++) {
        raw_accel_data.x[i] = 0.0f;
        raw_accel_data.y[i] = 0.0f;
        raw_accel_data.z[i] = 0.0f;
        raw_gyro_data.x[i] = 0.0f;
        raw_gyro_data.y[i] = 0.0f;
        raw_gyro_data.z[i] = 0.0f;
        raw_accel_data.timestamp[i] = 0;
        raw_gyro_data.timestamp[i] = 0;
    }
    raw_accel_data.unacked_data = 0;
    raw_gyro_data.unacked_data = 0;
    for (int i = 0; i < 10; i++) {
        filtered_accel_data.x[i] = 0.0f;
        filtered_accel_data.y[i] = 0.0f;
        filtered_accel_data.z[i] = 0.0f;
        filtered_gyro_data.x[i] = 0.0f;
        filtered_gyro_data.y[i] = 0.0f;
        filtered_gyro_data.z[i] = 0.0f;
        filtered_accel_data.timestamp[i] = 0;
        filtered_gyro_data.timestamp[i] = 0;
    }
    filtered_accel_data.unacked_data = 0;
    filtered_gyro_data.unacked_data = 0;
    // Initialize I2C communication
    Bmi088Accel accel(Wire, accel_addr);
    Bmi088Gyro gyro(Wire, gyro_addr);
    // Initialize the sensor
    sensor_status = accel.begin();
    if (status < 0) {
        Serial.println("Failed to initialize accel sensor!");
        while (1) {}
    }
    status = gyro.begin();
    if (status < 0) {
        Serial.println("Failed to initialize gyro sensor!");
        while (1) {}
        while (1) {}
    }
    // Compute filter coefficients
    compute_a();
    compute_b();
}

Sensor::~Sensor() { // Destructor
    // Close I2C communication if needed
    accel.end();
    gyro.end();
}

void Sensor::aquisition() {
    read_data();
    filter_data_1stOdr();
    // filter_data_2ndOdr(); // optional, can be called instead of 1st order filtering
}

void Sensor::read_data() {
    // Read accel data :
    static unsigned long accel_timestamp = 0;
    static unsigned long gyro_timestamp = 0;
    accel.readSensor();
    accel_timestamp = micros(); // get the current timestamp in microseconds
    gyro.readSensor();
    gyro_timestamp = micros(); // get the current timestamp in microseconds
    // Move data in the buffer and add new data at the last position
    for (int i = 0; i < 9; i++) {
        raw_accel_data.x[i] = raw_accel_data.x[i + 1];
        raw_accel_data.y[i] = raw_accel_data.y[i + 1];
        raw_accel_data.z[i] = raw_accel_data.z[i + 1];
        raw_gyro_data.x[i] = raw_gyro_data.x[i + 1];
        raw_gyro_data.y[i] = raw_gyro_data.y[i + 1];
        raw_gyro_data.z[i] = raw_gyro_data.z[i + 1];
        raw_accel_data.timestamp[i] = raw_accel_data.timestamp[i + 1];
        raw_gyro_data.timestamp[i] = raw_gyro_data.timestamp[i + 1];
    }
    // Add new data at the last position
    raw_accel_data.x[9] = accel.getAccelX_mss();
    raw_accel_data.y[9] = accel.getAccelY_mss();
    raw_accel_data.z[9] = accel.getAccelZ_mss();
    raw_gyro_data.x[9] = gyro.getGyroX_rads();
    raw_gyro_data.y[9] = gyro.getGyroY_rads();
    raw_gyro_data.z[9] = gyro.getGyroZ_rads();
    raw_accel_data.timestamp[9] = accel_timestamp;
    raw_gyro_data.timestamp[9] = gyro_timestamp;
    // increment the unacknowledged new data index
    raw_accel_data.unacked_data++;
    raw_gyro_data.unacked_data++;

}

float Sensor::firstOrder_LPF(float x, float x_prev) {
    // First order low pass filter implementation
    float y = b * x + a * x_prev;
    return y;
}

float Sensor::secondOrder_LPF(float x, float x_prev1, float x_prev2) {
    // Second order low pass filter implementation
    float y = b * x + a * x_prev1 + a * x_prev2;
    return y;
}

void Sensor::filter_data_1stOdr() {
    // Move all value by 1 position in the filtered buffers to make place for the new data
    for (int i = 0; i < 9; i++) {
        filtered_accel_data.x[i] = filtered_accel_data.x[i + 1];
        filtered_accel_data.y[i] = filtered_accel_data.y[i + 1];
        filtered_accel_data.z[i] = filtered_accel_data.z[i + 1];
        filtered_gyro_data.x[i] = filtered_gyro_data.x[i + 1];
        filtered_gyro_data.y[i] = filtered_gyro_data.y[i + 1];
        filtered_gyro_data.z[i] = filtered_gyro_data.z[i + 1];
        filtered_accel_data.timestamp[i] = filtered_accel_data.timestamp[i + 1];
        filtered_gyro_data.timestamp[i] = filtered_gyro_data.timestamp[i + 1];
    }

    // apply the first order filter to the new data and store in the filtered buffers
    filtered_accel_data.x[9] = firstOrder_LPF(raw_accel_data.x[9], filtered_accel_data.x[8]);
    filtered_accel_data.y[9] = firstOrder_LPF(raw_accel_data.y[9], filtered_accel_data.y[8]);
    filtered_accel_data.z[9] = firstOrder_LPF(raw_accel_data.z[9], filtered_accel_data.z[8]);
    filtered_gyro_data.x[9] = firstOrder_LPF(raw_gyro_data.x[9], filtered_gyro_data.x[8]);  
    filtered_gyro_data.y[9] = firstOrder_LPF(raw_gyro_data.y[9], filtered_gyro_data.y[8]);
    filtered_gyro_data.z[9] = firstOrder_LPF(raw_gyro_data.z[9], filtered_gyro_data.z[8]);
    filtered_accel_data.timestamp[9] = raw_accel_data.timestamp[9];
    filtered_gyro_data.timestamp[9] = raw_gyro_data.timestamp[9];
}

void Sensor::filter_data_2ndOdr() {
  // same as above, but 2nd order filtering
  // Move all value by 1 position in the filtered buffers to make place for the new data
    for (int i = 0; i < 9; i++) {
        filtered_accel_data.x[i] = filtered_accel_data.x[i + 1];
        filtered_accel_data.y[i] = filtered_accel_data.y[i + 1];
        filtered_accel_data.z[i] = filtered_accel_data.z[i + 1];
        filtered_gyro_data.x[i] = filtered_gyro_data.x[i + 1];
        filtered_gyro_data.y[i] = filtered_gyro_data.y[i + 1];
        filtered_gyro_data.z[i] = filtered_gyro_data.z[i + 1];
        filtered_accel_data.timestamp[i] = filtered_accel_data.timestamp[i + 1];
        filtered_gyro_data.timestamp[i] = filtered_gyro_data.timestamp[i + 1];
    }

    // apply the second order filter to the new data and store in the filtered buffers
    filtered_accel_data.x[9] = secondOrder_LPF(raw_accel_data.x[9], filtered_accel_data.x[8], filtered_accel_data.x[7]);
    filtered_accel_data.y[9] = secondOrder_LPF(raw_accel_data.y[9], filtered_accel_data.y[8], filtered_accel_data.y[7]);
    filtered_accel_data.z[9] = secondOrder_LPF(raw_accel_data.z[9], filtered_accel_data.z[8], filtered_accel_data.z[7]);
    filtered_gyro_data.x[9] = secondOrder_LPF(raw_gyro_data.x[9], filtered_gyro_data.x[8], filtered_gyro_data.x[7]);
    filtered_gyro_data.y[9] = secondOrder_LPF(raw_gyro_data.y[9], filtered_gyro_data.y[8], filtered_gyro_data.y[7]);
    filtered_gyro_data.z[9] = secondOrder_LPF(raw_gyro_data.z[9], filtered_gyro_data.z[8], filtered_gyro_data.z[7]);
    filtered_accel_data.timestamp[9] = raw_accel_data.timestamp[9];
    filtered_gyro_data.timestamp[9] = raw_gyro_data.timestamp[9];
}

inline bool Sensor::newDataReady() {
    return (raw_accel_data.unacked_data > 0) || (raw_gyro_data.unacked_data > 0);
}

sensor_buffer_t Sensor::getFilteredAccel() {
    // create the return structure
    sensor_buffer_t accel_data_out;
    // check if new unacked accel data is available
    if (!filtered_accel_data.unacked_data) {
        // If no new data available, return the error value =100
        accel_data_out.xyz_buffer = {0.0f, 0.0f, 0.0f};
        accel_data_out.timestamp_buffer = 0;
        accel_data_out.newData = false;
        return accel_data_out;
    }
    // If new data is available, return the last filtered data and timestamp
    accel_data_out.xyz_buffer = {filtered_accel_data.x[10 - filtered_accel_data.unacked_data], 
                                 filtered_accel_data.y[10 - filtered_accel_data.unacked_data],
                                 filtered_accel_data.z[10 - filtered_accel_data.unacked_data]};
    accel_data_out.timestamp_buffer = filtered_accel_data.timestamp[10 - filtered_accel_data.unacked_data];
    accel_data_out.newData = true;
    return accel_data_out;
}

sensor_buffer_t Sensor::getFilteredGyro() {
    // create the return structure
    sensor_buffer_t gyro_data_out;
    // check if new unacked gyro data is available
    if (!filtered_gyro_data.unacked_data) {
        // If no new data available, return the error value =100
        gyro_data_out.xyz_buffer = {0.0f, 0.0f, 0.0f};
        gyro_data_out.timestamp_buffer = 0;
        gyro_data_out.newData = false;
        return gyro_data_out;
    }
    // If new data is available, return the last filtered data and timestamp
    gyro_data_out.xyz_buffer = {filtered_gyro_data.x[10 - filtered_gyro_data.unacked_data], 
                                 filtered_gyro_data.y[10 - filtered_gyro_data.unacked_data],
                                 filtered_gyro_data.z[10 - filtered_gyro_data.unacked_data]};
    gyro_data_out.timestamp_buffer = filtered_gyro_data.timestamp[10 - filtered_gyro_data.unacked_data];
    gyro_data_out.newData = true;
    return gyro_data_out;
}

inline void Sensor::compute_a() {
    // Compute coefficient a based on cutoff frequency and sampling rate
    float fc = cutoff_freq;
    float fs = sampling_rate;
    a = exp(-2.0f * M_PI * fc / fs);
}

inline void Sensor::compute_b() {
    // Compute coefficient b based on cutoff frequency and sampling rate
    float fc = cutoff_freq;
    float fs = sampling_rate;
    b = 1.0f - exp(-2.0f * M_PI * fc / fs);
}


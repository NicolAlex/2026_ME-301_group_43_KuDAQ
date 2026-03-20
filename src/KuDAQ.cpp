#include "KuDAQ.h"

Sensor::Sensor(uint8_t __accel_addr, uint8_t __gyro_addr) { // Constructor
    // Set I2C addresses
    accel_addr = __accel_addr;
    gyro_addr = __gyro_addr;
    // initialize variables
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
    compute_a();
    compute_b();
}

Sensor::~Sensor() { // Destructor
    if (bmi) {
        delete bmi;
        bmi = nullptr;
    }
}

void Sensor::init() {
    // Initialize the bmi sensor inside of setup()
    // create BMI088 wrapper instance (uses Wire)
    bmi = new Bmi088(Wire, accel_addr, gyro_addr);
    // Initialize the sensors
    sensor_status = bmi->begin();
    if (sensor_status < 0) {
        while (1) {
            Serial.println("Failed to initialize BMI088 sensors!");
            delay(1000);
        }
    }
}

void Sensor::aquisition() {
    if (read_data()) {
        filter_data_1stOdr();
        // test
        Serial.print("nb unacked accel data: ");
        Serial.println(raw_accel_data.unacked_data);
    } 
}

bool Sensor::read_data() {
    // define time variable
    static unsigned long last_timestamp = 0;
    // read both accel and gyro via the bmi wrapper
    if (micros() - last_timestamp >= 1000000 / sampling_rate) {
        bmi->readSensor(); // readSensor() returns void, do not assign
        last_timestamp = micros();
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
        raw_accel_data.x[9] = bmi->getAccelX_mss();
        raw_accel_data.y[9] = bmi->getAccelY_mss();
        raw_accel_data.z[9] = bmi->getAccelZ_mss();
        raw_gyro_data.x[9] = bmi->getGyroX_rads();
        raw_gyro_data.y[9] = bmi->getGyroY_rads();
        raw_gyro_data.z[9] = bmi->getGyroZ_rads();
        raw_accel_data.timestamp[9] = last_timestamp;
        raw_gyro_data.timestamp[9] = last_timestamp;
        // increment the unacknowledged new data index
        raw_accel_data.unacked_data++;
        raw_gyro_data.unacked_data++;
        // overflow protection
        if (raw_accel_data.unacked_data > 10) {
            raw_accel_data.unacked_data = 10; // cap at buffer size
        }
        if (raw_gyro_data.unacked_data > 10) {
            raw_gyro_data.unacked_data = 10; // cap at buffer size
        }
        return true;
    }
    // else :
    // do not read data
    // do not update the buffers
    // do not increment the unacknowledged new data index
    return false;
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
    filtered_accel_data.unacked_data++;
    filtered_gyro_data.unacked_data++;
    // overflow protection
    if (filtered_accel_data.unacked_data > 10) {
        filtered_accel_data.unacked_data = 10; // cap at buffer size
    }
    if (filtered_gyro_data.unacked_data > 10) {
        filtered_gyro_data.unacked_data = 10; // cap at buffer size
    }

    // test
    Serial.print("filtered Accel X: ");
    Serial.println(filtered_accel_data.x[9]);
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

bool Sensor::newDataReady() {
    return (raw_accel_data.unacked_data > 0) || (raw_gyro_data.unacked_data > 0);
}

sensor_buffer_t Sensor::getFilteredAccel() {
    // create the return structure
    sensor_buffer_t accel_data_out;
    // check if new unacked accel data is available
    if (filtered_accel_data.unacked_data <= 0) {
        // If no new data available, return the error value =100
        accel_data_out.xyz_buffer = {30.0f, 30.0f, 30.0f};
        accel_data_out.timestamp = 0;
        accel_data_out.newData = false;
        return accel_data_out;
    }
    // If new data is available, return the last filtered data and timestamp
    accel_data_out.xyz_buffer = {filtered_accel_data.x[10 - filtered_accel_data.unacked_data], 
                                 filtered_accel_data.y[10 - filtered_accel_data.unacked_data],
                                 filtered_accel_data.z[10 - filtered_accel_data.unacked_data]};
    accel_data_out.timestamp = filtered_accel_data.timestamp[10 - filtered_accel_data.unacked_data];
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
        gyro_data_out.timestamp = 0;
        gyro_data_out.newData = false;
        return gyro_data_out;
    }
    // If new data is available, return the last filtered data and timestamp
    gyro_data_out.xyz_buffer = {filtered_gyro_data.x[10 - filtered_gyro_data.unacked_data], 
                                 filtered_gyro_data.y[10 - filtered_gyro_data.unacked_data],
                                 filtered_gyro_data.z[10 - filtered_gyro_data.unacked_data]};
    gyro_data_out.timestamp = filtered_gyro_data.timestamp[10 - filtered_gyro_data.unacked_data];
    gyro_data_out.newData = true;
    return gyro_data_out;
}

int Sensor::getUnackedAccelRaw() {
    return raw_accel_data.unacked_data;
}

int Sensor::getUnackedGyroRaw() {
    return raw_gyro_data.unacked_data;
}

int Sensor::getUnackedAccelFiltered() {
    return filtered_accel_data.unacked_data;
}

int Sensor::getUnackedGyroFiltered() {
    return filtered_gyro_data.unacked_data;
}

void Sensor::setCutoffFreq(float fc) {
    if (fc <= 0.0f || fc >= sampling_rate / 2.0f) {
        errorMsg.push_back("Invalid cutoff frequency! Must be between 0 and Nyquist frequency.");
        return; // Set to Nyquist frequency
    }
    cutoff_freq = fc;
    compute_a();
    compute_b();
}

void Sensor::setSamplingRate(float fs) {
    if (fs <= 0.0f) {
        errorMsg.push_back("Invalid sampling rate! Must be greater than 0.");
        return;
    }
    sampling_rate = fs;
    compute_a();
    compute_b();
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


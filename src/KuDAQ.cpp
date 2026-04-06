#include "KuDAQ.h"
// --- CONSTRUCTOR AND DESTRUCTOR ---

Sensor::Sensor(uint8_t __accel_addr, uint8_t __gyro_addr) { // Constructor
    // Set I2C addresses
    accel_addr = __accel_addr;
    gyro_addr = __gyro_addr;
    // initialize variables
    sensor_status = 0;
    a = 0.0f;
    b = 0.0f;
    raw_accel_data.index = 0;
    raw_gyro_data.index = 0;
    filtered_accel_data.index = 0;
    filtered_gyro_data.index = 0;
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
    sampling_rate = SAMPLING_RATE;
    cutoff_freq = CUTOFF_FREQ;
    compute_a();
    compute_b();
}

Sensor::~Sensor() { // Destructor
    if (bmi) {
        delete bmi;
        bmi = nullptr;
    }
}

// --- GENERAL METHODS ---

void Sensor::init() {
    // Initialize the bmi sensor inside of setup()
    // create BMI088 wrapper instance (uses Wire)
    bmi = new Bmi088(Wire, accel_addr, gyro_addr);
    ahrs = new Madgwick();
    // Initialize the sensors
    sensor_status = bmi->begin();
    if (sensor_status < 0) {
        while (1) {
            Serial.println("Failed to initialize BMI088 sensors!");
            delay(1000);
        }
    }
    // initialize the AHRS filter
    ahrs->begin(sampling_rate);
}

void Sensor::aquisition() {
    if (read_data()) {
        filter_data_1stOdr();
        AHRS_update();
        //filter_data_2ndOdr();
        // test
    } 
}

bool Sensor::read_data() {
    // define time variable
    static unsigned long last_timestamp = 0;
    // read both accel and gyro via the bmi wrapper
    bmi->readSensor();
    // check if enough time has passed since the last reading based on the sampling rate
    if (micros() - last_timestamp >= 1000000 / sampling_rate) {
        #ifdef DEBUG
        Serial.println("Reading new data from sensor...");
        #endif
        last_timestamp = micros();
        // Move data in the  circular buffer to make place for the new data
        raw_accel_data.index = (raw_accel_data.index + 1) % 10; // circular buffer index
        raw_gyro_data.index = (raw_gyro_data.index + 1) % 10; // circular buffer index
        // Add new data at the last position
        raw_accel_data.x[raw_accel_data.index] = bmi->getAccelX_mss();
        raw_accel_data.y[raw_accel_data.index] = bmi->getAccelY_mss();
        raw_accel_data.z[raw_accel_data.index] = bmi->getAccelZ_mss();
        raw_gyro_data.x[raw_gyro_data.index] = bmi->getGyroX_rads();
        raw_gyro_data.y[raw_gyro_data.index] = bmi->getGyroY_rads();
        raw_gyro_data.z[raw_gyro_data.index] = bmi->getGyroZ_rads();
        raw_accel_data.timestamp[raw_accel_data.index] = last_timestamp;
        raw_gyro_data.timestamp[raw_gyro_data.index] = last_timestamp;
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
        #ifdef DEBUG
        Serial.print("New data wrapped, raw unacked = ");
        Serial.println(raw_accel_data.unacked_data);
        Serial.print("accel_z = ");
        Serial.print(raw_accel_data.z[9], 3);
        Serial.print(", timestamp = ");
        Serial.println(raw_accel_data.timestamp[9]);
        #endif
        return true;
    }
    #ifdef DEBUG
    Serial.println("DO NOT READ");
    #endif
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
    #ifdef DEBUG
    Serial.println("Filtering data with 1st order filter...");
    #endif
    // Move all value by 1 position in the filtered buffers to make place for the new data
    filtered_accel_data.index = (filtered_accel_data.index + 1) % 10; // circular buffer index
    filtered_gyro_data.index = (filtered_gyro_data.index + 1) % 10; // circular buffer index
    // apply the first order filter to the new data and store in the filtered buffers
    if (filtered_accel_data.index == 0) {
        filtered_accel_data.x[filtered_accel_data.index] = firstOrder_LPF(raw_accel_data.x[raw_accel_data.index], filtered_accel_data.x[9]);
        filtered_accel_data.y[filtered_accel_data.index] = firstOrder_LPF(raw_accel_data.y[raw_accel_data.index], filtered_accel_data.y[9]);
        filtered_accel_data.z[filtered_accel_data.index] = firstOrder_LPF(raw_accel_data.z[raw_accel_data.index], filtered_accel_data.z[9]);
        filtered_gyro_data.x[filtered_gyro_data.index] = firstOrder_LPF(raw_gyro_data.x[raw_gyro_data.index], filtered_gyro_data.x[9]);
        filtered_gyro_data.y[filtered_gyro_data.index] = firstOrder_LPF(raw_gyro_data.y[raw_gyro_data.index], filtered_gyro_data.y[9]);
        filtered_gyro_data.z[filtered_gyro_data.index] = firstOrder_LPF(raw_gyro_data.z[raw_gyro_data.index], filtered_gyro_data.z[9]);
    } else {
        filtered_accel_data.x[filtered_accel_data.index] = firstOrder_LPF(raw_accel_data.x[raw_accel_data.index], filtered_accel_data.x[filtered_accel_data.index - 1]);
        filtered_accel_data.y[filtered_accel_data.index] = firstOrder_LPF(raw_accel_data.y[raw_accel_data.index], filtered_accel_data.y[filtered_accel_data.index - 1]);
        filtered_accel_data.z[filtered_accel_data.index] = firstOrder_LPF(raw_accel_data.z[raw_accel_data.index], filtered_accel_data.z[filtered_accel_data.index - 1]);
        filtered_gyro_data.x[filtered_gyro_data.index] = firstOrder_LPF(raw_gyro_data.x[raw_gyro_data.index], filtered_gyro_data.x[filtered_gyro_data.index - 1]);
        filtered_gyro_data.y[filtered_gyro_data.index] = firstOrder_LPF(raw_gyro_data.y[raw_gyro_data.index], filtered_gyro_data.y[filtered_gyro_data.index - 1]);
        filtered_gyro_data.z[filtered_gyro_data.index] = firstOrder_LPF(raw_gyro_data.z[raw_gyro_data.index], filtered_gyro_data.z[filtered_gyro_data.index - 1]);
    }
    filtered_accel_data.timestamp[filtered_accel_data.index] = raw_accel_data.timestamp[raw_accel_data.index];
    filtered_gyro_data.timestamp[filtered_gyro_data.index] = raw_gyro_data.timestamp[raw_gyro_data.index];

    filtered_accel_data.unacked_data++;
    filtered_gyro_data.unacked_data++;
    // overflow protection
    if (filtered_accel_data.unacked_data > 10) {
        filtered_accel_data.unacked_data = 10;  // cap at buffer size
    }
    if (filtered_gyro_data.unacked_data > 10) {
        filtered_gyro_data.unacked_data = 10; // cap at buffer size
    }
    #ifdef DEBUG
    Serial.print("New data filtered, filtered unacked = ");
    Serial.println(filtered_accel_data.unacked_data);
    Serial.print("accel_z = ");
    Serial.print(filtered_accel_data.z[9], 3);
    Serial.print(", timestamp = ");
    Serial.println(filtered_accel_data.timestamp[9]);
    #endif
}

void Sensor::filter_data_2ndOdr() {
  // same as above, but 2nd order filtering
  // Move all value by 1 position in the filtered buffers to make place for the new data
    filtered_accel_data.index = (filtered_accel_data.index + 1) % 10; // circular buffer index
    filtered_gyro_data.index = (filtered_gyro_data.index + 1) % 10; // circular buffer index
    // apply the second order filter to the new data and store in the filtered buffers
    if (filtered_accel_data.index == 0) {
        filtered_accel_data.x[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.x[raw_accel_data.index], filtered_accel_data.x[9], filtered_accel_data.x[8]);
        filtered_accel_data.y[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.y[raw_accel_data.index], filtered_accel_data.y[9], filtered_accel_data.y[8]);
        filtered_accel_data.z[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.z[raw_accel_data.index], filtered_accel_data.z[9], filtered_accel_data.z[8]);
        filtered_gyro_data.x[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.x[raw_gyro_data.index], filtered_gyro_data.x[9], filtered_gyro_data.x[8]);
        filtered_gyro_data.y[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.y[raw_gyro_data.index], filtered_gyro_data.y[9], filtered_gyro_data.y[8]);
        filtered_gyro_data.z[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.z[raw_gyro_data.index], filtered_gyro_data.z[9], filtered_gyro_data.z[8]);
    } else if (filtered_accel_data.index == 1) {
        filtered_accel_data.x[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.x[raw_accel_data.index], filtered_accel_data.x[0], filtered_accel_data.x[9]);
        filtered_accel_data.y[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.y[raw_accel_data.index], filtered_accel_data.y[0], filtered_accel_data.y[9]);
        filtered_accel_data.z[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.z[raw_accel_data.index], filtered_accel_data.z[0], filtered_accel_data.z[9]);
        filtered_gyro_data.x[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.x[raw_gyro_data.index], filtered_gyro_data.x[0], filtered_gyro_data.x[9]);
        filtered_gyro_data.y[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.y[raw_gyro_data.index], filtered_gyro_data.y[0], filtered_gyro_data.y[9]);
        filtered_gyro_data.z[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.z[raw_gyro_data.index], filtered_gyro_data.z[0], filtered_gyro_data.z[9]);
    } else {
        filtered_accel_data.x[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.x[raw_accel_data.index], filtered_accel_data.x[filtered_accel_data.index - 1], filtered_accel_data.x[filtered_accel_data.index - 2]);
        filtered_accel_data.y[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.y[raw_accel_data.index], filtered_accel_data.y[filtered_accel_data.index - 1], filtered_accel_data.y[filtered_accel_data.index - 2]);
        filtered_accel_data.z[filtered_accel_data.index] = secondOrder_LPF(raw_accel_data.z[raw_accel_data.index], filtered_accel_data.z[filtered_accel_data.index - 1], filtered_accel_data.z[filtered_accel_data.index - 2]);
        filtered_gyro_data.x[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.x[raw_gyro_data.index], filtered_gyro_data.x[filtered_gyro_data.index - 1], filtered_gyro_data.x[filtered_gyro_data.index - 2]);
        filtered_gyro_data.y[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.y[raw_gyro_data.index], filtered_gyro_data.y[filtered_gyro_data.index - 1], filtered_gyro_data.y[filtered_gyro_data.index - 2]);
        filtered_gyro_data.z[filtered_gyro_data.index] = secondOrder_LPF(raw_gyro_data.z[raw_gyro_data.index], filtered_gyro_data.z[filtered_gyro_data.index - 1], filtered_gyro_data.z[filtered_gyro_data.index - 2]);
    }
    filtered_accel_data.timestamp[filtered_accel_data.index] = raw_accel_data.timestamp[raw_accel_data.index];
    filtered_gyro_data.timestamp[filtered_gyro_data.index] = raw_gyro_data.timestamp[raw_gyro_data.index];
    filtered_accel_data.unacked_data++;
    filtered_gyro_data.unacked_data++;
    // overflow protection
    if (filtered_accel_data.unacked_data > 10) {
        filtered_accel_data.unacked_data = 10;  // cap at buffer size
    }
    if (filtered_gyro_data.unacked_data > 10) {
        filtered_gyro_data.unacked_data = 10; // cap at buffer size
    }
}

bool Sensor::newDataReady() {
    return (raw_accel_data.unacked_data > 0) || (raw_gyro_data.unacked_data > 0);
}

void Sensor::AHRS_update() {
    // Move all value by 1 position in the orientation buffers to make place for the new data
    raw_orientation_data.index = (raw_orientation_data.index + 1) % 10; // circular buffer index
    // apply the AHRS filter to the new data and store in the orientation buffers
    ahrs->updateIMU(raw_gyro_data.x[raw_gyro_data.index], raw_gyro_data.y[raw_gyro_data.index], raw_gyro_data.z[raw_gyro_data.index],
                 raw_accel_data.x[raw_accel_data.index], raw_accel_data.y[raw_accel_data.index], raw_accel_data.z[raw_accel_data.index]);
    raw_orientation_data.roll[raw_orientation_data.index] = ahrs->getRoll();
    raw_orientation_data.pitch[raw_orientation_data.index] = ahrs->getPitch();
    raw_orientation_data.yaw[raw_orientation_data.index] = ahrs->getYaw();
    raw_orientation_data.timestamp[raw_orientation_data.index] = (long)((raw_accel_data.timestamp[raw_accel_data.index] + raw_gyro_data.timestamp[raw_gyro_data.index]) / 2); // average timestamp
    raw_orientation_data.unacked_data++;
    if (raw_orientation_data.unacked_data > 10) {
        raw_orientation_data.unacked_data = 10; // cap at buffer size
    }
}

// --- GETTERS ---

sensor_buffer_t Sensor::getFilteredAccel() {
    // create the return structure
    sensor_buffer_t accel_data_out;
    // check if new unacked accel data is available
    #ifdef DEBUG
    Serial.print("Checking for new filtered accel data, unacked = ");
    Serial.println(raw_accel_data.unacked_data);
    #endif
    if (filtered_accel_data.unacked_data <= 0) {
        delay(1); // for debug
        // If no new data available, return the error value =100
        accel_data_out.xyz_buffer = {30.0f, 30.0f, 30.0f};
        accel_data_out.timestamp = 0;
        accel_data_out.newData = false;
        return accel_data_out;
    }
    // If new data is available, return the last filtered data and timestamp
    accel_data_out.xyz_buffer = {filtered_accel_data.x[(filtered_accel_data.index + 1 - filtered_accel_data.unacked_data + 10) % 10], 
                                 filtered_accel_data.y[(filtered_accel_data.index + 1 - filtered_accel_data.unacked_data + 10) % 10],
                                 filtered_accel_data.z[(filtered_accel_data.index + 1 - filtered_accel_data.unacked_data + 10) % 10]};
    accel_data_out.timestamp = filtered_accel_data.timestamp[(filtered_accel_data.index + 1 - filtered_accel_data.unacked_data + 10) % 10];
    accel_data_out.newData = true;
    filtered_accel_data.unacked_data--; // Mark this data as acknowledged
    raw_accel_data.unacked_data = filtered_accel_data.unacked_data; // Sync the unacked data
    #ifdef DEBUG
    Serial.print("Returning filtered accel data, unacked = ");
    Serial.println(filtered_accel_data.unacked_data);
    Serial.print("newData = ");
    Serial.println(accel_data_out.newData);
    Serial.print("accel_z = ");
    Serial.print(accel_data_out.xyz_buffer[2], 3);
    Serial.print(", timestamp = ");
    Serial.println(accel_data_out.timestamp);
    #endif
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
    gyro_data_out.xyz_buffer = {filtered_gyro_data.x[(filtered_gyro_data.index + 1 - filtered_gyro_data.unacked_data + 10) % 10], 
                                 filtered_gyro_data.y[(filtered_gyro_data.index + 1 - filtered_gyro_data.unacked_data + 10) % 10],
                                 filtered_gyro_data.z[(filtered_gyro_data.index + 1 - filtered_gyro_data.unacked_data + 10) % 10]};
    gyro_data_out.timestamp = filtered_gyro_data.timestamp[(filtered_gyro_data.index + 1 - filtered_gyro_data.unacked_data + 10) % 10];
    gyro_data_out.newData = true;
    filtered_gyro_data.unacked_data--; // Mark this data as acknowledged
    raw_gyro_data.unacked_data = filtered_gyro_data.unacked_data; // Sync the unacked data
    return gyro_data_out;
}

orientation_buffer_t Sensor::getRawOrientation() {
    // create the return structure
    orientation_buffer_t orientation_data_out;
    // check if new unacked orientation data is available
    if (!raw_orientation_data.unacked_data) {
        // If no new data available, return the error value =100
        orientation_data_out.rpy_buffer = {0.0f, 0.0f, 0.0f};
        orientation_data_out.timestamp = 0;
        orientation_data_out.newData = false;
        return orientation_data_out;
    }
    // If new data is available, return the last orientation data and timestamp
    orientation_data_out.rpy_buffer = {raw_orientation_data.roll[(raw_orientation_data.index + 1 - raw_orientation_data.unacked_data + 10) % 10], 
                                      raw_orientation_data.pitch[(raw_orientation_data.index + 1 - raw_orientation_data.unacked_data + 10) % 10],
                                      raw_orientation_data.yaw[(raw_orientation_data.index + 1 - raw_orientation_data.unacked_data + 10) % 10]};
    orientation_data_out.timestamp = raw_orientation_data.timestamp[(raw_orientation_data.index + 1 - raw_orientation_data.unacked_data + 10) % 10];
    orientation_data_out.newData = true;
    raw_orientation_data.unacked_data--; // Mark this data as acknowledged
    return orientation_data_out;
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

// --- SETTERS ---

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

// --- INLINE FUNCTIONS ---

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

// --- NON-CLASS FUNCTIONS ---
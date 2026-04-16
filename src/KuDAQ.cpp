#include "KuDAQ.h"

WiFiServer server(23);
WiFiClient client;

// --> --> --> CLASS SENSOR <-- <-- <--

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
        // If no new data available, set flag to false
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

// --> --> --> CLASS KuDAQ <-- <-- <--

// --- CONSTRUCTOR AND DESTRUCTOR ---

KuDAQ::KuDAQ() {
    // Initialize variables
    core0_state = CORE0_ORIENT_ACQ;
    core1_state = CORE1_ORIENT_STREAM;

}

KuDAQ::~KuDAQ() {
    // Clean up resources
    if (sensor1) {
        delete sensor1;
        sensor1 = nullptr;
    }
    if (sensor2) {
        delete sensor2;
        sensor2 = nullptr;
    }
    if (core0_stateQueue) {
        vQueueDelete(core0_stateQueue);
        core0_stateQueue = nullptr;
    }
    if (orientationQueue) {
        vQueueDelete(orientationQueue);
        orientationQueue = nullptr;
    }
}

// --- GENERAL METHODS ---

void KuDAQ::updateCore0() {
    // Core 0 is responsible for sensor data acquisition and processing
    // variables
    static orientation_buffer_t orientation1_sender;
    static CORE0_FSM core0_newState;
    // Check for new state commands from Core 1 via the core0 state queue
    if(xQueuePeek(core0_stateQueue, &core0_newState, 0) == pdTRUE) {
        core0_state = core0_newState;
        // Mark this state as received by clearing the queue
        xQueueReceive(core0_stateQueue, &core0_newState, 0);
    }

    // FSM
    switch(core0_state) {
        case CORE0_IDLE :
            // Wait for command from Core 1
            break;
        case CORE0_ORIENT_ACQ :
            // Acquire orientation data from both sensors and send to Core 1
            sensor1->aquisition();
            orientation1_sender = sensor1->getRawOrientation();
            // send only if there's new data
            if (orientation1_sender.newData) {
                // send orientation data to Core 1 via queue
                xQueueOverwrite(orientationQueue, &orientation1_sender);
                orientation1_sender.newData = false; // Mark data as sent
            }

            #ifdef DEBUG
                Serial.println("running : CORE0_ORIENT_ACQ");
            #endif

            break;
        case CORE0_ACCEL_ACQ :
            // Acquire accel and gyro data from both sensors, filter, and send to Core 1
            break;
    }

}

void KuDAQ::updateCore1() {
    // Core 1 is responsible for communication with the client and command processing
    // variables
    static std::vector<std::string> message;
    static orientation_buffer_t orientation1_receiver;
    static bool newMessage = false;
    static CORE1_FSM fallback_state = CORE1_IDLE; // To return to after command processing
    // Check WiFi connection and client status
    //WiFi_wellness();

    switch(core1_state) {
        case CORE1_IDLE :
            // Wait for client connection and commands
            fallback_state = CORE1_IDLE; // Stay in idle if no command is being processed
            break;
        case CORE1_CMD_PROCESSING :
            // Process incoming commands from the client
            cmdHandler(message);
            message.clear(); // Clear the message after processing
            core1_state = fallback_state; // Return to the previous state after command processing
            break;
        case CORE1_ORIENT_STREAM :
            // Stream orientation data to the client if connected
            if (client && client.connected()) {
                // Check if new orientation data is available from Core 0 via queue
                if (xQueuePeek(orientationQueue, &orientation1_receiver, 0) == pdTRUE) {
                    client.print("PITCH:");
                    client.println(orientation1_receiver.rpy_buffer[0], 3);
                    client.print("ROLL:");
                    client.println(orientation1_receiver.rpy_buffer[1], 3);
                    client.print("YAW:");
                    client.println(orientation1_receiver.rpy_buffer[2], 3);
                    // Mark this data as sent by clearing the queue
                    xQueueReceive(orientationQueue, &orientation1_receiver, 0);
                }
            } else {
                core1_state = CORE1_IDLE; // Return to idle if client disconnected
            }
            break;
        case CORE1_DEBUG_CMD :
            // placeholder
            break;
    }

    // If a client is connected, read messages and process commands
    if (client && client.connected()) {
        message = readMessage();
        if (!message.empty()) {
            client.println("message received !");
            core1_state = CORE1_CMD_PROCESSING;
        }
    }
}

void KuDAQ::initialize_all() {
    // Serial setup
    Serial.begin(115200);
    delay(100);

    // WiFi setup
    WiFi_setup();

    // Sensor setup
    // create sensor instances
    sensor1 = new Sensor(ACCEL1_ADDR, GYRO1_ADDR);
    sensor2 = new Sensor(ACCEL2_ADDR, GYRO2_ADDR);
    // Initialize the sensors
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // I2C initialization
    sensor1->init();
    sensor2->init();
    delay(100);

    // Queue setup
    orientationQueue = xQueueCreate(1, sizeof(orientation_buffer_t));
    if (orientationQueue == NULL) {
        Serial.println("Failed to create orientation queue!");
        while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    core0_stateQueue = xQueueCreate(1, sizeof(CORE0_FSM));
    if (core0_stateQueue == NULL) {
        Serial.println("Failed to create core0 state queue!");
        while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

void KuDAQ::WiFi_setup() {
    // WiFi connection setup
    WiFi.softAP(ssid, password); // Start WiFi in Access Point mode with given SSID and password
    Serial.println("Access Point started!");
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());  // usually 192.168.4.1
    server.begin(); // Start the TCP server on port 23
    delay(100); // wait for WiFi to initialize
}

void KuDAQ::core0_sendNewState(CORE0_FSM new_state) {
    // Send a new state to Core 0 via the core0 state queue
    xQueueOverwrite(core0_stateQueue, &new_state);
}

void KuDAQ::cmdHandler(std::vector<std::string> message) {
    // return if message is empty
    if (message.empty()) {
        return;
    }

    const std::string& command = message[0]; // First token is the command

    if (command == "set") {
        const std::string& param = message.size() > 1 ? message[1] : "";
        const std::string& value = message.size() > 2 ? message[2] : "";

        if (param == "cf") {
            float new_freq = std::stof(value);
            sensor1->setCutoffFreq(new_freq);
            sensor2->setCutoffFreq(new_freq);
            Serial.print("Cutoff frequency set to ");
            Serial.println(new_freq);
        } else if (param == "sr") {
            float new_sr = std::stof(value);
            sensor1->setSamplingRate(new_sr);
            sensor2->setSamplingRate(new_sr);
            Serial.print("Sampling rate set to ");
            Serial.println(new_sr);
        }
         // Add more parameters as needed
    }
    else if (command == "get") {
        const std::string& param = message.size() > 1 ? message[1] : "";
        const std::string& value = message.size() > 2 ? message[2] : "";

        if (param == "cf") {
            float current_cf = sensor1->getCutoffFreq(); // Assuming both sensors have the same cutoff frequency
            Serial.print("Current cutoff frequency is ");
            Serial.println(current_cf);
        } else if (param == "sr") {
            float current_sr = sensor1->getSamplingRate(); // Assuming both sensors have the same sampling rate
            Serial.print("Current sampling rate is ");
            Serial.println(current_sr);
        }
         // Add more parameters as needed
    }
    else if (command == "stream") {
        const std::string& param = message.size() > 1 ? message[1] : "";

        if (param == "orient") {
            // Trigger Core 0 to start streaming orientation data
            ;
        }
    }
    else if (command == "debug") {
        const std::string& param = message.size() > 1 ? message[1] : "";
        const std::string& value = message.size() > 2 ? message[2] : "";

        if (param == "cmd") {
            if (value == "on") {
                core1_state = CORE1_DEBUG_CMD;
                Serial.println("Debug mode enabled.");
            } 
            else if (value == "off") {
                #undef DEBUG
                Serial.println("Debug mode disabled.");
            }
        }
    }


    else {
        // In case of invalid message, print to TCP the received message for debugging
        if (client && client.connected()) {
            client.print("Invalid command received: ");
            for (size_t i = 0; i < message.size(); i++) {
                if (i > 0) client.print(" ");
                client.print(message[i].c_str());
            }
            client.println();
        }
    }
    // Add more command handling as needed
}

std::vector<std::string> KuDAQ::readMessage() {
    // Read a full line from the TCP stream, then split into max 3 tokens.
    static std::string rx_buffer;
    std::vector<std::string> parsed_message;
    bool got_newline = false;

    if (!client || !client.connected()) {
        return parsed_message; // Return empty if no client is connected
    }

    while (client.available() > 0) {
        char c = static_cast<char>(client.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            got_newline = true;
            break;
        }
        rx_buffer.push_back(c);
    }

    // Return only when a full line has been received.
    if (!got_newline) {
        return parsed_message;
    }

    if (rx_buffer.empty()) {
        return parsed_message;
    }

    // Parse command and up to 2 arguments.
    size_t pos = 0;
    while (pos < rx_buffer.size() && parsed_message.size() < 3) {
        while (pos < rx_buffer.size() && rx_buffer[pos] == ' ') {
            ++pos;
        }
        if (pos >= rx_buffer.size()) {
            break;
        }

        size_t start = pos;
        while (pos < rx_buffer.size() && rx_buffer[pos] != ' ') {
            ++pos;
        }
        parsed_message.push_back(rx_buffer.substr(start, pos - start));
    }

    rx_buffer.clear();
    return parsed_message;
}
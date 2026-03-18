#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include <math.h>

#define I2C_SDA_PIN 23
#define I2C_SCL_PIN 19

/* accel object */
Bmi088Accel accel_1(Wire,0x18);
Bmi088Gyro gyro_1(Wire,0x68);

Bmi088Accel accel_2(Wire,0x19);
Bmi088Gyro gyro_2(Wire,0x69);

// Fonction pour calculer le coefficient 'a'
double calculate_a(double fc, double Ts) {
    return exp(-2.0 * M_PI * fc * Ts);
}

// Fonction du filtre passe-bas du premier ordre
double low_pass_filter(double x_k, double x_k_prev, double a, double b) {
    return b * x_k + a * x_k_prev;
}

// Fréquence de coupure et période d'échantillonnage
  double fc = 50.0; // en Hz
  double Ts = 0.01;  // en secondes

void setup() 
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400 kHz I2C
    int status;
    /* USB Serial to print data */
    Serial.begin(115200);
    while(!Serial) {}
    /* start the sensors */
    status = accel_1.begin();
    if (status < 0) {
        Serial.println("Accel 1 Initialization Error");
        Serial.println(status);
        while (1) {}
  }
  status = gyro_1.begin();
  if (status < 0) {
    Serial.println("Gyro 1 Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = accel_2.begin();
    if (status < 0) {
        Serial.println("Accel 2 Initialization Error");
        Serial.println(status);
        while (1) {}
  }
  status = gyro_2.begin();
  if (status < 0) {
    Serial.println("Gyro 2 Initialization Error");
    Serial.println(status);
    while (1) {}
  }
}

void loop() 
{
  static float X, Y, Z;
  static float X_prev = 0.0, Y_prev = 0.0, Z_prev = 0.0;
  //static float X_rads, Y_rads, Z_rads;
  static constexpr int div = 60;
  static constexpr float scale = 20.0f / div;

    accel_1.readSensor();
    gyro_1.readSensor();

    X = accel_1.getAccelX_mss();
    Y = accel_1.getAccelY_mss();
    Z = accel_1.getAccelZ_mss();

    // Calcul des coefficients
    double a = calculate_a(fc, Ts);
    double b = 1 - a;

    // Application du filtre passe-bas
    X = low_pass_filter(X, X_prev, a, b);
    Y = low_pass_filter(Y, Y_prev, a, b);
    Z = low_pass_filter(Z, Z_prev, a, b);

    // Mise à jour des valeurs précédentes
    X_prev = X;
    Y_prev = Y;
    Z_prev = Z;




    Serial.print(">AccelX:");
    Serial.println(X, 3);
    Serial.print(">AccelY:");
    Serial.println(Y, 3);
    Serial.print(">AccelZ:");
    Serial.println(Z, 3);

    delay(10); // Délai pour correspondre à la période d'échantillonnage de 10 ms
}
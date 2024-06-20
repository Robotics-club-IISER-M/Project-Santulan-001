#ifndef SENSOR_H
#define SENSOR_H

#include <Wire.h>

class Sensor {
public:
    // Constructor
    Sensor();

    // Function prototypes
    void setupSensor();
    void computeOrientation();

    // Data members (variables)
    float roll, pitch, yaw; // Euler angles

private:
    // Helper functions
    void MPU_read_accel_data();
    void MPU_read_gyro_data();
    void calculate_IMU_error();

    // MPU6050 I2C address
    const int MPU_ADDRESS = 0x68;

    // MPU6050 sensor data
    float AccX, AccY, AccZ; // Accelerometer readings
    float GyroX, GyroY, GyroZ; // Gyroscope readings

    // Error correction variables
    float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

    // Time variables for orientation calculation
    float elapsedTime, currentTime, previousTime;
};

#endif

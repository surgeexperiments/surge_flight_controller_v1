/**
 *	@file structs.h
 *	@author SurgeExperiments
 *
 *	@brief This file contains the defines, enums and structs that are used
           throughout the program for several files.
 */

#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_

#include <stdint.h>
#include "enums.h"

/* FLY MODES */
// #define MOTORS_OFF 0
// #define MOTORS_STANDBY_MODE 1
// #define MOTORS_FLY_MODE 2
// #define MOTORS_ACROBATIC 3
// #define MOTORS_RX_SIGNAL_LOST 4

/**
 * @brief Struct for storing real-time stuff for the drone, used by main.c
 *
 *
 */
typedef struct
{
    flightModeSelector_et levelling_mode;
    int esc_voltage_compensation;

    /* set to 1 to use tpa */
    uint8_t use_tpa;
} droneRealTimeSettings_st;

/* STRUCTURES */

/* Not using int16_t creates problems w/the MPU6050 ect, don't change it */
typedef struct
{
    /* X axis */
    int16_t roll;
    /* Y axis */
    int16_t pitch;
    /* Z axis */
    int16_t z;
} accelData_st;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} gyroData_st;

typedef struct
{
    float temp;
} temperatureData_st;

/**
 * @brief struct containing all IMU data for real time drone operation.
 */
typedef struct
{
    /* Buffer to receive data from the IMU's */
    uint8_t dataBuffer[14];
    gyroData_st gyroCalibrationData;
    gyroData_st gyroDataRaw;
    gyroData_st gyroDataFilteredToDegSeconds;
    accelData_st accelDataRaw;
    accelData_st accelDataFiltered;
    accelData_st accelDataScale;
    accelData_st accelCalibrationData;
    temperatureData_st tempDataRaw, tempDataScaled;
} imuDataCollection_st;

/**
 * Fields needs to be volatile as they are used by interrupts
 */
typedef struct
{
    /* Roll */
    volatile uint32_t one;
    /* Pitch */
    volatile uint32_t two;
    /* Throttle */
    volatile uint32_t three;
    /* Yaw */
    volatile uint32_t four;
} rxChannels_st;

typedef struct
{
    uint16_t one;
    uint16_t two;
    uint16_t three;
    uint16_t four;
    uint16_t five;
    uint16_t six;
} escOutput_st;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} pidData_st;

/* Hold all data required for PID algo */
typedef struct
{
    float proportional;
    float proportionalGain;
    float integral;
    float integralGain;
    float derivative;
    float derivativeGain;
    float previousPidError;
    float pidMaxOutput;
    float output;
} pidStorage_st;

/* struct for storing the drone's angles, for use with auto-level */
typedef struct
{
    float pitchAngleGyro;
    float rollAngleGyro;
    float pitchAngleAccel;
    float rollAngleAccel;
} imuOrientation_st;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} imuOrientationEuler_st;

typedef struct
{
    float pitchAngle;
    float rollAngle;
} angleOrientation_st;

typedef struct
{
    float beta;
    float zeta;
    float deltat;
    float now;
    float lastUpdate;
    float q[4];
} madgwickQuaternionVars_st;

/* Used for Kalman algo found online */
typedef struct
{
    /* Process noise variance for the accelerometer */
    float Q_angle;
    /* Process noise variance for the gyro bias */
    float Q_bias;
    /* Measurement noise variance - this is actually the variance of the measurement noise */
    float R_measure;

    /* The angle calculated by the Kalman filter - part of the 2x1 state vector */
    float angle;
    /* The gyro bias calculated by the Kalman filter - part of the 2x1 state vector */
    float bias;
    /* Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate */
    float rate;

    /* Error covariance matrix - This is a 2x2 matrix */
    float P[2][2];
    /* Kalman gain - This is a 2x1 vector */
    float K[2];
    /* Angle difference */
    float y;
    /* Estimate error */
    float S;

    float gyroRate;
    float intermediateAngle;
    float finalAngle;
} kalmanVariables_st;

typedef struct
{
    /* 2 * proportional gain (Kp) */
    float twoKp;
    /* 2 * integral gain (Ki) */
    float twoKi;
    /* quaternion of sensor frame relative to auxiliary frame */
    float q0, q1, q2, q3;
    /* integral error terms scaled by Ki */
    float integralFBx, integralFBy, integralFBz;
    float invSampleFreq;
    float roll, pitch, yaw;
    char anglesComputed;
} mahonyVariables_st;

/* LCD AND LCD-MENU DATA STRUCTURES */
typedef struct
{
    /* Menu variables */
    char **introString;
    char **mainMenuNames;
    /* PID values: Roll P, Roll I, Roll D, Pitch P, Pitch I, Pitch D, Yaw P, Yaw I, Yaw D */
    char **PID_menu;
    char **levelling_menu;
    char **level_modes;
} menuStrings_st;

typedef struct
{
    long *lastButtonChangeTime;
    int *pinMapping;
    int *inputState;
    /* Cleared after each time */
    int *buttonPressEvent;
    int *lastInputState;
    int debounceCutoff;
} buttonHandling_st;

typedef struct
{
    /* Wire Address of the LCD */
    uint8_t addr;
    /* the backlight intensity */
    uint8_t backLight;
    /* lines and dots mode */
    uint8_t displayFunction;
    /* cursor, display, blink flags */
    uint8_t displayControl;
    /* left2right, autoscroll */
    uint8_t displayMode;
    /* The number of rows the display supports. */
    uint8_t numLines;
} lcdVariables_st;

#endif /* DATA_TYPES_H_ */

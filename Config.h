#import "Arduino.h"

// MACROS //

#define SensorDataIndex(x) ((sensorDataRingBufferIndex + kSensorDataBufferSize - x) % kSensorDataBufferSize)
#define SensorData(x) (sensorDataRingBuffer[SensorDataIndex(x)])

// CONSTANTS //

const int kLEDArrayCount = 5; // Number of LED arrays
const int kLEDArrayPins[kLEDArrayCount] = {
  9, 10, 11, 5, 6}; // PWM output pins for LED arrays

const int kRangeSensorPin = A0; // Analog input pin for ultrasonic range sensor
const int kSensorJitter = 2; // Allowed jitter in sensor reading without registering motion
const int kSensorDataBufferSize = 256; // Number of samples in sensor reading ring buffer

const int kMedianFilterSampleCount = 10; // Number of samples for running median filter
const unsigned long kCalibrationStableTime = 2000; // Required interval without motion for range recalibration (ms)
const int kHysteresisInterval = 500; // Contiguous interval in opposite detection state required to change (ms)
const unsigned long kVarianceWindow = 100; // Time window for measuring past variances from current distance (ms)
const float kLoopIntervalSmoothingFactor = 0.99; // Smoothing factor for estimated loop interval
const unsigned long kLoopIntervalMin = 2; // Lower bound for estimated loop interval

const int kHandDistanceControlWindow = 100; // Max deviation from initial hand detection distance
const float kHandDistanceSmoothingFactor = 0.99; // Smoothing factor for hand distance control
  
const uint8_t kCIELightnessTable[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };
const float kMinVisibleBrightness = (float(28)/255);

//const float kNormalTable[] = {
//  0.500,0.492,0.484,0.476,0.468,0.460,0.452,0.444,0.436,0.429,0.421,0.413,0.405,0.397,0.390,0.382,0.374,
//  0.367,0.359,0.352,0.345,0.337,0.330,0.323,0.316,0.309,0.302,0.295,0.288,0.281,0.274,0.268,0.261,0.255,
//  0.248,0.242,0.236,0.230,0.224,0.218,0.212,0.206,0.200,0.195,0.189,0.184,0.179,0.174,0.169,0.164,0.159,
//  0.154,0.149,0.145,0.140,0.136,0.131,0.127,0.123,0.119,0.115,0.111,0.107,0.104,0.100,0.097,0.093,0.090,
//  0.087,0.084,0.081,0.078,0.075,0.072,0.069,0.067,0.064,0.062,0.059,0.057,0.055,0.053,0.051,0.048,0.046,
//  0.045,0.043,0.041,0.039,0.038,0.036,0.034,0.033,0.031,0.030,0.029,0.027,0.026,0.025,0.024,0.023,0.022,
//  0.021,0.020,0.019,0.018,0.017,0.016,0.015,0.015,0.014,0.013,0.013,0.012,0.011,0.011,0.010,0.010,0.009,
//  0.009,0.008,0.008,0.007,0.007,0.007,0.006,0.006,0.006,0.005,0.005,0.005,0.004,0.004,0.004,0.004,0.003,
//  0.003,0.003,0.003,0.003,0.003,0.002,0.002,0.002,0.002,0.002,0.002,0.002,0.002,0.001,0.001,0.001,0.001 };
//const int kNormalTableSize = sizeof(kNormalTable) / sizeof(kNormalTable[0]);

// STRUCTS AND ENUMS //

struct SensorReading {
  unsigned long timestamp;
  int distance;
};

struct LEDArray {
  int pin;
  float brightness;
};

enum DisplayPattern {
  DisplayPatternNone
};


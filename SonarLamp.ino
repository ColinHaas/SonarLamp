// CONSTANTS //

const int kLEDArrayCount = 5; // Number of LED arrays
const int kLEDArrayPins[kLEDArrayCount] = {
  5, 6, 9, 10, 11}; // PWM output pins for LED arrays

const int kRangeSensorPin = A0;
const int kMedianFilterSampleCount = 10; // Number of samples for running median filter
const unsigned long kCalibrationStableTime = 5000; // Required interval without motion for range recalibration (ms)
const int kSensorJitter = 1; // Allowed jitter in sensor reading without registering motion
const int kSensorDataBufferSize = 128; // Number of samples in sensor reading ring buffer

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

// STRUCTS AND ENUMS //

struct SensorReading {
  unsigned long timestamp;
  int distance;
};

struct LEDArray {
  int pin;
  float brightness;
};

// GLOBALS //

LEDArray LEDArrays[5];

int sensorDataRingBuffer[kSensorDataBufferSize];
int sensorDataBufferIndex;
long motionDetectTimestamp;
long rangeCalibrationTimestamp;
int calibratedMaxRange;

// MACROS //

#define SensorDataIndex(x) ((sensorDataBufferIndex + kSensorDataBufferSize - x) % kSensorDataBufferSize)
#define SensorData(x) (sensorDataBuffer[SensorDataIndex(x)])

void setup() 
{
  // Start serial communication
  Serial.begin(9600); 

  // Initialize LED arrays
  for (uint8_t index = 0; index < kLEDArrayCount; index++) {
    LEDArrays[index].pin = kLEDArrayPins[index];
    LEDArrays[index].brightness = 0.2;
  }

  // Set 1.1V internal analog reference
  analogReference(INTERNAL);
}

void loop()
{      
  // SENSOR DATA ACQUISITION //

  // Increment sensor data ring buffer index (loops around) 
  sensorDataBufferIndex = (sensorDataBufferIndex + 1) % kSensorDataBufferSize;

  // Cache current distance and time info then add distance to ring buffer
  SensorReading newSensorData = getFilteredSensorReading();
  int currentDistance = newSensorData.distance;
  unsigned long currentTime = newSensorData.timestamp;
  sensorDataRingBuffer[sensorDataBufferIndex] = currentDistance;

  // MOTION DETECTION AND RANGE AUTO-CALIBRATION //

  // Retrieve previous distance measurement
  int previousBufferIndex = (sensorDataBufferIndex + kSensorDataBufferSize - 1) % kSensorDataBufferSize;
  int previousDistance = sensorDataRingBuffer[previousBufferIndex];

  // Check for motion beyond expected sensor jitter and note time if detected 
  if (abs(currentDistance - previousDistance) > kSensorJitter) {
    motionDetectTimestamp = currentTime;
    Serial.println("Motion detected");
  } 
  // Recalibrate default sensor range if readings have been stable for required time 
  else if (currentTime - motionDetectTimestamp > kCalibrationStableTime) {//removeme
    // Check that motion has been detected since last recalibration
    if (motionDetectTimestamp > rangeCalibrationTimestamp) {
      calibratedMaxRange = currentDistance;
      rangeCalibrationTimestamp = currentTime;
      Serial.println("Range recalibrated");
    }
  } 

  // OUTPUT UPDATE //

  // Update LED array brightnesses
  for (uint8_t index = 0; index < kLEDArrayCount; index++) {
    // Determine CIE psychoreactive lightness fr raw LED brightness value
    uint8_t cieLightness = kCIELightnessTable[int(LEDArrays[index].brightness * 255)];
    // Update PWM outputs
    analogWrite(LEDArrays[index].pin, cieLightness);
  }
}

struct SensorReading getFilteredSensorReading()
{
  static SensorReading filterRingBuffer[kMedianFilterSampleCount];
  static SensorReading sortedFilterBuffer[kMedianFilterSampleCount];
  static int ringBufferIndex = 0;
  static unsigned long lastReturnedReadingTime;

  // Get median-filtered readings until correctly time-ordered reading is available
  SensorReading filteredReading; 
  do {
    // Get new sensor reading and timestamp
    int distance = analogRead(kRangeSensorPin);
    unsigned long timestamp = millis();

    // Add reading to ring buffer
    filterRingBuffer[ringBufferIndex].distance = distance;
    filterRingBuffer[ringBufferIndex].timestamp = int(timestamp);

    // Increment ring buffer index
    ringBufferIndex = (ringBufferIndex + 1) % kMedianFilterSampleCount;

    // Create copy of ring buffer for sorting
    memcpy(sortedFilterBuffer, filterRingBuffer, kMedianFilterSampleCount * sizeof(SensorReading));

    // Sort copy of ring buffer
    for (int i = 1; i < kMedianFilterSampleCount; ++i)
    {
      SensorReading compareReading = sortedFilterBuffer[i];
      int compareIndex;
      for (compareIndex = i - 1; (compareIndex >= 0) && (compareReading.distance > sortedFilterBuffer[compareIndex].distance); compareIndex--) {
        sortedFilterBuffer[compareIndex + 1] = sortedFilterBuffer[compareIndex];
      }
      sortedFilterBuffer[compareIndex + 1] = compareReading;
    }

    // Get median reading from sorted buffer
    filteredReading =  sortedFilterBuffer[kMedianFilterSampleCount / 2];
  } 
  while (filteredReading.timestamp <= lastReturnedReadingTime);

  // Save sensor reading timestamp and return reading
  lastReturnedReadingTime = filteredReading.timestamp;
  return filteredReading;
}

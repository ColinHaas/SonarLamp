#import "Config.h"

// GLOBALS //

LEDArray LEDArrays[5];
int sensorDataRingBuffer[kSensorDataBufferSize];
int sensorDataRingBufferIndex;
unsigned long motionDetectTimestamp;
int calibratedMaxRange;
boolean handDetectState;
int loopInterval;
int handDetectInitialDistance;
float handControlPosition;

void setup()
{
  // Start serial communication
  Serial.begin(115200);

  // Initialize LED arrays
  for (uint8_t index = 0; index < kLEDArrayCount; index++) {
    LEDArrays[index].pin = kLEDArrayPins[index];
    LEDArrays[index].brightness = 0.0;
  }

  // Set 1.1V internal analog reference
  analogReference(INTERNAL);

  Serial.println("Setup complete");
}

void loop()
{
  // Update LED brightnesses
  updateOutput();

  // SENSOR DATA ACQUISITION //

  // Estimate loop interval (for converting time interval constants to sample counts)
  static unsigned long loopTimeMicros = 0;
  unsigned long loopIntervalMicros = micros() - loopTimeMicros;
  loopTimeMicros += loopIntervalMicros;
  static unsigned long smoothedLoopIntervalMicros = loopIntervalMicros;
  smoothedLoopIntervalMicros = kLoopIntervalSmoothingFactor * smoothedLoopIntervalMicros + (1.0 - kLoopIntervalSmoothingFactor) * loopIntervalMicros;
  loopInterval = max(smoothedLoopIntervalMicros / 1000, kLoopIntervalMin);

  // Increment sensor data ring buffer index (loops around)
  sensorDataRingBufferIndex = (sensorDataRingBufferIndex + 1) % kSensorDataBufferSize;

  // Cache current distance and time info then add distance to ring buffer
  SensorReading newSensorData = getFilteredSensorReading();
  int currentDistance = newSensorData.distance;
  unsigned long currentTime = newSensorData.timestamp;
  sensorDataRingBuffer[sensorDataRingBufferIndex] = currentDistance;

  // MOTION DETECTION AND RANGE CALIBRATION //

  // Retrieve previous distance measurement
  int previousBufferIndex = (sensorDataRingBufferIndex + kSensorDataBufferSize - 1) % kSensorDataBufferSize;
  int previousDistance = sensorDataRingBuffer[previousBufferIndex];

  // Check for motion beyond expected sensor jitter and note time if detected
  if (abs(currentDistance - previousDistance) > kSensorJitter) {
    motionDetectTimestamp = currentTime;
    // Serial.println("Motion detected");
  }

  // Check if max sensor range has been calibrated
  if (!calibratedMaxRange) {
    // Recalibrate max range if readings have been stable for required time
    if (currentTime - motionDetectTimestamp > kCalibrationStableTime) {
      calibratedMaxRange = currentDistance;
      for (uint8_t index = 0; index < kLEDArrayCount; index++) {
        LEDArrays[index].brightness = kMinVisibleBrightness;
      }
      Serial.println("Range calibrated");
    }
    // Skip data analysis if not yet calibrated
    return;
  }

  // Compute number of consecutive state change samples required for state change threshold
  int hysteresisSampleCountThreshold = kHysteresisInterval / loopInterval;

  // Count consecutive detection state change samples
  static int hysteresisSampleCount;
  boolean sampleDetect = (calibratedMaxRange - currentDistance) > kSensorJitter;
  boolean stateChange = (handDetectState && !sampleDetect) || (!handDetectState && sampleDetect);
  hysteresisSampleCount = stateChange ? (hysteresisSampleCount + 1) : 0;

  // Change hand detection state if past hysteresis threshold
  if (hysteresisSampleCount > hysteresisSampleCountThreshold) {
    handDetectState = !handDetectState;
    hysteresisSampleCount = 0;
    handControlPosition = 0;
    handDetectInitialDistance = handDetectState ? currentDistance : 0;
    Serial.println(handDetectState ? "Hand detected" : "Hand not detected");
  }

  // Track change in hand position from initial detection distance
  if (handDetectState) {
    int currentHandControlPosition = currentDistance - handDetectInitialDistance;
    if (abs(currentHandControlPosition) < kHandDistanceControlWindow) {
      // Smooth hand position to avoid LED flickering
      float cachedHandControlPosition = handControlPosition;
      handControlPosition = handControlPosition * kHandDistanceSmoothingFactor + float(currentHandControlPosition) * (1 - kHandDistanceSmoothingFactor);
      float handControlPositionDelta = handControlPosition - cachedHandControlPosition;
      // Change brightness of LEDs based on change in hand position
      // Serial.println(handControlPosition);
      for (uint8_t index = 0; index < kLEDArrayCount; index++) {
        LEDArrays[index].brightness += (handControlPositionDelta / kHandDistanceControlWindow);
        LEDArrays[index].brightness = constrain(LEDArrays[index].brightness, 0, 1);
      }
    }
  }
}

void updateOutput()
{
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

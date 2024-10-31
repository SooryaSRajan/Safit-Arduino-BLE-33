#include <Arduino.h>
#include <ArduinoBLE.h>

#define HEART_SERVICE_UUID        "180D"

#define HEART_RATE_UUID          "2A37" // Standard Heart Rate Measurement
#define HRV_UUID                 "2B90" // Custom UUID for HRV
#define HRMAD10_UUID            "2B91" // Custom UUID for HRMAD10
#define HRMAD30_UUID            "2B92" // Custom UUID for HRMAD30
#define HRMAD60_UUID            "2B93" // Custom UUID for HRMAD60

BLEService heartService(HEART_SERVICE_UUID);
// Define characteristics - using float for precision
BLEFloatCharacteristic heartRateChar(HEART_RATE_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrvChar(HRV_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrMad10Char(HRMAD10_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrMad30Char(HRMAD30_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrMad60Char(HRMAD60_UUID, BLERead | BLENotify);

// Pin definition
const int ecgPin = A0; // Analog pin where ECG signal is read
const int loPlusPin = 9;     // LO+ pin for lead detection on digital pin 9
const int loMinusPin = 10;   // LO- pin for lead detection on digital pin 10


// Variables for signal processing
unsigned long lastBeatTime = 0;   // Stores the time of the last heartbeat
int beatInterval = 0;             // Time between beats (R-R interval)
int heartRate = 0;                // Calculated heart rate in bpm
const int sampleWindow = 10;      // Sample window for analog reads
int threshold = 652;              // ECG signal threshold to detect R-peak
const int numRRIntervals = 10;    // Store last 10 R-R intervals for HRV

int rrIntervals[numRRIntervals];  // Array to store R-R intervals
int rrIndex = 0;                  // Index for the array

// Buffers for HRMAD10, HRMAD30, and HRMAD60
const int bufferSize10 = 10;
const int bufferSize30 = 30;
const int bufferSize60 = 60;

int hrBuffer10[bufferSize10];
int hrBuffer30[bufferSize30];
int hrBuffer60[bufferSize60];

int index10 = 0, index30 = 0, index60 = 0;
int bufferCount10 = 0, bufferCount30 = 0, bufferCount60 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ecgPin, INPUT);

  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the local name and advertised service
  BLE.setLocalName("Safit");
  BLE.setAdvertisedService(heartService);

  // Add all characteristics to the service
  heartService.addCharacteristic(heartRateChar);
  heartService.addCharacteristic(hrvChar);
  heartService.addCharacteristic(hrMad10Char);
  heartService.addCharacteristic(hrMad30Char);
  heartService.addCharacteristic(hrMad60Char);

  // Set initial values for characteristics
  heartRateChar.writeValue(0.0);
  hrvChar.writeValue(0.0);
  hrMad10Char.writeValue(0.0);
  hrMad30Char.writeValue(0.0);
  hrMad60Char.writeValue(0.0);

  // Add the service
  BLE.addService(heartService);
  BLE.advertise();

  Serial.println("Heart Monitor BLE Service is now advertising!");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      int ecgValue = analogRead(ecgPin);  // Read ECG signal from AD8232

      // Simple peak detection
      if (ecgValue > threshold) {
        unsigned long currentTime = millis();
        
        // Detect the R-peak and calculate the time between beats (R-R interval)
        beatInterval = currentTime - lastBeatTime;
        if (beatInterval > 300) {  // Avoid noise; set minimum R-R interval (~200 bpm)
          lastBeatTime = currentTime;
          heartRate = 60000 / beatInterval;  // Convert R-R interval to bpm
          
          // Store the R-R interval
          rrIntervals[rrIndex] = beatInterval;
          rrIndex = (rrIndex + 1) % numRRIntervals;
          
          // Print ECG value and HR
          Serial.print("ECG Value: ");
          Serial.print(ecgValue);
          Serial.print("\tHeart Rate: ");
          Serial.print(heartRate);
          Serial.println(" bpm");
          
          updateHRBuffer(hrBuffer10, bufferSize10, index10, bufferCount10, heartRate);
          updateHRBuffer(hrBuffer30, bufferSize30, index30, bufferCount30, heartRate);
          updateHRBuffer(hrBuffer60, bufferSize60, index60, bufferCount60, heartRate);

          // Calculate and print HRV, HRMAD10, HRMAD30, and HRMAD60
          float hrv = calculateHRV(rrIntervals);
          float hrmad10 = calculateHRMAD(hrBuffer10, bufferCount10);
          float hrmad30 = calculateHRMAD(hrBuffer30, bufferCount30);
          float hrmad60 = calculateHRMAD(hrBuffer60, bufferCount60);

          Serial.print("HRV: ");
          Serial.println(hrv);
          Serial.print("HRMAD10: ");
          Serial.println(hrmad10);
          Serial.print("HRMAD30: ");
          Serial.println(hrmad30);
          Serial.print("HRMAD60: ");
          Serial.println(hrmad60);

          // Write values to BLE characteristics
          heartRateChar.writeValue((float)heartRate);
          hrvChar.writeValue(hrv);
          hrMad10Char.writeValue(hrmad10);  // Using HRMAD for 10-second window
          hrMad30Char.writeValue(hrmad30);  // Using HRMAD for 30-second window
          hrMad60Char.writeValue(hrmad60);  // Using HRMAD for 60-second window
        }
      }

      delay(sampleWindow);  // Sample at 100Hz or 10ms interval
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

// Function to calculate HRV (Root Mean Square of Successive Differences - RMSSD)
float calculateHRV(int rrIntervals[]) {
  int sumSquaredDiffs = 0;
  for (int i = 1; i < numRRIntervals; i++) {
    int diff = rrIntervals[i] - rrIntervals[i - 1];
    sumSquaredDiffs += diff * diff;
  }
  return sqrt(sumSquaredDiffs / (numRRIntervals - 1));  // Return RMSSD value
}


// Function to update HR buffer with the latest heart rate
void updateHRBuffer(int* buffer, int bufferSize, int& index, int& bufferCount, int heartRate) {
  buffer[index] = heartRate;
  index = (index + 1) % bufferSize;
  if (bufferCount < bufferSize) bufferCount++;
}

// Function to calculate HRMAD for a given buffer
float calculateHRMAD(int* buffer, int bufferCount) {
  if (bufferCount == 0) return 0;

  // Calculate the moving average
  float sum = 0;
  for (int i = 0; i < bufferCount; i++) {
    sum += buffer[i];
  }
  float movingAverage = sum / bufferCount;

  // Calculate the absolute deviations from the moving average
  float deviationSum = 0;
  for (int i = 0; i < bufferCount; i++) {
    deviationSum += abs(buffer[i] - movingAverage);
  }

  // Return the mean of absolute deviations
  return deviationSum / bufferCount;
}
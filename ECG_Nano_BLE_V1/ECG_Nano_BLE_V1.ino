#include <Arduino.h>
#include <ArduinoBLE.h>

#define HEART_SERVICE_UUID        "0000180d-0000-1000-8000-00805f9b34fb"
#define HEART_RATE_UUID          "00002b90-0000-1000-8000-00805f9b34fb"
#define HRV_UUID                 "00002b91-0000-1000-8000-00805f9b34fb"
#define HRMAD10_UUID            "00002b92-0000-1000-8000-00805f9b34fb"
#define HRMAD30_UUID            "00002b93-0000-1000-8000-00805f9b34fb"
#define HRMAD60_UUID            "00002b94-0000-1000-8000-00805f9b34fb"
#define ECG_UUID                "00002b95-0000-1000-8000-00805f9b34fb"
#define LEADS_UUID                "00002b96-0000-1000-8000-00805f9b34fb"

BLEService heartService(HEART_SERVICE_UUID);

// Define characteristics
BLEFloatCharacteristic heartRateChar(HEART_RATE_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrvChar(HRV_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrMad10Char(HRMAD10_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrMad30Char(HRMAD30_UUID, BLERead | BLENotify);
BLEFloatCharacteristic hrMad60Char(HRMAD60_UUID, BLERead | BLENotify);
BLEFloatCharacteristic ecgChar(ECG_UUID, BLERead | BLENotify);
BLEFloatCharacteristic leadsChar(LEADS_UUID, BLERead | BLENotify);

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
  pinMode(loPlusPin, INPUT);
  pinMode(loMinusPin, INPUT);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  Serial.println(BLE.address());

  // Set the local name and advertised service
  BLE.setLocalName("Safit");
  BLE.setAdvertisedService(heartService);

  // Add all characteristics to the service
  heartService.addCharacteristic(heartRateChar);
  heartService.addCharacteristic(hrvChar);
  heartService.addCharacteristic(hrMad10Char);
  heartService.addCharacteristic(hrMad30Char);
  heartService.addCharacteristic(hrMad60Char);
  heartService.addCharacteristic(ecgChar);
  heartService.addCharacteristic(leadsChar);

  // Set initial values for characteristics
  heartRateChar.writeValue(0.0);
  hrvChar.writeValue(0.0);
  hrMad10Char.writeValue(0.0);
  hrMad30Char.writeValue(0.0);
  hrMad60Char.writeValue(0.0);
  ecgChar.writeValue(0.0);
  leadsChar.writeValue(0.0);

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
      bool leadOffDetected = digitalRead(loPlusPin) == HIGH || digitalRead(loMinusPin) == HIGH;

      if (leadOffDetected) {
        leadsChar.writeValue(float(1.0));
        continue;
      } else {
        leadsChar.writeValue(float(0.0));
      }
      
      int ecgValue = analogRead(ecgPin);

      if (ecgValue > threshold) {
        unsigned long currentTime = millis();
        beatInterval = currentTime - lastBeatTime;
        
        if (beatInterval > 300) {
          lastBeatTime = currentTime;
          heartRate = 60000 / beatInterval;
          
          rrIntervals[rrIndex] = beatInterval;
          rrIndex = (rrIndex + 1) % numRRIntervals;
          
          // Update buffers
          updateHRBuffer(hrBuffer10, bufferSize10, index10, bufferCount10, heartRate);
          updateHRBuffer(hrBuffer30, bufferSize30, index30, bufferCount30, heartRate);
          updateHRBuffer(hrBuffer60, bufferSize60, index60, bufferCount60, heartRate);

          // Calculate metrics
          float hrv = calculateHRV(rrIntervals);
          float hrmad10 = calculateHRMAD(hrBuffer10, bufferCount10);
          float hrmad30 = calculateHRMAD(hrBuffer30, bufferCount30);
          float hrmad60 = calculateHRMAD(hrBuffer60, bufferCount60);

          // Debug print
          Serial.print("HR: "); Serial.print(heartRate);
          Serial.print(" HRV: "); Serial.print(hrv);
          Serial.print(" ECG: "); Serial.println(ecgValue);

          // Write values to characteristics - note we're sending raw values now
          Serial.println("\n--- Write Results ---");
          Serial.print("Heart Rate Write: "); 
          Serial.println(heartRateChar.writeValue(float(heartRate)) ? "Success" : "Failed");
          
          Serial.print("HRV Write: ");
          Serial.println(hrvChar.writeValue(float(hrv)) ? "Success" : "Failed");
          
          Serial.print("HRMAD10 Write: ");
          Serial.println(hrMad10Char.writeValue(float(hrmad10)) ? "Success" : "Failed");
          
          Serial.print("HRMAD30 Write: ");
          Serial.println(hrMad30Char.writeValue(float(hrmad30)) ? "Success" : "Failed");
          
          Serial.print("HRMAD60 Write: ");
          Serial.println(hrMad60Char.writeValue(float(hrmad60)) ? "Success" : "Failed");
          
          Serial.print("ECG Write: ");
          Serial.println(ecgChar.writeValue(float(ecgValue)) ? "Success" : "Failed");
          Serial.println("-------------------\n");
        }
      }

      delay(sampleWindow);  // 100Hz sampling rate
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
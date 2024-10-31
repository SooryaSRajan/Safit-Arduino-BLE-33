#include <Arduino.h>

// Pin definition
const int ecgPin = A0; // Analog pin where ECG signal is read

// Variables for signal processing
unsigned long lastBeatTime = 0;   // Stores the time of the last heartbeat
int beatInterval = 0;             // Time between beats (R-R interval)
int heartRate = 0;                // Calculated heart rate in bpm
const int sampleWindow = 10;      // Sample window for analog reads
int threshold = 652;              // ECG signal threshold to detect R-peak
const int numRRIntervals = 10;    // Store last 10 R-R intervals for HRV

int rrIntervals[numRRIntervals];  // Array to store R-R intervals
int rrIndex = 0;                  // Index for the array

void setup() {
  Serial.begin(9600);
  pinMode(ecgPin, INPUT);
}

void loop() {
  int ecgValue = analogRead(ecgPin);  // Read ECG signal from AD8232
     // Serial.print("ADC Value11: ");
    // Serial.println(ecgValue);
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
      
      // Calculate and print HRV and HRMAD
      float hrv = calculateHRV(rrIntervals);
      float hrmad = calculateHRMAD(rrIntervals);
      Serial.print("HRV: ");
      Serial.println(hrv);
      Serial.print("HRMAD: ");
      Serial.println(hrmad);
    }
  }

  delay(sampleWindow);  // Sample at 100Hz or 10ms interval
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

// Function to calculate HRMAD (Mean Absolute Deviation of R-R intervals)
float calculateHRMAD(int rrIntervals[]) {
  float sum = 0;
  for (int i = 0; i < numRRIntervals; i++) {
    sum += rrIntervals[i];
  }
  float meanRR = sum / numRRIntervals;
  
  float mad = 0;
  for (int i = 0; i < numRRIntervals; i++) {
    mad += abs(rrIntervals[i] - meanRR);
  }
  return mad / numRRIntervals;
}
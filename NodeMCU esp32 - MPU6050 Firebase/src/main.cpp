#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoFFT.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// ==== Configuration Constants ====
#define SAMPLE_RATE 1000          // Hz
const uint16_t NUM_SAMPLES = 256; // Must be power of 2 for FFT

// WiFi credentials
const char *ssid = "SUMMERLAPTOP";
const char *password = "1234567890azj";

// ThingSpeak configuration
const char *host = "api.thingspeak.com";
const int httpPort = 80;
const unsigned long channelID = 2985325;
const String writeApiKey = "3DLQHR5FFVDV8RL1";
WiFiClient client;

// dummy ======================================================================================
float vx = 0.0, vy = 0.0, vz = 0.0;

// ==== Sensor and FFT Setup ====
Adafruit_MPU6050 mpu;

// rms@peak
float rmsX = 0, rmsY = 0, rmsZ = 0;
// frequency@peak
float pfX = 0, pfY = 0, pfZ = 0;

// FFT buffers
double vRealX[NUM_SAMPLES], vImagX[NUM_SAMPLES];
double vRealY[NUM_SAMPLES], vImagY[NUM_SAMPLES];
double vRealZ[NUM_SAMPLES], vImagZ[NUM_SAMPLES];

// FFT objects for each axis
ArduinoFFT<double> FFT_X(vRealX, vImagX, NUM_SAMPLES, SAMPLE_RATE);
ArduinoFFT<double> FFT_Y(vRealY, vImagY, NUM_SAMPLES, SAMPLE_RATE);
ArduinoFFT<double> FFT_Z(vRealZ, vImagZ, NUM_SAMPLES, SAMPLE_RATE);

// Threshold values for fault detection
const float RMS_THRESHOLD = 4;        // RMS warning
const float KURTOSIS_THRESHOLD = 6.0; // Kurtosis > 6.0 indicates impulsive vibration

// Function declarations
void analyzeAndStore(double *real, double *imag, ArduinoFFT<double> &fft, const char *label);

// ==== Setup ====
void setup()
{
  Serial.begin(115200);
  delay(100);

  // Initialize MPU6050
  if (!mpu.begin())
  {
    Serial.println("MPU6050 not detected. Halting.");
    while (1)
      delay(10);
  }

  // Set accelerometer range to ±8g (adjustable)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to ±");
  Serial.println(mpu.getAccelerometerRange() == MPU6050_RANGE_8_G ? "8G" : "Unknown");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Thingspeak
  ThingSpeak.begin(client);

  Serial.println("Setup complete. Starting loop...");
}

// ==== Main Loop ====
void loop()
{
  sensors_event_t a, g, temp;

  // Collect samples for FFT
  float zSum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    vRealX[i] = a.acceleration.x;
    vRealY[i] = a.acceleration.y;
    vRealZ[i] = a.acceleration.z;
    zSum += vRealZ[i];

    vImagX[i] = vImagY[i] = vImagZ[i] = 0;

    delayMicroseconds(1000000 / SAMPLE_RATE); // 1kHz
  }

  // Remove DC offset from Z
  float zMean = zSum / NUM_SAMPLES;
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    vRealZ[i] -= zMean;
  }

  // Analyze each axis and send results
  analyzeAndStore(vRealX, vImagX, FFT_X, "X");
  analyzeAndStore(vRealY, vImagY, FFT_Y, "Y");
  analyzeAndStore(vRealZ, vImagZ, FFT_Z, "Z");

  // dummy ======================================================================================
  // vx++;
  // vy++;
  // vz++;

  // Serial.print("rmsX: ");
  // Serial.println(rmsX);
  // Serial.print("rmsY: ");
  // Serial.println(rmsY);
  // Serial.print("rmsZ: ");
  // Serial.println(rmsZ);

  // Set ThingSpeak fields
  ThingSpeak.setField(1, rmsX);
  ThingSpeak.setField(2, rmsY);
  ThingSpeak.setField(3, rmsZ);
  ThingSpeak.setField(4, pfX);
  ThingSpeak.setField(5, pfY);
  ThingSpeak.setField(6, pfZ);

  // Send all data in one entry
  int x = ThingSpeak.writeFields(channelID, writeApiKey.c_str());

  if (x == 200)
  {
    Serial.println("Data sent to ThingSpeak successfully!");
  }
  else
  {
    Serial.print("Problem sending data. HTTP error code: ");
    Serial.println(x);
  }

  Serial.println("--------------------------------------------------\n");
  delay(15000); // Wait before next reading set, as firebase read interval is 15s for free account
}

// ==== Vibration Analysis + ThingSpeak Send ====
void analyzeAndStore(double *real, double *imag, ArduinoFFT<double> &fft, const char *label)
{
  float sumSq = 0, mean = 0, variance = 0, fourthMoment = 0;

  // Time-domain RMS
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    sumSq += real[i] * real[i];
    mean += real[i];
  }
  float rms_full = sqrt(sumSq / NUM_SAMPLES);
  mean /= NUM_SAMPLES;

  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    float diff = real[i] - mean;
    variance += diff * diff;
    fourthMoment += pow(diff, 4);
  }
  variance /= NUM_SAMPLES;
  fourthMoment /= NUM_SAMPLES;

  float kurtosis = (variance > 0) ? (fourthMoment / (variance * variance)) : 0;

  // Frequency-domain analysis
  fft.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  fft.compute(FFT_FORWARD);
  fft.complexToMagnitude();

  // peak
  double peak = 0;
  int peakIndex = -1;
  int maxBin = (int)(100.0 * NUM_SAMPLES / SAMPLE_RATE); // limits the peak search to ~100 Hz

  for (int i = 1; i < maxBin; i++)
  {
    if (real[i] > 0.01 && real[i] > peak) // skips FFT bins with very low magnitude (likely noise)
    {
      peak = real[i];
      peakIndex = i;
    }
  }
  // If no peak is found (i.e. all bins are ≤ 0.01), peakIndex remains -1 and frequency is set to 0.0
  double freq = (peakIndex >= 0) ? (peakIndex * SAMPLE_RATE) / NUM_SAMPLES : 0.0;
  double rms_at_peak = (2.0 * peak) / (NUM_SAMPLES * sqrt(2.0)); // RMS of sinusoid at dominant freq

  Serial.print("Axis ");
  Serial.print(label);
  Serial.print(" | Time-RMS: ");
  Serial.print(rms_full, 3);
  Serial.print(" g | Kurtosis: ");
  Serial.print(kurtosis, 2);
  Serial.print(" | Peak Freq: ");
  Serial.print(freq, 1);
  Serial.print(" Hz | RMS@Peak: ");
  Serial.print(rms_at_peak, 3);
  Serial.println(" g");

  if (label == "X")
  {
    rmsX = rms_at_peak + vx;
    pfX = freq;
  }
  else if (label == "Y")
  {
    rmsY = rms_at_peak + vy;
    pfY = freq;
  }
  else if (label == "Z")
  {
    rmsZ = rms_at_peak + vz;
    pfZ = freq;
  }
}
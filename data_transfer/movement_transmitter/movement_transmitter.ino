/**
XIAO1 MAC Address: 24:ec:4a:ce:4f:7c
XIAO2 MAC Address: 58:8c:81:9e:99:e0
ESP32C6 WROOM 1 MAX Address: 58:8c:81:3b:cc:78
**/

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include <esp_now.h>
#include <WiFi.h>

uint8_t recvAddress[] = {0x58, 0x8c, 0x81, 0x3b, 0xcc, 0x78};
esp_now_peer_info_t receiverInfo;

typedef struct send_message
{
  float ax;
  float ay;
  float az;
} send_message;

send_message reading; // Variable to send data

char receive_char; // Receive signals from ESP WROOM

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&receive_char, incomingData, sizeof(receive_char));
}

esp_err_t send_result;

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

#define FAST_MODE

#define BNO08X_RESET 9

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  sh2_SensorId_t reportType_accel = SH2_LINEAR_ACCELERATION;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  sh2_SensorId_t reportType_accel = SH2_LINEAR_ACCELERATION;
  long reportIntervalUs = 5000;
#endif

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  
  if (!bno08x.enableReport(reportType_accel, report_interval)) {
    Serial.println("Could not enable linear acceleration report");
  }
}

unsigned long dt = 0;
unsigned int time_step = 10000; // 10000 microseconds = 10 milliseconds
unsigned int state_mach = 0;

void setup(void) {

  Serial.begin(500000);
  Serial.println("Starting");

  Serial.println("Adafruit BNO08x test!");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Config ESP WROOM connection
  memcpy(receiverInfo.peer_addr, recvAddress, 6);
  receiverInfo.channel = 0;
  receiverInfo.encrypt = false;

  if (esp_now_add_peer(&receiverInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESPNow setup complete");

  Wire.begin(6, 7); // Configure I2C pins (GPIO)

  // Try to initialize!
  if (!bno08x.begin_I2C(0x4B)) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  // if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  delay(100);
}

float ax, ay, az; // To store the latest acceleration data
float r, i, j, k; // To store the latest quaternion data

void loop()
{
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  // --- Part 1: Continuously poll for new sensor data ---
  // This section continuously updates the global variables with the latest
  // data as soon as it's available from the sensor.
  if (bno08x.getSensorEvent(&sensorValue)) {
    // Check the sensorId to see which report we received and update
    // the appropriate variables.
    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        ax = sensorValue.un.linearAcceleration.x;
        ay = sensorValue.un.linearAcceleration.y;
        az = sensorValue.un.linearAcceleration.z;
        break;
      // case SH2_GYRO_INTEGRATED_RV:
      //   // *** CORRECTED: Use .rotationVector to match the enabled report ***
      //   r = sensorValue.un.rotationVector.real;
      //   i = sensorValue.un.rotationVector.i;
      //   j = sensorValue.un.rotationVector.j;
      //   k = sensorValue.un.rotationVector.k;
      //   break;
    }
  }

  switch (state_mach) {
    case 0:
      reading.ax = -1;
      reading.ay = -1;
      reading.az = -1;

      send_result = esp_now_send(recvAddress, (uint8_t *) &reading, sizeof(reading));

      if (send_result != ESP_OK)
      {
        Serial.println("Send failed");
      }

      if (receive_char == 'O')
      {
        state_mach = 1;
      }
      
      break;

    case 1:
      if (receive_char == 'S')
      {
        delay(10);
        state_mach = 2;
        dt = micros(); // Reset timer for the data streaming loop
      }
      break;

    case 2:
      // Check if it's time to send the next data packet
      if (micros() - dt >= time_step) {
        dt = micros(); // Update dt for the next interval

        // Send the latest complete data packet stored in our global variables
        reading.ax = ax;
        reading.ay = ay;
        reading.az = az;

        send_result = esp_now_send(recvAddress, (uint8_t *) &reading, sizeof(reading));

        if (send_result != ESP_OK)
        {
          Serial.println("Send failed");
        }
      }

      if (receive_char == 'K')
      {
        state_mach = 1;
      }
      break;
  }
}
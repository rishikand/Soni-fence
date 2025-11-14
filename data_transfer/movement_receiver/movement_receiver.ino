/**
XIAO1 MAC Address: 24:ec:4a:ce:4f:7c
XIAO2 MAC Address: 58:8c:81:9e:99:e0
ESP32C6 WROOM 1 MAX Address: 58:8c:81:3b:cc:78
**/

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include <esp_now.h>
#include <WiFi.h>

uint8_t xiao1Address[] = {0x24, 0xec, 0x4a, 0xce, 0x4f, 0x7c};
uint8_t xiao2Address[] = {0x58, 0x8c, 0x81, 0x9e, 0x99, 0xe0};

esp_now_peer_info_t xiao1Info;
esp_now_peer_info_t xiao2Info;

typedef struct receive_message
{
  float ax;
  float ay;
  float az;
} receive_message;

receive_message xiao1reading;
receive_message xiao2reading;

bool send_data;

char espSignal;
char serialSignal;
bool imuConnected;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  const uint8_t *senderMac = info->src_addr;

  if (memcmp(senderMac, xiao1Address, 6) == 0)
  {
    memcpy(&xiao1reading, incomingData, sizeof(receive_message));
  }
  else if (memcmp(senderMac, xiao2Address, 6) == 0)
  {
    memcpy(&xiao2reading, incomingData, sizeof(receive_message));
  }

  if (send_data == true)
  {
    Serial.print("ax1: ");
    Serial.println(xiao1reading.ax);
    Serial.print("ay1: ");
    Serial.println(xiao1reading.ay);
    Serial.print("az1: ");
    Serial.println(xiao1reading.az);

    Serial.print("ax2: ");
    Serial.println(xiao2reading.ax);
    Serial.print("ay2: ");
    Serial.println(xiao2reading.ay);
    Serial.print("az2: ");
    Serial.println(xiao2reading.az);
  }
}

esp_err_t xiao1_send_result;
esp_err_t xiao2_send_result;

unsigned int state_mach = 0;

void setup(void) {

  Serial.begin(500000);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  memcpy(xiao1Info.peer_addr, xiao1Address, 6);
  xiao1Info.channel = 0;
  xiao1Info.encrypt = false;

  if (esp_now_add_peer(&xiao1Info) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(xiao2Info.peer_addr, xiao2Address, 6);
  xiao2Info.channel = 0;
  xiao2Info.encrypt = false;

  if (esp_now_add_peer(&xiao2Info) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESPNow setup complete");

  delay(100);
}

void loop()
{
  switch (state_mach) {
    case 0:
      imuConnected = true;

      if (xiao1reading.ax == -1 && xiao1reading.ay == -1 && xiao1reading.az == -1)
      {
        Serial.println("xiao1Connected");
      }
      else
      {
        imuConnected = false;
      }

      if (xiao2reading.ax == -1 && xiao2reading.ay == -1 && xiao2reading.az == -1)
      {
        Serial.println("xiao2Connected");
      }
      else
      {
        imuConnected = false;
      }

      if (imuConnected)
      {
        state_mach = 1;
        espSignal = 'O';
        
        xiao1_send_result = esp_now_send(xiao1Address, (uint8_t *) &espSignal, sizeof(espSignal));
        xiao2_send_result = esp_now_send(xiao2Address, (uint8_t *) &espSignal, sizeof(espSignal));

        if (xiao1_send_result != ESP_OK || xiao2_send_result != ESP_OK)
          Serial.println("O send failed");
      }

      break;

    case 1:
      if (Serial.available() != 0)
      {
        char serialSignal = Serial.read();

        if (serialSignal == 'K')
        {
          state_mach = 2;
        }
      }

      break;

    case 2:
      espSignal = 'S';

      xiao1_send_result = esp_now_send(xiao1Address, (uint8_t *) &espSignal, sizeof(espSignal));
      xiao2_send_result = esp_now_send(xiao2Address, (uint8_t *) &espSignal, sizeof(espSignal));

      if (xiao1_send_result == ESP_OK && xiao2_send_result == ESP_OK)
      {
        Serial.println("Starting data recording");
        state_mach = 3;
      }
      else if (xiao1_send_result != xiao2_send_result)
      {
        if (xiao1_send_result)
        {
          Serial.println("XIAO2 failed to start, attempting data collection stop at xiao1");

          espSignal = 'K';
          xiao1_send_result = esp_now_send(xiao1Address, (uint8_t *) &espSignal, sizeof(espSignal));

          if (xiao1_send_result == ESP_OK)
          {
            Serial.println("Successfully stopped");
          }
          else
          {
            Serial.println("Stop failed");
          }
        }
        else
        {
          Serial.println("XIAO1 failed to start, attempting data collection stop at xiao2");

          espSignal = 'K';
          xiao2_send_result = esp_now_send(xiao2Address, (uint8_t *) &espSignal, sizeof(espSignal));

          if (xiao2_send_result == ESP_OK)
          {
            Serial.println("Successfully stopped");
          }
          else
          {
            Serial.println("Stop failed");
          }
        }
      }

      break;

    case 3:
      send_data = true;

      if (Serial.available() != 0)
      {
        serialSignal = Serial.read();

        if (serialSignal == 'K')
        {
          state_mach = 4;
        }
      }
      break;
    
    case 4:
      send_data = false;
      Serial.println("Data collection stopped");
      
      break;
  }
}
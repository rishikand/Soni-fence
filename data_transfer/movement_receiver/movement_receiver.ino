/**
XIAO1 MAC Address: 24:ec:4a:ce:4f:7c
XIAO2 MAC Address: 58:8c:81:9e:99:e0
ESP32C6 WROOM 1 MAC Address: 58:8c:81:3b:cc:78
**/

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

#include <Adafruit_TLV320DAC3100.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP_I2S.h>

Adafruit_TLV320DAC3100 codec;
I2SClass i2s;

#define pBCLK 4    // BITCLOCK - I2S clock
#define pWS   7    // LRCLOCK - Word select
#define pDOUT 5    // DATA - I2S data
#define TLV_RESET 10

const int ZUPT_ACC_THRESHOLD = 0.08 * 0.08 * 0.08;
const int ZUPT_TIMEOUT = 100; // milliseconds
bool zupt1_state = false;
bool zupt2_state = false;
bool zupt1_timer = 0;
bool zupt2_timer = 0;

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

float vel1;
float vel2;

receive_message xiao1reading;
receive_message xiao2reading;
//Copied from constants sound file
const int sampleRate = 16000; // 16 KHz
const int16_t max_amplitude = 5000; // Max volume for the tone

// --- Carrier (Audible Tone) ---
volatile double current_frequency1 = 0.0; // Start silent
volatile double current_frequency2 = 0.0; // Start silent
volatile double current_phase1 = 0.0;
volatile double current_phase2 = 0.0;
volatile double phase_step1 = 0.0;
volatile double phase_step2 = 0.0;

// --- Modulator (LFO for Amplitude "Tremolo") ---
volatile double mod_frequency = 0.0; 
volatile double mod_phase = 0.0;
volatile double mod_phase_step = 0.0;
volatile double mod_depth = 0.0;

enum LfoShape { LFO_SINE }; 
volatile LfoShape current_lfo_shape = LFO_SINE;

#define I2S_BUFFER_LEN_SAMPLES 256
int32_t i2s_buffer[I2S_BUFFER_LEN_SAMPLES];


//Helper function to map a value from one range to another.
 
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


 //Helper function to halt on critical audio errors.
 
void audio_halt(const char *message) {
  Serial.println(message);
  while (1)
    yield();
}

bool send_data; // Says whether or not data should be logged

char espSignal; // Signals to send to xiao1 and xiao2 to control logic flow
char serialSignal; // Signals from serial monitor (S to start, K to stop)
bool imuConnected; // Initial check for if receiver has received connection signal from both xiao1 and xiao2

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  //extra safety test to determine if the length of the data is same as the bytes in the xiao structure
  if (len != sizeof(receive_message)) {
    return; // Not our data, ignore it.
  }

  const uint8_t *senderMac = info->src_addr;

  // ID board based on mac address
  if (memcmp(senderMac, xiao1Address, 6) == 0)
  {
    memcpy(&xiao1reading, incomingData, sizeof(receive_message));
  }
  else if (memcmp(senderMac, xiao2Address, 6) == 0)
  {
    memcpy(&xiao2reading, incomingData, sizeof(receive_message));
  }
} // revemove this brace if debugging
  // Send data if send_data is true (Debugging)
//   if (send_data == true)
//   {
//     Serial.print("ax1: ");
//     Serial.println(xiao1reading.ax);
//     Serial.print("ay1: ");
//     Serial.println(xiao1reading.ay);
//     Serial.print("az1: ");
//     Serial.println(xiao1reading.az);

//     Serial.print("ax2: ");
//     Serial.println(xiao2reading.ax);
//     Serial.print("ay2: ");
//     Serial.println(xiao2reading.ay);
//     Serial.print("az2: ");
//     Serial.println(xiao2reading.az);
//   }
// }

// Variables to store result of sending data
esp_err_t xiao1_send_result;
esp_err_t xiao2_send_result;

unsigned int state_mach = 0;

void setup(void) {

  Serial.begin(500000);
  Wire.setPins(21, 22);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // xiao1 config
  memcpy(xiao1Info.peer_addr, xiao1Address, 6);
  xiao1Info.channel = 0;
  xiao1Info.encrypt = false;

  if (esp_now_add_peer(&xiao1Info) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // xiao2 config
  memcpy(xiao2Info.peer_addr, xiao2Address, 6);
  xiao2Info.channel = 0;
  xiao2Info.encrypt = false;

  if (esp_now_add_peer(&xiao2Info) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESPNow setup complete");

  Serial.println("Initializing I2S & DAC...");
  i2s.setPins(pBCLK, pWS, pDOUT);
  if (!i2s.begin(I2S_MODE_STD, sampleRate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH))
  {
    audio_halt("Failed to initialise I2S.");
  }

  pinMode(TLV_RESET, OUTPUT);
  digitalWrite(TLV_RESET, LOW);
  delay(100);
  digitalWrite(TLV_RESET, HIGH);

  if (!codec.begin()) { audio_halt("Failed to initialize codec!"); }
  delay(10);
  if (!codec.setCodecInterface(TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_16)) { audio_halt("Failed to configure codec interface!"); }
  if (!codec.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL) || !codec.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK)) { audio_halt("Failed to configure codec clocks!"); }
  if (!codec.setPLLValues(1, 2, 32, 0)) { audio_halt("Failed to configure PLL values!"); }
  if (!codec.setNDAC(true, 8) || !codec.setMDAC(true, 2)) { audio_halt("Failed to configure DAC dividers!"); }
  if (!codec.powerPLL(true)) { audio_halt("Failed to power up PLL!"); }
  if (!codec.setDACDataPath(true, true, TLV320_DAC_PATH_NORMAL, TLV320_DAC_PATH_NORMAL, TLV320_VOLUME_STEP_1SAMPLE)) { audio_halt("Failed to configure DAC data path!"); }
  if (!codec.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER, TLV320_DAC_ROUTE_MIXER, false, false, false, false)) { audio_halt("Failed to configure DAC routing!"); }
  if (!codec.setDACVolumeControl(false, false, TLV320_VOL_INDEPENDENT) || !codec.setChannelVolume(false, 100) || !codec.setChannelVolume(true, 100)) { audio_halt("Failed to configure DAC volumes!"); }
  if (!codec.configureHeadphoneDriver(true, true, TLV320_HP_COMMON_1_35V, false) || !codec.configureHPL_PGA(0, true) || !codec.configureHPR_PGA(0, true) || !codec.setHPLVolume(true, 100) || !codec.setHPRVolume(true, 100)) { audio_halt("Failed to configure headphone outputs!"); }
  if (!codec.enableSpeaker(true) || !codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true) || !codec.setSPKVolume(true, 0)) { audio_halt("Failed to configure speaker output!"); }
  if (!codec.configMicBias(false, true, TLV320_MICBIAS_AVDD) || !codec.setHeadsetDetect(true) || !codec.setInt1Source(true, true, false, false, false, false) || !codec.setGPIO1Mode(TLV320_GPIO1_INT1)) { audio_halt("Failed to configure headset detect"); }
  Serial.println("TLV config done!");

  delay(100);
}

void loop()
{
  switch (state_mach) {
    case 0:
      // Ensure receiving from xiao1 && xiao2
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
        espSignal = 'O'; // Signal xiao1 and xiao2 to start sending IMU data

        xiao1_send_result = esp_now_send(xiao1Address, (uint8_t *) &espSignal, sizeof(espSignal));
        xiao2_send_result = esp_now_send(xiao2Address, (uint8_t *) &espSignal, sizeof(espSignal));

        if (xiao1_send_result != ESP_OK || xiao2_send_result != ESP_OK)
        {
          Serial.println("O send failed");
        }
        else
        {
          state_mach = 1;
        }
      }

      break;

    case 1:
      if (Serial.available() != 0)
      {
        char serialSignal = Serial.read();

        if (serialSignal == 'K') // Signal start from serial
        {
          state_mach = 2;
        }
      }

      break;

    case 2:
      // Send start signal
      espSignal = 'S';

      xiao1_send_result = esp_now_send(xiao1Address, (uint8_t *) &espSignal, sizeof(espSignal));
      xiao2_send_result = esp_now_send(xiao2Address, (uint8_t *) &espSignal, sizeof(espSignal));

      // Check if both xiao1 and xiao2 are sending data, otherwise, stop the one that did not run
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
      {
        send_data = true;

        // 1. Check for Stop Signal
        if (Serial.available() != 0)
        {
          serialSignal = Serial.read();

          if (serialSignal == 'K') // Signal stop from serial
          {
            state_mach = 4;
            current_frequency1 = 0.0; // Mute sound immediately
            current_frequency2 = 0.0; // Mute sound immediately
            mod_frequency = 0.0;

            break; // Exit case 3: and go to case 4 next loop
          }
        }

        // Use z-axis as forward for xiao1 (on right foot), and use y-axis as forward for xiao2 (on left foot)
        float acc1 = xiao1reading.az;
        float acc2 = xiao2reading.ay;

        float mag_acc1 = (xiao1reading.ax * xiao1reading.ax) + (xiao1reading.ay * xiao1reading.ay) + (xiao1reading.az * xiao1reading.az);

        if (mag_acc1 < ZUPT_ACC_THRESHOLD)
        {
          if (!zupt1_state)
          {
            zupt1_timer = millis();
            zupt1_state = true;
          }

          if (millis() - zupt1_timer > ZUPT_TIMEOUT)
          {
            vel1 = 0;
          }
        }
        else
        {
          zupt1_state = false;
          vel1 += acc1 * 0.01; // m/s^2 * s = m/s
        }

        float mag_acc2 = (xiao2reading.ax * xiao2reading.ax) + (xiao2reading.ay * xiao2reading.ay) + (xiao2reading.az * xiao2reading.az);

        if (mag_acc2 < ZUPT_ACC_THRESHOLD)
        {
          if (!zupt2_state)
          {
            zupt2_timer = millis();
            zupt2_state = true;
          }

          if (millis() - zupt2_timer > ZUPT_TIMEOUT)
          {
            vel2 = 0;
          }
        }
        else
        {
          zupt2_state = false;
          vel2 += acc2 * 0.01; // m/s^2 * s = m/s
        }

        Serial.print("vel1: ");
        Serial.print(vel1);
        Serial.print(", vel2: ");
        Serial.println(vel2);

        // 1. Map Accelerations to Frequencies
        current_frequency1 = mapfloat(vel1, -15.0, 15.0, 200.0, 600.0); // Right Channel (vel1)
        current_frequency2 = mapfloat(vel2, -15.0, 15.0, 200.0, 600.0); // Left Channel (vel2)

        // 2. Update Phase Increments (Required to change pitch)
        phase_step1 = (current_frequency1 * 2.0 * M_PI) / sampleRate;
        phase_step2 = (current_frequency2 * 2.0 * M_PI) / sampleRate;

        // 4. Audio Synthesis Loop
        for (int i = 0; i < I2S_BUFFER_LEN_SAMPLES; i++) {
          int16_t sample_right = 0;
          int16_t sample_left = 0;

          // --- MODULATOR (LFO) --- 
          // Shared LFO for both channels to keep effects synchronized
          double mod_value = sin(mod_phase);
          double lfo_gain = 1.0 - (mod_depth * (1.0 + mod_value) / 2.0);
          
          // Update LFO Phase
          mod_phase += mod_phase_step;
          if (mod_phase >= 2.0 * M_PI) mod_phase -= 2.0 * M_PI;

          // --- RIGHT CHANNEL (vel1) ---
          if (current_frequency1 > 0.0) {
              double carrier_sample1 = sin(current_phase1);
              double current_amplitude1 = max_amplitude * lfo_gain;
              sample_right = (int16_t)(current_amplitude1 * carrier_sample1);

              // Update Phase 1
              current_phase1 += phase_step1;
              if (current_phase1 >= 2.0 * M_PI) current_phase1 -= 2.0 * M_PI;
          }

          // --- LEFT CHANNEL (vel2) ---
          if (current_frequency2 > 0.0) {
              double carrier_sample2 = sin(current_phase2);
              double current_amplitude2 = max_amplitude * lfo_gain;
              sample_left = (int16_t)(current_amplitude2 * carrier_sample2);

              // Update Phase 2
              current_phase2 += phase_step2;
              if (current_phase2 >= 2.0 * M_PI) current_phase2 -= 2.0 * M_PI;
          }

          // Combine into a 32-bit stereo sample
          // Standard I2S packing: High Word = Left, Low Word = Right
          i2s_buffer[i] = ((int32_t)sample_left << 16) | (sample_right & 0xFFFF);
        }

        // 5. Write the entire buffer to the I2S DAC
        i2s.write((uint8_t*)i2s_buffer, I2S_BUFFER_LEN_SAMPLES * sizeof(int32_t)); // End of new scope
      
        break;
      }

    case 4:
      send_data = false;
      Serial.println("Data collection stopped");

      state_mach = 1;
      
      break;
  }
}


    // phase_step1 = (2.0 * M_PI * current_frequency1) / (double)sampleRate;
    // phase_step2 = (2.0 * M_PI * current_frequency2) / (double)sampleRate;

    // // A. Base Frequency (Pitch): Average Y-axis movement from both sensors.
    // float avg_y_move = (y1_move + y2_move) / 2.0;
    // current_frequency = mapfloat(avg_y_move, 0.0, 15.0, 200.0, 600.0); // 200Hz (low) to 600Hz (high)
    
    // // B. LFO Frequency (Tremolo Speed): mapped to both xiaos
    // float avg_x_move = (x1_move + x2_move) / 2.0;
    // mod_frequency = mapfloat(avg_x_move, 0.0, 15.0, 0.5, 20.0); // 0.5Hz (slow) to 20Hz (fast)

    // // C. LFO Depth (Tremolo Amount): Mapped to both xiao's z axis (front of shin)
    // float avg_z_move = (z1_move + z2_move) / 2.0;
    // mod_depth = mapfloat(avg_z_move, 0.0, 15.0, 0.0, 1.0); // 0.0 = no effect, 1.0 = full effect

    // // 3. Calculate phase steps based on new frequencies 
    // phase_step = (2.0 * M_PI * current_frequency) / (double)sampleRate;
    // mod_phase_step = (2.0 * M_PI * mod_frequency) / (double)sampleRate;
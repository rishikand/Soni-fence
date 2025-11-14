// SPDX-FileCopyrightText: 2016 Sandeep Mistry
// SPDX-FileCopyrightText: 2022 Earle F. Philhower, III
// SPDX-FileCopyrightText: 2023 Ladyada for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include <Adafruit_TLV320DAC3100.h>
#include <ESP_I2S.h>
#include <math.h>

Adafruit_TLV320DAC3100 codec;  // Create codec object

#define pBCLK 4   // BITCLOCK - I2S clock
#define pWS   7  // LRCLOCK - Word select
#define pDOUT 5  // DATA - I2S data
#define TLV_RESET 10

#define SINE_LUT_SIZE 100 // Number of samples in one cycle
int16_t sineWave[SINE_LUT_SIZE]; // Array to hold the pre-calculated sine wave

void halt(const char *message) {
  Serial.println(message);
  while (1)
    yield(); // Function to halt on critical errors
}

// Create I2S port
I2SClass i2s;

const int frequency = 440; // frequency of square wave in Hz
const int amplitude = 100; // amplitude of square wave
const int sampleRate = 16000; // 16 KHz is a good quality

const int halfWavelength = (sampleRate / frequency); // half wavelength of square wave

int16_t sample = amplitude; // current sample value
int count = 0;

void setup() {
  Serial.begin(115200);
  Wire.setPins(21, 22);

  // Initialize I2S peripheral
  Serial.println("Initializing I2S...");
  i2s.setPins(pBCLK, pWS, pDOUT);

  if (!i2s.begin(I2S_MODE_STD, sampleRate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH))
  {
    Serial.println("Failed to initialise I2S.");
  }
  
  while (!Serial) delay(10);
  Serial.println("\n\nTLV320DAC3100 Sine Tone Test");

  pinMode(TLV_RESET, OUTPUT);
  digitalWrite(TLV_RESET, LOW);
  delay(100);
  digitalWrite(TLV_RESET, HIGH);

  Serial.println("Init TLV DAC");
  if (!codec.begin()) {
    halt("Failed to initialize codec!");
  }
  delay(10);

  // Interface Control
  if (!codec.setCodecInterface(TLV320DAC3100_FORMAT_I2S,     // Format: I2S
                               TLV320DAC3100_DATA_LEN_16)) { // Length: 16 bits
    halt("Failed to configure codec interface!");
  }

  // Clock MUX and PLL settings
  if (!codec.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL) ||
      !codec.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK)) {
    halt("Failed to configure codec clocks!");
  }

  if (!codec.setPLLValues(1, 2, 32, 0)) { // P=2, R=2, J=32, D=0
    halt("Failed to configure PLL values!");
  }

  // DAC/ADC Config
  if (!codec.setNDAC(true, 8) || // Enable NDAC with value 8
      !codec.setMDAC(true, 2)) { // Enable MDAC with value 2
    halt("Failed to configure DAC dividers!");
  }

  if (!codec.powerPLL(true)) { // Power up the PLL
    halt("Failed to power up PLL!");
  }

  // DAC Setup
  if (!codec.setDACDataPath(true, true,                    // Power up both DACs
                            TLV320_DAC_PATH_NORMAL,        // Normal left path
                            TLV320_DAC_PATH_NORMAL,        // Normal right path
                            TLV320_VOLUME_STEP_1SAMPLE)) { // Step: 1 per sample
    halt("Failed to configure DAC data path!");
  }

  if (!codec.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER, // Left DAC to mixer
                                   TLV320_DAC_ROUTE_MIXER, // Right DAC to mixer
                                   false, false, false,    // No AIN routing
                                   false)) {               // No HPL->HPR
    halt("Failed to configure DAC routing!");
  }

  // DAC Volume Control
  if (!codec.setDACVolumeControl(
          false, false, TLV320_VOL_INDEPENDENT) || // Unmute both channels
      !codec.setChannelVolume(false, 100) ||        // Left DAC +0dB
      !codec.setChannelVolume(true, 100)) {         // Right DAC +0dB
    halt("Failed to configure DAC volumes!");
  }

  // Headphone and Speaker Setup
  if (!codec.configureHeadphoneDriver(
          true, true,                     // Power up both drivers
          TLV320_HP_COMMON_1_35V,         // Default common mode
          false) ||                       // Don't power down on SCD
      !codec.configureHPL_PGA(0, true) || // Set HPL gain, unmute
      !codec.configureHPR_PGA(0, true) || // Set HPR gain, unmute
      !codec.setHPLVolume(true, 100) ||     // Enable and set HPL volume
      !codec.setHPRVolume(true, 100)) {     // Enable and set HPR volume
    halt("Failed to configure headphone outputs!");
  }

  if (!codec.enableSpeaker(true) ||                // Dis/Enable speaker amp
      !codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, // Set gain to 6dB
                              true) ||             // Unmute
      !codec.setSPKVolume(true, 0)) { // Enable and set volume to 0dB
    halt("Failed to configure speaker output!");
  }

  if (!codec.configMicBias(false, true, TLV320_MICBIAS_AVDD) ||
      !codec.setHeadsetDetect(true) ||
      !codec.setInt1Source(true, true, false, false, false,
                           false) || // GPIO1 is detect headset or button press
      !codec.setGPIO1Mode(TLV320_GPIO1_INT1)) {
    halt("Failed to configure headset detect");
  }
  Serial.println("TLV config done!");

  Serial.println("Pre-calculating sine wave...");
  for (int i = 0; i < SINE_LUT_SIZE; i++) {
    double phase = 2.0 * M_PI * ((double)i / SINE_LUT_SIZE);
    // Use the scaled amplitude
    sineWave[i] = (int16_t)(amplitude * sin(phase));
  }

  uint8_t reg48_value = codec.readRegister(0x00, 0x30); // 0x30 is the address for Register 48

  // We are expecting bits to be set based on:
  // - setHeadsetDetect(true)
  // - setInt1Source (including headset detect)
  // - setGPIO1Mode(TLV320_GPIO1_INT1)
  
  // The expected value is not simple, but any non-zero value indicates settings took hold.
  // For the TLV320DAC3100, Register 48 (0x30) controls Headset/GPIO Configuration.
  // The critical bit is D7 (Headset Detect Enable) and D3-D0 for GPIO1 mode.

  Serial.print("🛠️ Diagnostic Check: Register 48 (Headset/GPIO) value: 0x");
  Serial.println(reg48_value, HEX);
}

void loop() {
  // 1. Get the current sample from the Look-Up Table
  int16_t current_sample = sineWave[count];

  // 2. Combine the Left and Right 16-bit samples into a single 32-bit word.
  // Left channel is in the upper 16 bits, Right channel in the lower 16 bits.
  int32_t stereo_sample = ((int32_t)current_sample << 16) | (current_sample & 0xFFFF);

  // 3. Write the combined 32-bit stereo sample ONCE.
  i2s.write(stereo_sample);

  // 4. Increment the LUT index and wrap around
  count++;
  if (count >= SINE_LUT_SIZE) {
    count = 0;
  }

    
}

// void loop() {
//   // Simple square wave logic
//   if (count < halfWavelength) {
//     // Keep the current sample value
//   } else {
//     // Flip the sign of the sample
//     sample = -sample; 
//     // Reset the count for the next half-wavelength
//     count = 0;
//   }

//   // Combine the Left and Right 16-bit samples into a single 32-bit word.
//   // Left channel is in the upper 16 bits, Right channel in the lower 16 bits.
//   int32_t stereo_sample = ((int32_t)sample << 16) | (sample & 0xFFFF);

//   // Write the combined 32-bit stereo sample ONCE.
//   i2s.write(stereo_sample);

//   // Increment the counter for the next sample
//   count++;
// }

// SPDX-FileCopyrightText: 2016 Sandeep Mistry
// SPDX-FileCopyrightText: 2022 Earle F. Philhower, III
// SPDX-FileCopyrightText: 2023 Ladyada for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include <Adafruit_TLV320DAC3100.h>
#include <ESP_I2S.h>
#include <math.h>

Adafruit_TLV320DAC3100 codec;  // Create codec object

#define pBCLK 4    // BITCLOCK - I2S clock
#define pWS   7    // LRCLOCK - Word select
#define pDOUT 5    // DATA - I2S data
#define TLV_RESET 10

// ---------------------------------------------------
// GLOBALS FOR TONE GENERATION
// ---------------------------------------------------
const int sampleRate = 16000; // 16 KHz
const int16_t max_amplitude = 5000; // Max volume for the tone

// --- Carrier (Audible Tone) ---
volatile double current_frequency = 0.0; // Start silent
volatile double current_phase = 0.0;
volatile double phase_step = 0.0;

// --- Modulator (LFO for Amplitude) ---
volatile double mod_frequency = 0.0; // Start with no effect
volatile double mod_phase = 0.0;
volatile double mod_phase_step = 0.0;

#define I2S_BUFFER_LEN_SAMPLES 256
int32_t i2s_buffer[I2S_BUFFER_LEN_SAMPLES]; 
// ---------------------------------------------------


void halt(const char *message) {
  Serial.println(message);
  while (1)
    yield();  // Function to halt on critical errors
}

// Create I2S port
I2SClass i2s;


// ---------------------------------------------------
// ✨ FUNCTIONS TO CONTROL FREQUENCIES ✨
// ---------------------------------------------------

/**
 * @brief Sets the new CARRIER (audible) frequency
 */
void setFrequency(double new_freq) {
  current_frequency = new_freq;
  phase_step = (2.0 * M_PI * current_frequency) / (double)sampleRate;
}

/**
 * @brief Sets the new MODULATOR (LFO) frequency
 */
void setModulation(double new_mod_freq) {
  mod_frequency = new_mod_freq;
  mod_phase_step = (2.0 * M_PI * mod_frequency) / (double)sampleRate;
}

/**
 * @brief Stops all sound
 */
void stopSound() {
  Serial.println("Stopping sound.");
  setFrequency(0.0);
  setModulation(0.0);
}


// ---------------------------------------------------
// ✨ FUNCTION TO HANDLE SERIAL COMMANDS ✨
// ---------------------------------------------------
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    Serial.print("Received command: '");
    Serial.print(input);
    Serial.println("'");

    String command = "";
    double freq1 = 440.0; // Default carrier frequency
    double freq2 = 0.0;   // Default modulation frequency (0 = no effect)

    int space1 = input.indexOf(' ');
    int space2 = -1;

    if (space1 == -1) {
      // No spaces, just a command
      command = input;
    } else {
      // At least one parameter
      command = input.substring(0, space1);
      
      // Look for a second parameter
      space2 = input.indexOf(' ', space1 + 1);

      if (space2 == -1) {
        // Only one parameter
        String val1 = input.substring(space1 + 1);
        freq1 = val1.toFloat();
      } else {
        // Two parameters
        String val1 = input.substring(space1 + 1, space2);
        String val2 = input.substring(space2 + 1);
        freq1 = val1.toFloat();
        freq2 = val2.toFloat();
      }
    }
    
    // Now, act on the command
    if (command == "forward" || command == "f") {
      Serial.printf("Setting FORWARD: Tone=%.1f Hz, Effect=%.1f Hz\n", freq1, freq2);
      setFrequency(freq1);
      setModulation(freq2);
    } else if (command == "backward" || command == "b") {
      Serial.printf("Setting BACKWARD: Tone=%.1f Hz, Effect=%.1f Hz\n", freq1, freq2);
      setFrequency(freq1);
      setModulation(freq2);
    } else if (command == "stop" || command == "s") {
      stopSound();
    } else {
      Serial.println("Unknown command. Try 'forward [tone_hz] [effect_hz]' or 'stop'.");
    }
  }
}

// ---------------------------------------------------
// END NEW FUNCTIONS
// ---------------------------------------------------


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
  Serial.println("\n\nTLV320DAC3100 AM Tone Generator");

  // --- All your TLV320 codec setup (UNCHANGED) ---
  pinMode(TLV_RESET, OUTPUT);
  digitalWrite(TLV_RESET, LOW);
  delay(100);
  digitalWrite(TLV_RESET, HIGH);

  Serial.println("Init TLV DAC");
  if (!codec.begin()) { halt("Failed to initialize codec!"); }
  delay(10);
  if (!codec.setCodecInterface(TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_16)) { halt("Failed to configure codec interface!"); }
  if (!codec.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL) || !codec.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK)) { halt("Failed to configure codec clocks!"); }
  if (!codec.setPLLValues(1, 2, 32, 0)) { halt("Failed to configure PLL values!"); }
  if (!codec.setNDAC(true, 8) || !codec.setMDAC(true, 2)) { halt("Failed to configure DAC dividers!"); }
  if (!codec.powerPLL(true)) { halt("Failed to power up PLL!"); }
  if (!codec.setDACDataPath(true, true, TLV320_DAC_PATH_NORMAL, TLV320_DAC_PATH_NORMAL, TLV320_VOLUME_STEP_1SAMPLE)) { halt("Failed to configure DAC data path!"); }
  if (!codec.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER, TLV320_DAC_ROUTE_MIXER, false, false, false, false)) { halt("Failed to configure DAC routing!"); }
  if (!codec.setDACVolumeControl(false, false, TLV320_VOL_INDEPENDENT) || !codec.setChannelVolume(false, 100) || !codec.setChannelVolume(true, 100)) { halt("Failed to configure DAC volumes!"); }
  if (!codec.configureHeadphoneDriver(true, true, TLV320_HP_COMMON_1_35V, false) || !codec.configureHPL_PGA(0, true) || !codec.configureHPR_PGA(0, true) || !codec.setHPLVolume(true, 100) || !codec.setHPRVolume(true, 100)) { halt("Failed to configure headphone outputs!"); }
  if (!codec.enableSpeaker(true) || !codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true) || !codec.setSPKVolume(true, 0)) { halt("Failed to configure speaker output!"); }
  if (!codec.configMicBias(false, true, TLV320_MICBIAS_AVDD) || !codec.setHeadsetDetect(true) || !codec.setInt1Source(true, true, false, false, false, false) || !codec.setGPIO1Mode(TLV320_GPIO1_INT1)) { halt("Failed to configure headset detect"); }
  Serial.println("TLV config done!");
  // --- End of TLV320 codec setup ---

  
  // --- NEW: Print instructions for the user ---
  Serial.println("\n--- 🎛️ Serial AM Control ---");
  Serial.println("Type commands to control the tone:");
  Serial.println("  'forward [tone_hz] [effect_hz]'");
  Serial.println("  'backward [tone_hz] [effect_hz]'");
  Serial.println("  'stop' or 's'");
  Serial.println("\n--- Examples ---");
  Serial.println("  'forward 440 1' (440Hz tone, 1Hz tremolo)");
  Serial.println("  'forward 440 0' (440Hz tone, no effect)");
  Serial.println("  'backward 220 5' (220Hz tone, 5Hz tremolo)");
  Serial.println("---------------------------------");

  // Start silent
  stopSound();
}


void loop() {
  
  // 1. Check for new serial commands
  handleSerialCommands();

  // 2. Fill the audio buffer with generated samples
  for (int i = 0; i < I2S_BUFFER_LEN_SAMPLES; i++) {
    int16_t sample;
    
    if (current_frequency == 0.0) {
      sample = 0; // If main frequency is 0, we're silent
    } else {
      
      // --- MODULATOR ---
      // 1. Get the LFO value (-1.0 to +1.0)
      double mod_value = sin(mod_phase);
      // 2. Remap it to a 0.0 to 1.0 range
      // This 'current_gain' will smoothly pulse between 0 (silent) and 1 (full)
      double current_gain = (mod_value + 1.0) / 2.0; 

      // --- CARRIER ---
      // 3. Get the audible tone value (-1.0 to +1.0)
      double carrier_sample = sin(current_phase);

      // --- COMBINE ---
      // 4. Apply the LFO's gain to the max_amplitude
      double current_amplitude = max_amplitude * current_gain;
      // 5. Apply that amplitude to the carrier tone
      sample = (int16_t)(current_amplitude * carrier_sample);

      
      // --- UPDATE PHASES ---
      // 6. Increment phases for the *next* sample
      current_phase += phase_step;
      mod_phase += mod_phase_step;
      
      // Wrap phases
      if (current_phase >= 2.0 * M_PI) { current_phase -= 2.0 * M_PI; }
      if (mod_phase >= 2.0 * M_PI) { mod_phase -= 2.0 * M_PI; }
    }
    
    // Combine into a 32-bit stereo sample
    i2s_buffer[i] = ((int32_t)sample << 16) | (sample & 0xFFFF);
  }

  // 3. Write the entire buffer to the I2S DAC
  i2s.write((uint8_t*)i2s_buffer, I2S_BUFFER_LEN_SAMPLES * sizeof(int32_t));

  if (current_frequency == 0.0) {
    current_phase = 0.0;
    mod_phase = 0.0;
    delay(10); // Don't waste CPU cycles on silence
  }
}
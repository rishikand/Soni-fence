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
volatile double current_frequency = 0.0;
volatile double current_phase = 0.0;
volatile double phase_step = 0.0;

// --- Modulator (LFO for Amplitude) ---
volatile double mod_frequency = 0.0;
volatile double mod_phase = 0.0;
volatile double mod_phase_step = 0.0;

enum LfoShape {
  LFO_NONE,
  LFO_SINE,
  LFO_SAWTOOTH_RISING,
  LFO_SAWTOOTH_FALLING,
  LFO_SQUARE
};
volatile LfoShape current_lfo_shape = LFO_NONE;
// ---------------------------------------------------

// ---------------------------------------------------
// FENCING SOUND SEQUENCER (with Envelopes)
// ---------------------------------------------------

// 1. Sound definition with ASR envelope
struct FencingSound {
  double carrier_hz;
  double mod_hz;
  LfoShape mod_shape;
  int attack_ms;
  int sustain_ms;
  int release_ms;
};

// 2. The "playlist" of actions
#define MAX_QUEUE_SIZE 20
FencingSound actionQueue[MAX_QUEUE_SIZE];
int queueHead = 0; // Where we are reading from
int queueTail = 0; // Where we are writing to

// 3. Playback state machine
enum PlaybackState {
  STATE_IDLE,
  STATE_ATTACK,
  STATE_SUSTAIN,
  STATE_RELEASE,
  STATE_GAP
};
volatile PlaybackState currentState = STATE_IDLE;

unsigned long stateStartTime = 0;
FencingSound currentAction;
const int GAP_BETWEEN_SOUNDS_MS = 50; 

// ✨ Global speed control
volatile float globalTempoFactor = 1.0; // 1.0 = normal speed. 0.5 = double speed.

// ✨ Master volume control for the envelope
volatile float envelope_gain = 0.0; 
// ---------------------------------------------------


#define I2S_BUFFER_LEN_SAMPLES 256
int32_t i2s_buffer[I2S_BUFFER_LEN_SAMPLES];


void halt(const char *message) {
  Serial.println(message);
  while (1)
    yield();  // Function to halt on critical errors
}

// Create I2S port
I2SClass i2s;

// --- Function to set the synthesizer's "knobs" ---
void setSynth(double new_freq, double new_mod_freq, LfoShape new_shape) {
  current_frequency = new_freq;
  phase_step = (2.0 * M_PI * current_frequency) / (double)sampleRate;
  
  mod_frequency = new_mod_freq;
  mod_phase_step = (2.0 * M_PI * mod_frequency) / (double)sampleRate;
  current_lfo_shape = new_shape;
}

void stopSound() {
  setSynth(0.0, 0.0, LFO_NONE);
}

// ---------------------------------------------------
// SERIAL COMMAND PARSING
// ---------------------------------------------------
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input.length() == 0) return;

    // --- Parse multi-word command ---
    int space1 = input.indexOf(' ');
    String command = (space1 == -1) ? input : input.substring(0, space1);
    String remaining = (space1 == -1) ? "" : input.substring(space1 + 1);

    // --- TEMPO / SPEED COMMAND ---
    if (command == "tempo" || command == "speed") {
      float new_speed_multiplier = remaining.toFloat();
      
      // Safety check: Don't allow extreme values
      if (new_speed_multiplier > 0.2 && new_speed_multiplier < 5.0) { 
        // We invert the user's input: if they enter 2.0 (twice as fast), we use 0.5
        globalTempoFactor = 1.0 / new_speed_multiplier; 
        Serial.printf("-> Set Speed: %.1fx (Total duration multiplier: %.2f)\n", 
                      new_speed_multiplier, globalTempoFactor);
      } else {
        Serial.println("Tempo must be between 0.2x and 5.0x.");
      }
      return;
    }
    
    if (currentState != STATE_IDLE) {
      Serial.println("Busy playing sequence, try again later.");
      return; 
    }

    Serial.print("Received Sequence: '"); Serial.print(input); Serial.println("'");

    // Clear the old playlist
    queueHead = 0;
    queueTail = 0;
    
    char strBuffer[100];
    input.toCharArray(strBuffer, 100);
    char* word = strtok(strBuffer, " ");

    while (word != NULL && queueTail < MAX_QUEUE_SIZE) {
      String w = String(word);
      bool found = true;
      
      // Presets: {Carrier Hz, Mod Hz, LFO Shape, Attack MS, Sustain MS, Release MS}
      if (w == "advance" || w == "a") {
        actionQueue[queueTail] = {400, 5.0, LFO_SAWTOOTH_RISING, 30, 200, 30};
      } 
      else if (w == "retreat" || w == "r") {
        actionQueue[queueTail] = {300, 5.0, LFO_SAWTOOTH_FALLING, 30, 200, 30};
      }
      else if (w == "lunge" || w == "l") {
        actionQueue[queueTail] = {500, 10.0, LFO_SAWTOOTH_RISING, 10, 400, 50};
      }
      else if (w == "hit" || w == "h") {
        actionQueue[queueTail] = {1200, 0.0, LFO_NONE, 5, 100, 20};
      }
      else if (w == "parry" || w == "p") {
        actionQueue[queueTail] = {250, 20.0, LFO_SQUARE, 10, 150, 20};
      }
      else {
        found = false; 
      }
      
      if (found) {
        queueTail++; 
      }
      
      word = strtok(NULL, " "); 
    }
  }
}

// ---------------------------------------------------
// PLAYBACK ENGINE STATE MACHINE
// ---------------------------------------------------
void checkPlaybackQueue() {
  unsigned long elapsed = millis() - stateStartTime;
  unsigned long required_time;

  switch (currentState) {
    
    case STATE_IDLE:
      if (queueHead < queueTail) { 
        currentAction = actionQueue[queueHead];
        queueHead++;
        setSynth(currentAction.carrier_hz, currentAction.mod_hz, currentAction.mod_shape);
        stateStartTime = millis();
        currentState = STATE_ATTACK;
      }
      break;

    case STATE_ATTACK:
      required_time = (unsigned long)(currentAction.attack_ms * globalTempoFactor);
      if (elapsed < required_time) {
        // Ramp gain from 0.0 to 1.0 (Linear interpolation for smoothing)
        envelope_gain = (float)elapsed / required_time; 
      } else {
        envelope_gain = 1.0;
        stateStartTime = millis();
        currentState = STATE_SUSTAIN;
      }
      break;

    case STATE_SUSTAIN:
      required_time = (unsigned long)(currentAction.sustain_ms * globalTempoFactor);
      if (elapsed >= required_time) {
        stateStartTime = millis();
        currentState = STATE_RELEASE;
      }
      break;

    case STATE_RELEASE:
      required_time = (unsigned long)(currentAction.release_ms * globalTempoFactor);
      if (elapsed < required_time) {
        // Ramp gain from 1.0 down to 0.0
        envelope_gain = 1.0 - ((float)elapsed / required_time);
      } else {
        envelope_gain = 0.0;
        stopSound();
        stateStartTime = millis();
        currentState = STATE_GAP;
      }
      break;

    case STATE_GAP:
      required_time = (unsigned long)(GAP_BETWEEN_SOUNDS_MS * globalTempoFactor);
      if (elapsed >= required_time) {
        currentState = STATE_IDLE; 
      }
      break;
  }
}
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
  Serial.println("\n\nTLV320DAC3100 Fencing Sequencer (Final)");

  // --- TLV320 codec setup (unchanged) ---
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

  
  // --- INSTRUCTIONS ---
  Serial.println("\n--- 🤺 Fencing Sound Control (Smoothed) ---");
  Serial.println("Type a sequence of actions separated by spaces.");
  Serial.println("\n--- Speed Control ---");
  Serial.println("  'tempo [value]' or 'speed [value]'");
  Serial.println("  Example: 'tempo 2.0' (Twice as fast), 'tempo 0.5' (Half speed)");
  Serial.println("\n--- Actions ---");
  Serial.println("  'advance' (a), 'retreat' (r), 'lunge' (l)");
  Serial.println("  'hit' (h), 'parry' (p)");
  Serial.println("\n--- Example Sequence ---");
  Serial.println("  'tempo 1.5' followed by 'advance lunge hit'");
  Serial.println("---------------------------------");

  // Start silent
  stopSound();
  envelope_gain = 0.0;
  currentState = STATE_IDLE;
}


void loop() {
  
  // 1. Check for new serial commands (this fills the queue or sets tempo)
  handleSerialCommands();

  // 2. Check the queue and update the envelope state
  checkPlaybackQueue();

  // 3. Fill the audio buffer
  for (int i = 0; i < I2S_BUFFER_LEN_SAMPLES; i++) {
    int16_t sample;
    
    if (envelope_gain == 0.0) {
      sample = 0; // Silence
    } else {
      
      // --- MODULATOR (LFO) ---
      double lfo_gain = 1.0;
      double mod_phase_normalized = mod_phase / (2.0 * M_PI); 

      switch (current_lfo_shape) {
        case LFO_SINE:
          lfo_gain = (sin(mod_phase) + 1.0) / 2.0;
          break;
        case LFO_SAWTOOTH_RISING:
          lfo_gain = mod_phase_normalized;
          break;
        case LFO_SAWTOOTH_FALLING:
          lfo_gain = 1.0 - mod_phase_normalized;
          break;
        case LFO_SQUARE:
          lfo_gain = (mod_phase_normalized < 0.5) ? 0.0 : 1.0;
          break;
        case LFO_NONE:
        default:
          lfo_gain = 1.0; 
          break;
      }

      // --- CARRIER ---
      double carrier_sample = sin(current_phase);

      // --- COMBINE ---
      double current_amplitude = max_amplitude * lfo_gain * envelope_gain;
      sample = (int16_t)(current_amplitude * carrier_sample);

      // --- UPDATE PHASES ---
      current_phase += phase_step;
      mod_phase += mod_phase_step;
      if (current_phase >= 2.0 * M_PI) { current_phase -= 2.0 * M_PI; }
      if (mod_phase >= 2.0 * M_PI) { mod_phase -= 2.0 * M_PI; }
    }
    
    // Combine into a 32-bit stereo sample
    i2s_buffer[i] = ((int32_t)sample << 16) | (sample & 0xFFFF);
  }

  // 4. Write the buffer to the I2S DAC
  i2s.write((uint8_t*)i2s_buffer, I2S_BUFFER_LEN_SAMPLES * sizeof(int32_t));
}
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/triangle2048_int8.h>

// ----- Pin Definitions -----
// Finger (touch) input pins for the oscillator matrix:
#define RIGHT_INDEX 23
#define RIGHT_MIDDLE 22
#define RIGHT_RING 21
#define RIGHT_PINKY 19

#define LEFT_INDEX 18
#define LEFT_MIDDLE 4
#define LEFT_RING 15
#define LEFT_PINKY 13

// Joystick analog and button pin definitions:
// Top 8 oscillators (rows 0-1) use the left joystick’s Y,
// Bottom 8 oscillators (rows 2-3) use the right joystick’s Y.
#define JOY_LEFT_Y_PIN 34
#define JOY_LEFT_BUTTON_PIN 32
#define JOY_RIGHT_Y_PIN 14
#define JOY_RIGHT_BUTTON_PIN 12

// ----- Adjusted Amplitudes -----
#define OSC_AMPLITUDE 127
const uint8_t gameAmps[4] = { 32, 64, 96, 127 };

// ----- Oscillator Setup -----
const uint8_t rightPins[4] = { RIGHT_INDEX, RIGHT_MIDDLE, RIGHT_RING, RIGHT_PINKY };
const uint8_t leftPins[4]  = { LEFT_INDEX, LEFT_MIDDLE, LEFT_RING, LEFT_PINKY };

const int freqTable[4][4] = {
  {262, 294, 330, 349},   // Row 0: C4, D4, E4, F4
  {392, 440, 494, 523},   // Row 1: G4, A4, B4, C5
  {587, 659, 698, 784},   // Row 2: D5, E5, F5, G5
  {880, 988, 1046, 1175}  // Row 3: A5, B5, C6, D6
};

const uint8_t noteGroup[4][4] = {
  {0, 1, 2, 3},
  {4, 5, 6, 0},
  {1, 2, 3, 4},
  {5, 6, 0, 1}
};

Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscs[4][4] = {
  { Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA) },
    
  { Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA) },
    
  { Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA) },
    
  { Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA),
    Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE>(TRIANGLE2048_DATA) }
};

uint8_t oscTarget[4][4] = {0};
uint8_t smoothedAmp[4][4] = {0};

// ----- Finger Input Debug Setup -----
const uint8_t fingerPins[8] = { RIGHT_INDEX, RIGHT_MIDDLE, RIGHT_RING, RIGHT_PINKY,
                                LEFT_INDEX, LEFT_MIDDLE, LEFT_RING, LEFT_PINKY };
bool prevFingerState[8] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };


// ----- Simon Says Game Variables and Definitions -----
enum SimonState {
  SIMON_IDLE,
  SIMON_PLAY_NOTE,
  SIMON_GAP,
  SIMON_WAIT_INPUT,
  SIMON_FEEDBACK
};

SimonState simonState = SIMON_IDLE;
uint8_t simonLevel = 1;
uint8_t simonTargetRow = 0;
uint8_t simonTargetCol = 0;
bool simonCorrect = false;
unsigned long simonTimer = 0;
unsigned long noteDurationAllowed = 1000;
const unsigned long SIMON_FEEDBACK_DURATION = 1000;


// ----- Song Mode Variables -----

// --- Sticky Loop Mode (triggered by LEFT joystick) ---
// This mode will continuously loop a melody inspired by the "dandandan" opening rhythm.
bool stickyMode = false;
uint8_t stickyIndex = 0;
unsigned long stickyNoteEndTime = 0;
// (All frequencies in Hz; durations in milliseconds)
// The melody below is an original creation that you can adjust further.
const int stickyMelody[] = {
  392, 392, 440, 392, 349, 330,   // Phrase 1
  392, 392, 440, 392, 349, 330    // Repeat for a full cycle (12 notes)
};
const int stickyDurations[] = {
  300, 150, 300, 300, 450, 450,    // Phrase 1 timings
  300, 150, 300, 300, 450, 450     // Repeat timings
};
const uint8_t STICKY_LENGTH = sizeof(stickyMelody) / sizeof(stickyMelody[0]);

// --- Like a Good Neighbor Mode (triggered by RIGHT joystick) ---
// (Uses previous arrays for a 12‑note phrase repeated twice.)
bool neighborMode = false;
uint8_t neighborIndex = 0;
unsigned long neighborNoteEndTime = 0;
const int neighborMelody[] = {
  // First iteration:
  392, 392, 440, 494, 587, 523, 494, 440, 392, 330, 370, 392,
  // Second iteration:
  392, 392, 440, 494, 587, 523, 494, 440, 392, 330, 370, 392
};
const int neighborNoteDurations[] = {
  400, 200, 200, 400, 600, 300, 300, 400, 400, 200, 200, 400,
  400, 200, 200, 400, 600, 300, 300, 400, 400, 200, 200, 400
};
const uint8_t NEIGHBOR_LENGTH = sizeof(neighborMelody) / sizeof(neighborMelody[0]);


// ----- Helper Function: Start a Simon Round -----
void startSimonRound() {
  uint8_t allowedRows = simonLevel;
  simonTargetRow = random(allowedRows);
  simonTargetCol = random(4);
  simonState = SIMON_PLAY_NOTE;
  
  noteDurationAllowed = 1000 - (simonLevel - 1) * 200;
  simonTimer = millis() + noteDurationAllowed;
  
  Serial.print("Simon target: row ");
  Serial.print(simonTargetRow);
  Serial.print(", col ");
  Serial.println(simonTargetCol);
}


void setup() {
  startMozzi();
  
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(rightPins[i], INPUT_PULLUP);
    pinMode(leftPins[i], INPUT_PULLUP);
  }
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(fingerPins[i], INPUT_PULLUP);
  }
  
  pinMode(JOY_LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(JOY_RIGHT_BUTTON_PIN, INPUT_PULLUP);
  
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      oscs[i][j].setFreq(freqTable[i][j]);
    }
  }
  
  Serial.begin(115200);
  randomSeed(analogRead(0));
}


void updateControl() {
  // ---- Sticky Loop Mode Trigger & Processing (LEFT joystick) ----
  if(simonState == SIMON_IDLE && !stickyMode && !neighborMode) {
    bool leftJoyPressed = (digitalRead(JOY_LEFT_BUTTON_PIN) == LOW);
    bool otherKeyPressed = false;
    if(digitalRead(JOY_RIGHT_BUTTON_PIN) == LOW) { otherKeyPressed = true; }
    for (uint8_t i = 0; i < 8; i++) {
      if(digitalRead(fingerPins[i]) == LOW) { otherKeyPressed = true; break; }
    }
    if(leftJoyPressed && !otherKeyPressed) {
      stickyMode = true;
      stickyIndex = 0;
      stickyNoteEndTime = millis(); // trigger immediately
      Serial.println("Sticky Loop Mode Started!");
    }
  }
  
  if(stickyMode) {
    // Cancel if any key (other than the left joystick button) is pressed.
    bool cancelSticky = false;
    if(digitalRead(JOY_RIGHT_BUTTON_PIN) == LOW) { cancelSticky = true; }
    for (uint8_t i = 0; i < 8; i++) {
      if(digitalRead(fingerPins[i]) == LOW) { cancelSticky = true; break; }
    }
    if(cancelSticky) {
      stickyMode = false;
      oscTarget[0][0] = 0;
      Serial.println("Sticky Loop Mode Cancelled");
      return;
    }
    
    unsigned long currentMillis = millis();
    if(currentMillis >= stickyNoteEndTime) {
      int note = stickyMelody[stickyIndex];
      int duration = stickyDurations[stickyIndex];
      oscs[0][0].setFreq(note);
      oscTarget[0][0] = OSC_AMPLITUDE;
      stickyNoteEndTime = currentMillis + duration;
      stickyIndex++;
      if(stickyIndex >= STICKY_LENGTH) stickyIndex = 0; // loop back to beginning
    }
    // Silence all other oscillators.
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        if(!(i == 0 && j == 0))
          oscTarget[i][j] = 0;
      }
    }
    return;
  }
  
  // ---- Like a Good Neighbor Mode Trigger & Processing (RIGHT joystick) ----
  if(simonState == SIMON_IDLE && !neighborMode && !stickyMode) {
    bool rightJoyPressed = (digitalRead(JOY_RIGHT_BUTTON_PIN) == LOW);
    bool otherKeyPressed = false;
    if(digitalRead(JOY_LEFT_BUTTON_PIN) == LOW) { otherKeyPressed = true; }
    for (uint8_t i = 0; i < 8; i++) {
      if(digitalRead(fingerPins[i]) == LOW) { otherKeyPressed = true; break; }
    }
    if(rightJoyPressed && !otherKeyPressed) {
      neighborMode = true;
      neighborIndex = 0;
      neighborNoteEndTime = millis(); // trigger immediately
      Serial.println("Like a Good Neighbor Mode Started!");
    }
  }
  
  if(neighborMode) {
    bool cancelNeighbor = false;
    if(digitalRead(JOY_LEFT_BUTTON_PIN) == LOW) { cancelNeighbor = true; }
    for (uint8_t i = 0; i < 8; i++) {
      if(digitalRead(fingerPins[i]) == LOW) { cancelNeighbor = true; break; }
    }
    if(cancelNeighbor) {
      neighborMode = false;
      oscTarget[0][1] = 0;
      Serial.println("Like a Good Neighbor Mode Cancelled");
      return;
    }
    
    unsigned long currentMillis = millis();
    if(currentMillis >= neighborNoteEndTime) {
      if(neighborIndex < NEIGHBOR_LENGTH) {
        int note = neighborMelody[neighborIndex];
        int duration = neighborNoteDurations[neighborIndex];
        oscs[0][1].setFreq(note);
        oscTarget[0][1] = OSC_AMPLITUDE;
        neighborNoteEndTime = currentMillis + duration;
        neighborIndex++;
      } else {
        neighborMode = false;
        oscTarget[0][1] = 0;
        Serial.println("Like a Good Neighbor Mode Ended");
      }
    }
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        if(!(i == 0 && j == 1))
          oscTarget[i][j] = 0;
      }
    }
    return;
  }
  
  // ---- Simon Game Logic ----
  if (simonState == SIMON_IDLE) {
    bool joyLeftPressed = (digitalRead(JOY_LEFT_BUTTON_PIN) == LOW);
    bool joyRightPressed = (digitalRead(JOY_RIGHT_BUTTON_PIN) == LOW);
    int chosenLevel = 0;
    for (uint8_t i = 0; i < 4; i++) {
      if (digitalRead(rightPins[i]) == LOW) {
        chosenLevel = i + 1;
        break;
      }
    }
    if (joyLeftPressed && joyRightPressed && chosenLevel != 0) {
      simonLevel = chosenLevel;
      Serial.print("Starting Simon Says at level ");
      Serial.println(simonLevel);
      startSimonRound();
      return;
    }
  }
  
  if (simonState != SIMON_IDLE) {
    unsigned long currentMillis = millis();
    switch (simonState) {
      case SIMON_PLAY_NOTE:
        for (uint8_t i = 0; i < 4; i++)
          for (uint8_t j = 0; j < 4; j++)
            oscTarget[i][j] = 0;
        oscTarget[simonTargetRow][simonTargetCol] = gameAmps[simonTargetRow];
        if (currentMillis >= simonTimer) {
          unsigned long gapDuration = 500 - (simonLevel - 1) * 100;
          simonState = SIMON_GAP;
          simonTimer = currentMillis + gapDuration;
        }
        break;
        
      case SIMON_GAP:
        for (uint8_t i = 0; i < 4; i++)
          for (uint8_t j = 0; j < 4; j++)
            oscTarget[i][j] = 0;
        if (currentMillis >= simonTimer) {
          simonState = SIMON_WAIT_INPUT;
          simonTimer = currentMillis + noteDurationAllowed;
        }
        break;
        
      case SIMON_WAIT_INPUT: {
        if (currentMillis >= simonTimer) {
          simonCorrect = false;
          simonState = SIMON_FEEDBACK;
          simonTimer = currentMillis + SIMON_FEEDBACK_DURATION;
          break;
        }
        bool inputDetected = false;
        uint8_t inputRow = 255, inputCol = 255;
        for (uint8_t i = 0; i < 4; i++) {
          if (digitalRead(rightPins[i]) == LOW) {
            for (uint8_t j = 0; j < 4; j++) {
              if (digitalRead(leftPins[j]) == LOW) {
                inputDetected = true;
                inputRow = i;
                inputCol = j;
                break;
              }
            }
            if (inputDetected) break;
          }
        }
        if (inputDetected) {
          simonCorrect = (inputRow == simonTargetRow && inputCol == simonTargetCol);
          simonState = SIMON_FEEDBACK;
          simonTimer = currentMillis + SIMON_FEEDBACK_DURATION;
        }
        break;
      }
      case SIMON_FEEDBACK:
        for (uint8_t i = 0; i < 4; i++)
          for (uint8_t j = 0; j < 4; j++)
            oscTarget[i][j] = 0;
        if (simonCorrect)
          oscTarget[simonTargetRow][simonTargetCol] = gameAmps[simonTargetRow];
        else
          oscTarget[0][0] = gameAmps[0];
        if (currentMillis >= simonTimer) {
          if (simonCorrect) {
            if (simonLevel < 4) {
              simonLevel++;
              startSimonRound();
            } else {
              Serial.println("Simon Says: You won!");
              simonState = SIMON_IDLE;
            }
          } else {
            Serial.println("Simon Says: Incorrect. Game over.");
            simonLevel = 1;
            simonState = SIMON_IDLE;
          }
        }
        break;
      default:
        break;
    }
    return;
  }
  
  // ---- Normal Synth Mode ----
  int leftY = mozziAnalogRead(JOY_LEFT_Y_PIN);
  int rightY = mozziAnalogRead(JOY_RIGHT_Y_PIN);
  float leftShift = ((leftY - 512) / 512.0) * 0.025;
  float rightShift = ((rightY - 512) / 512.0) * 0.025;
  
  for (uint8_t i = 0; i < 4; i++) {
    float multiplier = (i < 2) ? (1.0 + leftShift) : (1.0 + rightShift);
    for (uint8_t j = 0; j < 4; j++) {
      int newFreq = freqTable[i][j] * multiplier;
      oscs[i][j].setFreq(newFreq);
    }
  }
  
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++)
      oscTarget[i][j] = ((digitalRead(rightPins[i]) == LOW) && (digitalRead(leftPins[j]) == LOW))
                        ? OSC_AMPLITUDE : 0;
}


AudioOutput updateAudio() {
  int32_t mixedOutput = 0;
  const uint8_t NUM_GROUPS = 7;
  bool groupActive[NUM_GROUPS] = { false };
  
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      smoothedAmp[i][j] = (smoothedAmp[i][j] * 15 + oscTarget[i][j]) >> 4;
      mixedOutput += (oscs[i][j].next() * smoothedAmp[i][j]);
      if (oscTarget[i][j] > 0) {
        uint8_t group = noteGroup[i][j];
        groupActive[group] = true;
      }
    }
  }
  
  uint8_t activeGroupCount = 0;
  for (uint8_t g = 0; g < NUM_GROUPS; g++) {
    if (groupActive[g])
      activeGroupCount++;
  }
  uint8_t divisor = (activeGroupCount == 0) ? 1 : (activeGroupCount > 4 ? 4 : activeGroupCount);
  mixedOutput /= divisor;
  
  return MonoOutput::from16Bit(mixedOutput);
}


void loop() {
  audioHook();
  
  for (uint8_t i = 0; i < 8; i++) {
    bool currentState = (digitalRead(fingerPins[i]) == LOW);
    if (currentState != prevFingerState[i]) {
      prevFingerState[i] = currentState;
      if (currentState) {
        Serial.print("Finger ");
        Serial.print(i + 1);
        Serial.println(" pressed");
      }
    }
  }
}



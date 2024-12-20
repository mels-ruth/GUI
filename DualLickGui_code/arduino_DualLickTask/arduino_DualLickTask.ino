// This is a task adapted from Ally's Labview Code.
// Sketch runs experiment and communicates with Melanie's Dual_Lick_GUI in MATLAB

// load libraries
#include <EEPROM.h> // EEPROM: hard-drive memory storage for parameter variables
// #include <math.h>

// Reward size
unsigned long RewardSizeLeft = 25;   // Duration that left solenoid is left open
unsigned long RewardSizeRight = 25;  // Duration that right solenoid is left open
int minLeft = 25;
int maxLeft = 25;
int stepLeft = 1;
int minRight = 25;
int maxRight = 25;
int stepRight = 1;
int LeftRewardSizes[100];
int RightRewardSizes[100];
int numLeftRewardSizes;
int numRightRewardSizes;

// Fill reward sizes left
void fillLeftRewardSizes() {
  int currentValue = minLeft;
  numLeftRewardSizes = 0;

  for (int i = 0; i < 100; i++) {
    if (currentValue > maxLeft) {
      break;
    }

    LeftRewardSizes[i] = currentValue;
    currentValue += stepLeft;
    numLeftRewardSizes++;
  }
}

// Fill reward sizes right
void fillRightRewardSizes() {
  int currentValue = minRight;
  numRightRewardSizes = 0;

  for (int i = 0; i < 100; i++) {
    if (currentValue > maxRight) {
      break;
    }

    RightRewardSizes[i] = currentValue;
    currentValue += stepRight;
    numRightRewardSizes++;
  }
}

// Reward probability
int RewardProb = 100;  // probability * 100 of reward being dispensed if correct
int RewardProbList[100];
bool giveReward = false;

// Fill reward probability list
void fillRewardProbList() {
  for (int i = 0; i < RewardProb; i++) {
    RewardProbList[i] = 1;
  }
  for (int i = RewardProb + 1; i < (100 - RewardProb); i++) {
    RewardProbList[i] = 0;
  }
}

// Spout start
int spoutStart = 0;  // 0 = random, 1 = left, 2 = right

// Free Reward
int failCounter = 0;        // Tracks consecutive failures
int fails2reward = 6;       // Number of failures before triggering free reward
int freeRewardCounter = 0;  // Counts number of free rewards

// Block length
int BlockLengths[50];  // Number of trials per block - each new block picks from this list with uniform prob
int minBlock = 4;      // Minimum block length
int maxBlock = 8;      // Maximum block length
int numBlockLengths;   // Once fillBlockLengths() is called, this will equal the number of options a block length can be

// Fill block lengths
void fillBlockLengths() {  // Fills BlockLengths with all values in between min and max
  numBlockLengths = 0;

  for (int i = minBlock; i <= maxBlock; i++) {
    BlockLengths[numBlockLengths] = i;
    numBlockLengths++;
  }
}

// Cue variables
unsigned long LeftCueFreq = 5000;           // in Hz
unsigned long RightCueFreq = 5000;          // in Hz
unsigned long ToneDuration = 75;  // Duration of the speaker tone in ms

// Optogenetic stimulation parameters - when to stimulate
bool OptoActiveDuringENL = false;          // Opto stim will happen during the ENL of that trial (only when previous trial was rewarded, but not during the very first trial)
bool OptoActiveDuringCue = false;          // Opto stim will happen during the cue presentation
bool OptoActiveDuringConsumption = false;  // Opto stim will happen during the Consumption state of that trial
bool OptoActiveDuringBlockSwitch = false;   // Opto stim will happen during the first trial after a block switch

// Optogenetic stimulation parameters - minor details
int numCorrectTrialsBeforeOpto = 0;        // Opto stim will happen after the Nth correct trial
bool AllowMultiOptoStimBlock = false;       // allow multiple trials per block to have opto
bool AllowMultiOptoStimENL = false;         // Stimulate every time the ENL gets reset

// Optogenetic stimulation parameters - how to stimulate
unsigned long OptoStimProb = 100;               // 0.65 Probability of opto stim on that trial (set to 0 to turn off opto)
unsigned long OptoDelay = 0;           // delay from start of ENL/Consuption to laser on (in ms)
unsigned long OptoDuration = 1000;     // total laser on time (in ms)
bool optoContinuous = false;           // opto still will be continuous (ideal for inhibition)
unsigned long optoPulseDuration = 10;  // opto pulse duration (ms)
unsigned long optoFrequency = 20;      // opto frequency of pulse + ITI (Hz)

// Trial time parameters (all values in milliseconds)
unsigned long ReactionTime = 5000;         // maximum reaction period (after Cue) in ms
unsigned long ConsumptionDuration = 3000;  // comsumption duration in ms
unsigned long minEnl = 1500;
unsigned long maxEnl = 1500;
unsigned long ENLDurationList[100] = { 0 };
int NumENLDurations = 0;  // ** Must equal num items in ENLDurationList list **
unsigned long ENLstep = 1;

void fillENLDuration() {
  unsigned long currentValue = minEnl;
  NumENLDurations = 0;
  for (int i = 0; i < 100; i++) {
    // Break if we exceed maxEnl
    if (currentValue > maxEnl) {
      break;
    }
    ENLDurationList[i] = currentValue;
    currentValue += ENLstep;
    NumENLDurations++;
  }
}
//unsigned long ENLDurationList[5] = {1000, 1250, 1500, 1750, 2000};

// Penalty durations
unsigned long ENLPenaltyDuration = 10;
unsigned long CuePenaltyDuration = 1500;

// Keep track of session
bool sessionStart = false;  // if true, session is running
bool sessionEnd = false;  // if true, syncPulse will stay low

// Input output pin description //
const byte SyncPin = 2;           // non-periodic sync pulse - 2
const byte SpeakerPin = 3;       //speaker output pin - 47
const byte LeftLickPin = 14;      //left lick detection - 18
const byte RightLickPin = 15;     //right lick detection - 19
const byte LeftRewardPin = 4;    //left spout solenoid - 9
const byte RightRewardPin = 5;   //right spout solenofid - 8

// The following pins are not used in our rig setup
const byte CameraTriggerPin = 1;  //camera trigger - 21
const byte LaserPin = 6;         //laser control - 22 - 4
const byte ShutterRed = 7;        //red shutter as background -24

const bool buttonsAreWiredUp = false;  // Are the buttons wired up??
const byte StartStopButtonPin = 8;    //start-stop button - 53
const byte LeftRewardButtonPin = 9;   //left spout manual activation - 11
const byte RightRewardButtonPin = 16;  //right spout manual activation - 12

const byte LeftCueIndicatorPin = 17;     //speaker output step copy - Left tone - 48
const byte RightCueIndicatorPin = 18;    //speaker output step copy - Right tone - 49
const byte LeftRewardIndicatorPin = 19;   //copy left spout solenoid for data recording device - 6
const byte RightRewardIndicatorPin = 20;  //copy right spout solenoid for data recording device - 7
const byte ENLIndicatorPin = 21;         //pulse that signals ENL period (Modified to 24 by SL, originally 36) - 36
const byte StartStopIndicatorPin = 22;   //start-stop session output pin - 30


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------ Most user-modifiable parameters are above this line -------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////


// Declare all possible states here: (order doesn't matter)
enum state {
  IDLE,
  NEW_TRIAL,
  ENL,
  ENL_PENALTY,
  CUE,
  CUE_PENALTY,
  SELECTION,
  PRE_REWARD,
  FREE_REWARD,
  NO_REWARD,
  REWARD,
  CONSUMPTION
};
typedef enum state DualLickState;

// Initialize real time variables //
char SerialInput = '0';  //incoming serial data

// State-Machine related
DualLickState CurrentState = IDLE;  // MAIN behavior state variable for running behavior task
DualLickState NextState = IDLE;     // used for state transitions
int StartStopButtonStatus = 1;      //start-stop button status

// Sync pulses
unsigned long TimerSync = 0;             //timer for non-periodic sync pulse
unsigned long SyncPulseInterval = 1000;  //interval for non-periodic sync pulse
int SyncPulseStatus = 0;                 //current sync signal status
// Shutter sound at random
unsigned long TimerShutterSound = 0;
unsigned long ShutterSoundInterval = 0;
int ShutterSoundNow = 0;

// Trial stats
int TrialNum = 0;           //current trial number
int BlockNum = 0;           //current block number
int TrialInBlock = 0;       //current number of trials of this block
int Num_Reward = 0;         //current number of rewards
int CorrectTrialCount = 0;  // number of correct trials so far this block
int incorrectCounter = 0;
const int LEFT_BLOCK = 0;
const int RIGHT_BLOCK = 1;
int BlockType = LEFT_BLOCK;

// Assign BlockType based on random number
void randomBlock() {
  int randomNum = random(0, 2);  // generates either 0 or 1
  if (randomNum == 0) {
    BlockType = LEFT_BLOCK;
  } else if (randomNum == 1) {
    BlockType = RIGHT_BLOCK;
  }
}

// Block counters
int Num_ENLPenalty = 0;          // number of enl penalties per block
int Num_Omissions = 0;           // number of omissions per block
int freeRewardBlockCounter = 0;  // number of free rewards per block
int BlockRewardCounter = 0;      // number of rewards per block
int BlockNoRewardCounter = 0;    // number of no-rewards per block
int leftLickCounter = 0;         // number of left licks per block
int rightLickCounter = 0;        // number of right licks per block
int winStayCounter = 0;          // number of win-repeats per block
int loseSwitchCounter = 0;       // number of lose-switches per block
int winSwitchCounter = 0;        // number of win-switches per block
int loseStayCounter = 0;         // number of lose-repeats per block
int badTrialsCounter = 0;        // number of incorrect trials and omissions after a reward per block
bool lastTrial = true;           // outcome of last trial
bool firstTrial = true;          // outcome of first trial
bool blockLengthSent = false;

// Trial structure
int BlockLength = 0;            // will pick from BlockLengths[]
unsigned long ENLDuration = 0;  // Current ITI - will pick from ENLDurationList
const int MAX_BLOCKS = 1024;
int TrialsUntilFirstRewardHistory[MAX_BLOCKS];
int TrialsUntilFirstReward = 0;

// Opto bookkeeping
bool optoTriggeredENL = false;          // are we stimulating
bool optoTriggeredOutcome = false;      // are we stimulating
bool optoTriggeredBlockSwitch = false;  // are we stimulating
bool duringOptoStim = false;
bool duringOptoStimDelay = false;
unsigned long optoStartTime = 0;
bool optoPulseActive = false;           // keep track of pulses
bool optoIPIActive = false;             // keep track of time in opto IPI
bool optoBlockDone = false;             // keep track if opto has occured in this block
bool optoTrialDone = false;             // keep track if opto has occured in this block
unsigned long optoStartPulse = 0;       // keep track of start opto pulse
unsigned long optoTimeInPulse = 0;      // keep track of time in opto pulse
bool optoBlockFirstTrial = false;       // keep track of first trial after block switch
unsigned long optoIPI = 0;              // keep track of opto IPI

// Detection related
bool LeftLickOccuring = false;
bool RightLickOccuring = false;
unsigned long LeftLickStartTime = 0;   //timestamp of last L lick
unsigned long RightLickStartTime = 0;  //timestamp of last R lick

// Reward related
int LeftRewardButtonStatus = 0;      //left reward button status
int RightRewardButtonStatus = 0;     //right reward button status
unsigned long LeftRewardTimer = 0;   //timer for left reward button
unsigned long RightRewardTimer = 0;  //timer for right reward button

// Laser related
bool LaserEnabled = false;    // maser laser enable, no laser stims when false
int LaserTrial = 0;           //number of laser trials
unsigned long LaserTime = 0;  //timestamp for laser on

// Timestamp related
unsigned long LickStartTime = 0;  //timestamp for current lick
unsigned long ENLStartTime = 0;   //timestamp for beginning of ENL
unsigned long PenaltyStartTime = 0;
unsigned long CueStartTime = 0;          //timestamp for cue onset
unsigned long CueOffTime = 0;            //timestamp for cue off
unsigned long SelectionStartTime = 0;    //timestamp for selection window
unsigned long RewardStartTime = 0;       //timestamp for solenoid on (choice lick) or current timestamp after cue off
unsigned long RewardOffTime = 0;         //timestamp for solenoid off
unsigned long ConsumptionStartTime = 0;  //timestamp for Consumption
unsigned long CameraTrigStartTime = 0;   //timestamp for starting the session (used for triggering camera)

// Buffer String for serial printing
String serialOutBuffer = "";
bool pausePrint = false;

// Loop timing metrics
unsigned long maxLoopTime_us = 0;
unsigned long cumulativeLoopTime_us = 0;
unsigned long loopCounter = 0;
unsigned long loopStartTime_us = 0;

//////////////////////////////////////////////////////////////////////////////////
// ------------------------------ EEPROM things ------------------------------- //
//////////////////////////////////////////////////////////////////////////////////

// Arrays of variables to be stored. Must be in their respective variable type array
int* intParams[] = { // variables to save to EEPROM
  &minBlock,
  &maxBlock,
  &fails2reward,
  &minLeft,
  &stepLeft,
  &minRight,
  &maxRight,
  &stepRight,
  &RewardProb,
  &spoutStart,
  &numCorrectTrialsBeforeOpto
};

bool* boolParams[] = {
  &optoContinuous,
  &OptoActiveDuringBlockSwitch,
  &OptoActiveDuringENL,
  &OptoActiveDuringCue,
  &OptoActiveDuringConsumption,
  &AllowMultiOptoStimENL,
  &AllowMultiOptoStimBlock
};

unsigned long* ulongParams[] = {
  &minEnl,
  &maxEnl,
  &ENLstep,
  &LeftCueFreq,
  &RightCueFreq,
  &ReactionTime,
  &CuePenaltyDuration,
  &RewardSizeLeft,
  &RewardSizeRight,
  &OptoStimProb,
  &OptoDelay,
  &OptoDuration,
  &optoPulseDuration,
  &optoFrequency
};

const int numIntParams = sizeof(intParams) / sizeof(intParams[0]);
const int numBoolParams = sizeof(boolParams) / sizeof(boolParams[0]);
const int numUlongParams = sizeof(ulongParams) / sizeof(ulongParams[0]);

// Save parameters to EEPROM. Can only be done 100,000 times
void saveParams() {
  int address = 0;

  // Save all int parameters
  for (int i = 0; i < numIntParams; i++) {
    EEPROM.update(address, *intParams[i]); // update() compares the new and old values and only re-writes if they are different, this avoids excessive overwrites
    address += sizeof(int);
  }

  // Save all boolean parameters
  for (int i = 0; i < numBoolParams; i++) {
    EEPROM.update(address, *boolParams[i]);
    address += sizeof(bool);
  }

  // Save all unsigned long parameters
  for (int i = 0; i < numUlongParams; i++) {
    byte* pointer = (byte*)&(*ulongParams[i]);
    for (unsigned long j = 0; j < sizeof(unsigned long); j++) {
      EEPROM.update(address++, pointer[j]);
    }
  }
}

// Get parameters from EEPROM. No limit on how many times you can call this function
void getParams() {
  int address = 0;

  // Get all int parameters
  for (int i = 0; i < numIntParams; i++) {
    *intParams[i] = EEPROM.read(address);
    address += sizeof(int);
  }

  // Get all boolean parameters
  for (int i = 0; i < numBoolParams; i++) {
    *boolParams[i] = EEPROM.read(address);
    address += sizeof(bool);
  }

  // Get all unsigned long parameters
  for (int i = 0; i < numUlongParams; i++) {
    byte* pointer = (byte*)&(*ulongParams[i]);
    for (unsigned long j = 0; j < sizeof(unsigned long); j++) {
      pointer[j] = EEPROM.read(address++);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////
// -------------------------- End of EEPROM things ---------------------------- //
//////////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(38400);

  getParams();

  pinMode(SyncPin, OUTPUT);
  pinMode(LeftRewardPin, OUTPUT);
  pinMode(RightRewardPin, OUTPUT);
  pinMode(LeftRewardIndicatorPin, OUTPUT);
  pinMode(RightRewardIndicatorPin, OUTPUT);
  pinMode(LeftLickPin, INPUT);
  pinMode(RightLickPin, INPUT);
  pinMode(SpeakerPin, OUTPUT);
  pinMode(LeftCueIndicatorPin, OUTPUT);
  pinMode(RightCueIndicatorPin, OUTPUT);
  pinMode(CameraTriggerPin, OUTPUT);
  pinMode(StartStopIndicatorPin, OUTPUT);  //output pin for start and end of task
  pinMode(LaserPin, OUTPUT);               //laser pin
  pinMode(ENLIndicatorPin, OUTPUT);
  if (buttonsAreWiredUp) {
    pinMode(StartStopButtonPin, INPUT);
    pinMode(LeftRewardButtonPin, INPUT);
    pinMode(RightRewardButtonPin, INPUT);
  }

  // attachInterrupt(digitalPinToInterrupt(RightLickPin), LICK_OFF, RISING);
  // attachInterrupt(digitalPinToInterrupt(LeftLickPin), LICK_ON, FALLING);
  // attachInterrupt(digitalPinToInterrupt(RightRewardButtonPin), DeliverReward, RISING);

  CameraTrigStartTime = millis();
  CurrentState = IDLE;
  SyncPulseStatus = 0;

  digitalWrite(SyncPin, LOW);
  digitalWrite(LeftRewardPin, LOW);
  digitalWrite(RightRewardPin, LOW);
  digitalWrite(LeftRewardIndicatorPin, LOW);
  digitalWrite(RightRewardIndicatorPin, LOW);
  digitalWrite(ENLIndicatorPin, LOW);
  noTone(SpeakerPin);
  digitalWrite(CameraTriggerPin, LOW);
  digitalWrite(StartStopIndicatorPin, LOW);
  digitalWrite(LaserPin, LOW);
  randomSeed(analogRead(3));

  Serial.println("-----------------------------------------------------------------");
  Serial.println("Manual check: 1 -> L-reward; 2 -> R-reward;");
  Serial.println("Laser shutter: 3 -> blue stim on; 4 -> blue stim off");
  Serial.println("OptoStim: 5 -> enable; 6 -> disable");
  Serial.println("Trial start/stop: 8 -> start; 9 -> end");
  Serial.println("Timing Debug Info: T    Status: S");
  Serial.println("-----------------------------------------------------------------");
  Serial.println("");
  Serial.println("");
  printDataHeader();
  serialOutBuffer.reserve(1024);  // pre-allocate to larger size than we'll need
  loopStartTime_us = micros();

  fillRewardProbList();  // called here for an un-changeable probability
}


void loop() {
  updateSyncPulses();
  lickDetection();
  updateStateMachine();
  checkSerialInput();  // Needed for non-parameter commands
  updateCameraTrigger();
  updateOptoStim();
  updateSerialOutput();
  //randomShutterSound();

  // loop timing stats
  unsigned long now_us = micros();
  loopCounter++;
  cumulativeLoopTime_us += (now_us - loopStartTime_us);
  maxLoopTime_us = max(maxLoopTime_us, now_us - loopStartTime_us);
  loopStartTime_us = now_us;
}

void updateStateMachine() {
  // if we just entered a new state, set <enteredNewState> to true, and print out state info
  bool enteredNewState = false;
  if (NextState != CurrentState) {
    enteredNewState = true;
    CurrentState = NextState;
    outputTrialState();
  }

  //   ==============   STATE MACHINE LOGIC   ==============

  //   ----------------------------   IDLE   ----------------------------
  // - wait until StartStop button pushed
  if (CurrentState == IDLE) {
    if (enteredNewState) {
      // reset all trial variables and hardware
      TrialNum = 0;           //current trial number
      BlockNum = 0;           //current block number
      Num_Reward = 0;         // current number of rewards
      TrialInBlock = 0;       //current number of trials of this block
      CorrectTrialCount = 0;  // number of correct trials so far this block
      incorrectCounter = 0;
      Num_ENLPenalty = 0;
      Num_Omissions = 0;
      failCounter = 0;  // counter for fails until reward
      freeRewardCounter = 0;
      freeRewardBlockCounter = 0;
      BlockRewardCounter = 0;
      BlockNoRewardCounter = 0;
      leftLickCounter = 0;
      rightLickCounter = 0;
      winStayCounter = 0;
      loseSwitchCounter = 0;
      winSwitchCounter = 0;
      loseStayCounter = 0;
      badTrialsCounter = 0;
      lastTrial = true;

      BlockLength = 0;  // will pick from BlockLengths[]
      digitalWrite(LeftRewardPin, LOW);
      digitalWrite(RightRewardPin, LOW);
      digitalWrite(LeftRewardIndicatorPin, LOW);
      digitalWrite(RightRewardIndicatorPin, LOW);
      digitalWrite(ENLIndicatorPin, LOW);
      noTone(SpeakerPin);
      digitalWrite(CameraTriggerPin, LOW);
      digitalWrite(StartStopIndicatorPin, LOW);
      digitalWrite(LaserPin, LOW);
    }
    //   ----------------------------   NEW_TRIAL   ----------------------------
    // - Update trial/block, Set trial-specific variables (ENL duration, opto stim)
  } else if (CurrentState == NEW_TRIAL) {
    if (failCounter >= fails2reward) {  // Check if fails until reward threshold has been met, this is here for omissions
      NextState = FREE_REWARD;
      failCounter = 0;
      freeRewardCounter += 1;
      freeRewardBlockCounter += 1;
      Serial.print("Free Rewards\t");
      Serial.print(freeRewardCounter);
      Serial.print("\n");
    } else {
      digitalWrite(LeftRewardPin, LOW);
      digitalWrite(RightRewardPin, LOW);
      digitalWrite(LeftRewardIndicatorPin, LOW);
      digitalWrite(RightRewardIndicatorPin, LOW);
      noTone(SpeakerPin);

      ENLStartTime = millis();
      digitalWrite(StartStopIndicatorPin, LOW);
      digitalWrite(ENLIndicatorPin, HIGH);

      int randomIdx = random(0, NumENLDurations);
      ENLDuration = ENLDurationList[randomIdx];
      // Serial.print("randomIdx: ");
      // Serial.println(randomIdx);
      // Serial.print("ENLDuration: ");
      // Serial.println(ENLDuration);

      incrementTrial();
      NextState = ENL;
    }

    //   ----------------------------   ENL   ----------------------------
    // - wait for full ENLDuration with no licks
  } else if (CurrentState == ENL) {
    if (enteredNewState) {
      blockLengthSent = false;
      
      ENLStartTime = millis();
      digitalWrite(ENLIndicatorPin, HIGH);

      // check for opto stim first trial after block switch
      if (optoTriggeredBlockSwitch == true) {
        // ENL penalty so repeat Block switch opto
        startOptoStim();
        if (LaserEnabled) {
          outputEvent("Opto BlockSwitch_P", millis(), OptoDuration);
        }
      } else if ((OptoActiveDuringBlockSwitch == true) && (optoBlockFirstTrial == true) && (optoTrialDone == false)) {
        // first trial ENL after block switch
        if (random(100) < OptoStimProb) {
          optoTriggeredBlockSwitch = true;
          optoBlockFirstTrial = false;
          optoTrialDone = true;
          startOptoStim();
          outputEvent("Opto BlockSwitch", millis(), OptoDuration);
          if (!AllowMultiOptoStimBlock) {
            optoBlockDone = true;
          }
        }
      }

      // check for opto stim during the current ENL
      if (optoTriggeredENL == true) {
        // ENL penalty so repeat ENL opto
        startOptoStim();
        outputEvent("Opto ENL_P", millis(), OptoDuration);
      } else if ((OptoActiveDuringENL == true) && (optoBlockDone == false) && (firstTrial == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          // just stimulate whenever possible
          if (random(100) < OptoStimProb) {
            optoTriggeredENL = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto ENL", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if ((CorrectTrialCount >= numCorrectTrialsBeforeOpto) && (lastTrial == true) && (optoTrialDone == false)) {
          // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
          if (random(100) < OptoStimProb) {
            optoTriggeredENL = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto ENL", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }
    }

    if (LeftLickOccuring || RightLickOccuring) {
      digitalWrite(ENLIndicatorPin, LOW);
      // check if we need to stimulate ENL again this trial
      if ((AllowMultiOptoStimENL == true) && (optoTriggeredENL == true)) {
        optoTriggeredENL = true;
      } else {
        optoTriggeredENL = false;
      }
      // check if we need to stimulate BlockSwitch again this trial
      if ((AllowMultiOptoStimENL == true) && (optoTriggeredBlockSwitch == true)) {
        optoTriggeredBlockSwitch = true;
      } else {
        optoTriggeredBlockSwitch = false;
      }

      //
      NextState = ENL_PENALTY;
    }

    if (millis() - ENLStartTime >= ENLDuration) {
      digitalWrite(ENLIndicatorPin, LOW);
      optoTriggeredENL = false;
      optoTriggeredBlockSwitch = false;
      optoBlockFirstTrial = false;
      NextState = CUE;
    }

    //   ----------------------------   ENL_PENALTY   ----------------------------
    // - wait ENLPenaltyDuration, then enter ENL
  } else if (CurrentState == ENL_PENALTY) {
    if (enteredNewState) {
      PenaltyStartTime = millis();
      Num_ENLPenalty += 1;
    }
    if (millis() - PenaltyStartTime >= ENLPenaltyDuration) {
      if (!LeftLickOccuring && !RightLickOccuring) {
        NextState = ENL;
      }
    }

    //   ----------------------------   CUE   ----------------------------
    // - play tone
    // - wait for full CueDuration with no licks
  } else if (CurrentState == CUE) {
    if (enteredNewState) {
      CueStartTime = millis();

      // check for opto stim during the cue
      if ((OptoActiveDuringCue == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          // just stimulate whenever possible
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto Cue", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto Cue", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }

      if (BlockType == LEFT_BLOCK) {
        tone(SpeakerPin, LeftCueFreq);
        digitalWrite(LeftCueIndicatorPin, HIGH);
      } else {
        tone(SpeakerPin, RightCueFreq);
        digitalWrite(RightCueIndicatorPin, HIGH);
      }
    }

    if (LeftLickOccuring || RightLickOccuring) {
      digitalWrite(LeftCueIndicatorPin, LOW);
      digitalWrite(RightCueIndicatorPin, LOW);
      noTone(SpeakerPin);
      NextState = CUE_PENALTY;
    }

    if (millis() - CueStartTime >= ToneDuration) {
      digitalWrite(LeftCueIndicatorPin, LOW);
      digitalWrite(RightCueIndicatorPin, LOW);
      noTone(SpeakerPin);
      firstTrial = false;  // for sure not first trial anymore
      NextState = SELECTION;
    }

    //   ----------------------------   CUE_PENALTY   ----------------------------
    // - wait CuePenaltyDuration, then enter ENL
  } else if (CurrentState == CUE_PENALTY) {
    if (enteredNewState) {
      PenaltyStartTime = millis();
    }
    if (millis() - PenaltyStartTime >= CuePenaltyDuration) {
      if (!LeftLickOccuring && !RightLickOccuring) {
        NextState = ENL;
      }
    }

    //   ----------------------------   SELECTION   ----------------------------
    // - wait for lick or timeout
    // - initiate reward delivery if correct lick
    // - move on to CONSUMPTION upon lick or timeout
  } else if (CurrentState == SELECTION) {
    if (enteredNewState) {
      SelectionStartTime = millis();
    }
    bool correctSelection = false;

    // // check for opto stim in next ENL independent of trial outcome (OptoActiveDuringENL==true and numCorrectTrialsBeforeOpto==0)
    // if ((OptoActiveDuringENL == true) && (numCorrectTrialsBeforeOpto == 0) && (optoBLockDone == false)) {
    //   // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
    //   if (random(100) < OptoStimProb) {
    //     optoTriggeredENL = true;
    //     if (!AllowMultiOptoStimBlock) {
    //       optoBlockDone = true;
    //     }
    //   }
    // }

    if (LeftLickOccuring && BlockType == LEFT_BLOCK) {
      correctSelection = true;
      outputEvent("# CORRECT LEFT", millis(), 0);
    } else if (RightLickOccuring && BlockType == RIGHT_BLOCK) {
      correctSelection = true;
      outputEvent("# CORRECT RIGHT", millis(), 0);
    }
    if (correctSelection) {
      // CORRECT TRIAL
      failCounter = 0;
      CorrectTrialCount = CorrectTrialCount + 1;

      if (lastTrial) {
        winStayCounter++;
      } else {
        loseSwitchCounter++;
      }
      lastTrial = true;

      // send message to MATLAB
      Serial.print("Reward in block\t");
      Serial.print(CorrectTrialCount);
      Serial.print("\n");

      // check for opto stim during correct selection (reward)
      if ((OptoActiveDuringConsumption == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          // just stimulate whenever possible
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredOutcome = true;
            outputEvent("Opto Reward", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredOutcome = true;
            outputEvent("Opto Reward", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }

      NextState = PRE_REWARD;
    } else if (millis() - SelectionStartTime >= ReactionTime) {
      // TIMEOUT
      outputEvent("# Timeout", millis(), 0);
      failCounter += 1;  // Count omission as a fail
      Num_Omissions += 1;
      if (TrialsUntilFirstReward != 0) {
        badTrialsCounter++;
      }
      NextState = NEW_TRIAL;
    } else if (LeftLickOccuring) {
      // INCORRECT SELECTION
      outputEvent("# Incorrect LEFT", millis(), 0);
      failCounter += 1;  // Count incorrect lick as a fail
      incorrectCounter += 1;
      if (lastTrial) {
        if (TrialsUntilFirstReward == 0) {
          winStayCounter++;
        } else {
          winSwitchCounter++;
          badTrialsCounter++;
        }
      } else {
        loseStayCounter++;
        if (TrialsUntilFirstReward != 0) {
          badTrialsCounter++;
        }
      }
      lastTrial = false;
      NextState = CONSUMPTION;
    } else if (RightLickOccuring) {
      // INCORRECT SELECTION
      outputEvent("# Incorrect RIGHT", millis(), 0);
      failCounter += 1;  // Count incorrect lick as a fail
      incorrectCounter += 1;
      if (lastTrial) {
        if (TrialsUntilFirstReward == 0) {
          winStayCounter++;
        } else {
          winSwitchCounter++;
          badTrialsCounter++;
        }
      } else {
        loseStayCounter++;
        if (TrialsUntilFirstReward != 0) {
          badTrialsCounter++;
        }
      }
      lastTrial = false;
      NextState = CONSUMPTION;
    }


    //   ----------------------------   PRE_REWARD   ----------------------------
    // - wait for lick to end before starting reward delivery
  } else if (CurrentState == PRE_REWARD) {
    if (enteredNewState) {
      if (giveReward) {
        NextState = REWARD;
        BlockRewardCounter += 1;
      } else {
        NextState = NO_REWARD;
        BlockNoRewardCounter += 1;
      }
    }
    // if (!LeftLickOccuring && !RightLickOccuring) {
    //   NextState = REWARD;
    // }

    //   ----------------------------  NO REWARD   ----------------------------
    // - if correct lick made but no reward given (reward probability < 1.00)
  } else if (CurrentState == NO_REWARD) {
    if (BlockType == LEFT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardSizeLeft) {
        digitalWrite(LeftRewardPin, LOW);
        digitalWrite(LeftRewardIndicatorPin, LOW);
        NextState = CONSUMPTION;
      }
    } else if (BlockType == RIGHT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardSizeRight) {
        digitalWrite(RightRewardPin, LOW);
        digitalWrite(RightRewardIndicatorPin, LOW);
        NextState = CONSUMPTION;
      }
    }
    //   ----------------------------  FREE REWARD   ----------------------------
    // - open L or R reward valve for correct duration
  } else if (CurrentState == FREE_REWARD) {
    if (enteredNewState) {
      if (BlockType == LEFT_BLOCK) {
        digitalWrite(LeftRewardPin, HIGH);
        digitalWrite(LeftRewardIndicatorPin, HIGH);
      } else {
        digitalWrite(RightRewardPin, HIGH);
        digitalWrite(RightRewardIndicatorPin, HIGH);
      }
      RewardStartTime = millis();
    }

    if (BlockType == LEFT_BLOCK && (millis() - RewardStartTime) > RewardSizeLeft) {
      digitalWrite(LeftRewardPin, LOW);
      digitalWrite(LeftRewardIndicatorPin, LOW);
      NextState = CONSUMPTION;
    } else if (BlockType == RIGHT_BLOCK && (millis() - RewardStartTime) > RewardSizeRight) {
      digitalWrite(RightRewardPin, LOW);
      digitalWrite(RightRewardIndicatorPin, LOW);
      NextState = CONSUMPTION;
    }
    //   ----------------------------   REWARD   ----------------------------
  } else if (CurrentState == REWARD) {
    if (enteredNewState) {
      if (BlockType == LEFT_BLOCK) {
        digitalWrite(LeftRewardPin, HIGH);
        digitalWrite(LeftRewardIndicatorPin, HIGH);
        Num_Reward = Num_Reward + 1;
      } else {
        digitalWrite(RightRewardPin, HIGH);
        digitalWrite(RightRewardIndicatorPin, HIGH);
        Num_Reward = Num_Reward + 1;
      }
      RewardStartTime = millis();
      if (TrialsUntilFirstReward == 0) {
        TrialsUntilFirstReward = TrialInBlock;
        TrialsUntilFirstRewardHistory[BlockNum - 1] = TrialsUntilFirstReward;
      }
    }
    if (BlockType == LEFT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardSizeLeft) {
        digitalWrite(LeftRewardPin, LOW);
        digitalWrite(LeftRewardIndicatorPin, LOW);
        NextState = CONSUMPTION;
        // Send message to MATLAB
        Serial.print("Total Rewards\t");
        Serial.print(Num_Reward);
        Serial.print("\n");
      }
    } else if (BlockType == RIGHT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardSizeRight) {
        digitalWrite(RightRewardPin, LOW);
        digitalWrite(RightRewardIndicatorPin, LOW);
        NextState = CONSUMPTION;
        // Send message to MATLAB
        Serial.print("Total Rewards\t");
        Serial.print(Num_Reward);
        Serial.print("\n");
      }
    }
    if (optoTriggeredOutcome && OptoActiveDuringConsumption) {
      optoTriggeredOutcome = false;
      startOptoStim();
      outputEvent("Opto Outcome", millis(), OptoDuration);
    }

    //   ----------------------------   CONSUMPTION   ----------------------------
    // - wait for ConsumptionDuration
    // - turn off reward if necessary
  } else if (CurrentState == CONSUMPTION) {
    if (enteredNewState) {
      ConsumptionStartTime = millis();
    }

    if (millis() - ConsumptionStartTime >= ConsumptionDuration) {
      if (failCounter >= fails2reward) {  // Give free reward if threshold is met
        NextState = FREE_REWARD;
        failCounter = 0;
        freeRewardCounter += 1;
        freeRewardBlockCounter += 1;
        Serial.print("Free Rewards\t");
        Serial.print(freeRewardCounter);
        Serial.print("\n");
      } else {
        NextState = NEW_TRIAL;
      }
    }


    //   ----------------------------   END of STATES   ----------------------------
  } else {
    // should never get here
    NextState = IDLE;
    digitalWrite(CameraTriggerPin, LOW);
  }
}

void updateCameraTrigger() {
  // 2ms pulses at 50 Hz
  unsigned long Now = millis();
  if (CurrentState != IDLE) {
    if (((Now - CameraTrigStartTime) % 20) == 0 || ((Now - CameraTrigStartTime) % 20) == 1) {
      digitalWrite(CameraTriggerPin, HIGH);
    } else {
      digitalWrite(CameraTriggerPin, LOW);
    }
  }
}

void checkButtons() {
  // TODO
}

void checkSerialInput() {

  if (Serial.available() > 0) {
    // read the incoming byte
    SerialInput = Serial.read();

    // Handle inputs 1, 2, 8, and 9
    if (SerialInput == '1' && LeftRewardButtonStatus == 0) {
      CueStartTime = millis();
      LeftRewardButtonStatus = 1;
      digitalWrite(LeftRewardPin, HIGH);
      digitalWrite(LeftRewardIndicatorPin, HIGH);
      LeftRewardTimer = millis();

      outputEvent("MANUAL_LEFT_REWARD", CueStartTime, RewardSizeLeft);
    }

    else if (SerialInput == '2' && RightRewardButtonStatus == 0) {
      CueStartTime = millis();
      RightRewardButtonStatus = 1;
      digitalWrite(RightRewardPin, HIGH);
      digitalWrite(RightRewardIndicatorPin, HIGH);
      RightRewardTimer = millis();

      outputEvent("MANUAL_RIGHT_REWARD", CueStartTime, RewardSizeRight);
    }

    else if ((SerialInput == '8') || (buttonsAreWiredUp && (digitalRead(StartStopButtonPin) == HIGH))) {
      sessionStart = true;   // variable to keep track of task running
      TimerSync = millis();  // start sync pulse
      digitalWrite(CameraTriggerPin, LOW);
      CameraTrigStartTime = millis();

      digitalWrite(StartStopIndicatorPin, HIGH);
      Serial.print("TASK STARTED AT");
      Serial.print("\t");
      Serial.println(millis());

      if (spoutStart == 0) {
        randomBlock();
      } else if (spoutStart == 1) {  // start on left block
        BlockType = RIGHT_BLOCK;
      } else if (spoutStart == 2) {  // start on right block
        BlockType = LEFT_BLOCK;
      }

      fillBlockLengths();
      fillENLDuration();
      fillLeftRewardSizes();
      fillRightRewardSizes();
      fillRewardProbList();
      NextState = NEW_TRIAL;
    }

    else if (SerialInput == '9') {
      sessionStart = false;  // variable to keep track of task running
      sessionEnd = true; // make syncPulse low
      digitalWrite(StartStopIndicatorPin, HIGH);
      NextState = IDLE;
      // stop task
      Serial.print("TASK ENDED AT");
      Serial.print("\t");
      Serial.println(millis());
      if (TrialInBlock > 1) {
        printEndOfSessionStats();
      }
    }

    // This code adapted from Ofer's readFromUSB function
    // Will accept messages coming as inputs that can be parsed later
    else {
      static String usbMessage = "";  // initialize usbMessage to empty string

      // Start building the message from the current input
      usbMessage += SerialInput;

      // Continue reading characters until end of message
      while (Serial.available() > 0) {
        char inByte = Serial.read();
        if ((inByte == '\n') || (inByte == ';')) {
          // Message is complete, interpret it
          interpretUSBMessage(usbMessage);
          usbMessage = "";  // clear message buffer
        } else {
          // Append character to message buffer
          usbMessage += inByte;
        }
      }
    }

    // if (SerialInput == '3') {
    //   Serial.println("Entered 3: ShutterBlue on");
    //   digitalWrite(LaserPin, HIGH);
    //   //delay(2000);
    //   //digitalWrite(ShutterBlue, LOW);
    // }

    //  if (SerialInput == '4') {
    //   Serial.println("Entered 4: ShutterBlue off");
    //   digitalWrite(LaserPin, LOW);
    //   //delay(2000);
    //   //digitalWrite(ShutterBlue, LOW);
    // }

    // if (SerialInput == '5') {
    //   Serial.println("Entered 5: OptoStim Enabled");
    //   LaserEnabled = true;
    // }

    // if (SerialInput == '6') {
    //   Serial.println("Entered 6: OptoStim Disabled");
    //   LaserEnabled = false;
    // }

    if (SerialInput == 'T') {
      Serial.println("");
      Serial.println("** Loop Timing **");
      Serial.print(" Mean Loop: ");
      Serial.print(cumulativeLoopTime_us / loopCounter);
      Serial.println(" us");
      Serial.print(" Max Loop: ");
      Serial.print(maxLoopTime_us);
      Serial.println(" us");
      Serial.println("");
      maxLoopTime_us = 0;
      cumulativeLoopTime_us = 0;
      loopCounter = 0;
      loopStartTime_us = micros();  // cancel out this loop
    }

    if (SerialInput == 'S') {
      outputTrialState();
    }
  }
  //Left StartStopButtonPin Reward
  // if (digitalRead(LeftRewardButtonPin) == 1 && LeftRewardButtonStatus == 0) {
  //   LeftRewardButtonStatus = 1;
  //   digitalWrite(LeftRewardPin, HIGH);
  //   digitalWrite(LeftRewardIndicatorPin, HIGH);
  //   LeftRewardTimer = millis();
  // }
  if ((millis() - LeftRewardTimer) > RewardSizeLeft && LeftRewardButtonStatus == 1) {
    digitalWrite(LeftRewardPin, LOW);
    digitalWrite(LeftRewardIndicatorPin, LOW);
    LeftRewardButtonStatus = 0;

    // Serial.print("Manual reward L");
    // Serial.print("\t");
    // Serial.print(LeftRewardTimer);
    // Serial.print("\t");
    // Serial.println(RewardSize);
    //delay(4000); //suspend trial for a delay
  }

  //Right button Reward
  // if (digitalRead(RightRewardButtonPin) == 1 && RightRewardButtonStatus == 0) {
  //   RightRewardButtonStatus = 1;
  //   digitalWrite(RightRewardPin, HIGH);
  //   digitalWrite(RightRewardIndicatorPin, HIGH);
  //   RightRewardTimer = millis();
  // }
  if ((millis() - RightRewardTimer) > RewardSizeRight && RightRewardButtonStatus == 1) {
    digitalWrite(RightRewardPin, LOW);
    digitalWrite(RightRewardIndicatorPin, LOW);
    RightRewardButtonStatus = 0;

    // Serial.print("Manual reward R");
    // Serial.print("\t");
    // Serial.print(RightRewardTimer);
    // Serial.print("\t");
    // Serial.println(RewardSize);
    //delay(4000); //suspend trial for a delay
  }
}


// Generate sync pulses (100 ms pulses with random inter-pulse-interval)
void updateSyncPulses() {
  if (sessionStart == true) {
    if (millis() - TimerSync > SyncPulseInterval) {
      TimerSync = millis();
      if (SyncPulseStatus == 1) {
        digitalWrite(SyncPin, LOW);
        SyncPulseStatus = 0;
        SyncPulseInterval = 100 + 100 * random(5);  // random sync pulse interval between 100~600 ms in steps of 100
        Serial.print("Sync off\t");
        Serial.print(millis());
        Serial.print("\t");
        Serial.println(SyncPulseInterval);
      } else {
        digitalWrite(SyncPin, HIGH);
        SyncPulseStatus = 1;
        SyncPulseInterval = 100;  // sync pulse HIGH for 100 ms
        Serial.print("Sync on\t");
        Serial.print(millis());
        Serial.print("\t");
        Serial.println(SyncPulseInterval);
      }
    }
  } else if (sessionEnd == true) {
        // force camera ttl low
        digitalWrite(SyncPin, LOW);
        Serial.print("Sync off\t");
        Serial.print(millis());
        Serial.print("\t");
        Serial.println(SyncPulseInterval);
        // stop if statement
        sessionEnd = false; 
  }
}

// Increment trial count and update block
void incrementTrial() {
  // Update trial number
  optoTrialDone = false;
  TrialNum = TrialNum + 1;
  TrialInBlock = TrialInBlock + 1;
  int LeftRewardIdx = random(numLeftRewardSizes);
  RewardSizeLeft = LeftRewardSizes[LeftRewardIdx];
  int RightRewardIdx = random(numRightRewardSizes);
  RewardSizeRight = RightRewardSizes[RightRewardIdx];

  int randomRewardIdx = random(100);
  int RewardMaybe = RewardProbList[randomRewardIdx];
  if (RewardMaybe == 1) {
    giveReward = true;
  } else {
    giveReward = false;
  }

  // Switching to new block
  if ((CorrectTrialCount >= BlockLength) || (TrialNum == 1)) {
    // Print stats for previous block
    if (TrialNum > 1) {
      printEndOfBlockStats();
      optoBlockFirstTrial = true;  // opto first trial in block
    }
    
    // Decide block length
    int BlockLengthIdx = random(numBlockLengths);
    BlockLength = BlockLengths[BlockLengthIdx];

    BlockNum = BlockNum + 1;
    TrialInBlock = 1;
    TrialsUntilFirstReward = 0;
    TrialsUntilFirstRewardHistory[BlockNum - 1] = 0;
    CorrectTrialCount = 0;
    incorrectCounter = 0;
    Num_ENLPenalty = 0;
    Num_Omissions = 0;
    freeRewardBlockCounter = 0;
    BlockRewardCounter = 0;
    BlockNoRewardCounter = 0;
    leftLickCounter = 0;
    rightLickCounter = 0;
    winStayCounter = 0;
    loseSwitchCounter = 0;
    winSwitchCounter = 0;
    loseStayCounter = 0;
    badTrialsCounter = 0;
    lastTrial = true;
    optoBlockDone = false;  // reset opto in block

    // switch BlockType
    if (BlockType == RIGHT_BLOCK) {
      BlockType = LEFT_BLOCK;
    } else if (BlockType == LEFT_BLOCK) {
      BlockType = RIGHT_BLOCK;
    }

    // Send message to MATLAB
      Serial.print("Out of\t");
      Serial.print(BlockLength);
      Serial.print("\n");
      blockLengthSent = true;
  }
}

// Print trial Statements
void printDataHeader() {
  Serial.println("TrialNum\tBlockNum\tTrialInBlock\tCorrectTrialCount\tEvent\tEventTime\tLickDuration");
}

void outputTrialState() {
  if (CurrentState == PRE_REWARD) {
    // no need to indicate pre-reward state
    return;
  }
  const char* stateName = "";
  unsigned long duration = 0;
  if (CurrentState == IDLE) {
    stateName = "IDLE";
  } else if (CurrentState == NEW_TRIAL) {
    stateName = "NEW_TRIAL";
  } else if (CurrentState == ENL) {
    if (BlockType == RIGHT_BLOCK) {
      stateName = "ENL_RIGHT";
    } else {
      stateName = "ENL_LEFT";
    }
    // stateName = "ENL";
    duration = ENLDuration;
  } else if (CurrentState == ENL_PENALTY) {
    stateName = "ENL_PENALTY";
    duration = ENLPenaltyDuration;
  } else if (CurrentState == CUE) {
    if (BlockType == RIGHT_BLOCK) {
      stateName = "CUE_RIGHT";
    } else {
      stateName = "CUE_LEFT";
    }
    duration = ToneDuration;
  } else if (CurrentState == CUE_PENALTY) {
    stateName = "CUE_PENALTY";
    duration = CuePenaltyDuration;
  } else if (CurrentState == SELECTION) {
    stateName = "SELECTION";
    duration = ReactionTime;
  } else if (CurrentState == REWARD) {
    stateName = "REWARD";
    if (BlockType == RIGHT_BLOCK) {
      duration = RewardSizeRight;
    } else {
      duration = RewardSizeLeft;
    }
  } else if (CurrentState == NO_REWARD) {
    stateName = "NO_REWARD";
    duration = 0;
  } else if (CurrentState == FREE_REWARD) {
    stateName = "FREE_REWARD";
  } else if (CurrentState == CONSUMPTION) {
    stateName = "CONSUMPTION";
    duration = ConsumptionDuration;
  }
  outputEvent(stateName, millis(), duration);
}

void outputEvent(const char* eventName, unsigned long startTime, unsigned long duration) {
  pausePrint = true;
  // unsigned long pTime = micros();
  serialOutBuffer += TrialNum;
  serialOutBuffer += '\t';
  serialOutBuffer += BlockNum;
  serialOutBuffer += '\t';
  serialOutBuffer += TrialInBlock;
  serialOutBuffer += '\t';
  serialOutBuffer += CorrectTrialCount;
  serialOutBuffer += '\t';
  serialOutBuffer += eventName;
  serialOutBuffer += '\t';
  serialOutBuffer += startTime;
  serialOutBuffer += '\t';
  serialOutBuffer += duration;
  serialOutBuffer += '\n';
  // Serial.print(tmpserialOutBuffer.c_str());
  // Serial.println(micros() - pTime);

  // TIMING tested by OM 2022-10-04
  // - 250-450us  building up the string
  // - ~250us     printing the string to Serial
}

const unsigned int MAX_CHARS_PER_LOOP = 24;
char tempBuffer[MAX_CHARS_PER_LOOP + 10];
void updateSerialOutput() {
  // // FOR DEBUGGING
  // Serial.print(serialOutBuffer.c_str());
  // serialOutBuffer = "";
  // return;


  // Spread Serial output over multiple loop iterations by printing
  // no more than MAX_CHARS_PER_LOOP characters at a time.
  if (pausePrint) {
    // don't print anything on iterations where pausePrint flag is set
    // (should be set when long string is written to serialOutBuffer)
    pausePrint = false;
    return;
  }
  if (serialOutBuffer.length() > 0) {
    unsigned int numCharToPrint = min(MAX_CHARS_PER_LOOP, serialOutBuffer.length());
    serialOutBuffer.toCharArray(tempBuffer, numCharToPrint + 1);
    Serial.print(tempBuffer);
    // Serial.print("["); // for debugging
    // Serial.print(numCharToPrint); // for debugging
    // Serial.print("]"); // for debugging
    serialOutBuffer.remove(0, numCharToPrint);  // remove the chars we just printed
  }
}


void printEndOfSessionStats() {
  pausePrint = true;

  serialOutBuffer += "\nBlock Stats\t";
  serialOutBuffer += BlockNum;
  serialOutBuffer += "\n";
  serialOutBuffer += "Block Num\t"; serialOutBuffer += BlockNum; serialOutBuffer += "\t";
  serialOutBuffer += "Block\t";
  if (BlockType == RIGHT_BLOCK) {
    serialOutBuffer += "RIGHT\t";
  } else {
    serialOutBuffer += "LEFT\t";
  }
  serialOutBuffer += "Length\t";
  serialOutBuffer += BlockLength;
  serialOutBuffer += '\t';
  serialOutBuffer += "Trials\t";
  serialOutBuffer += (TrialInBlock - 1);
  serialOutBuffer += '\t';
  serialOutBuffer += "Correct\t";
  serialOutBuffer += CorrectTrialCount;
  serialOutBuffer += '\t';
  serialOutBuffer += "Incorrect\t";
  serialOutBuffer += incorrectCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "Rewards\t";
  serialOutBuffer += BlockRewardCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "No Rewards\t";
  serialOutBuffer += BlockNoRewardCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "ENL Penalties\t";
  serialOutBuffer += Num_ENLPenalty;
  serialOutBuffer += "\t";
  serialOutBuffer += "Omissions\t";
  serialOutBuffer += Num_Omissions;
  serialOutBuffer += "\t";
  serialOutBuffer += "Free Rewards\t";
  serialOutBuffer += freeRewardBlockCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Left licks\t";
  serialOutBuffer += leftLickCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Right licks\t";
  serialOutBuffer += rightLickCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Win Stays\t";
  serialOutBuffer += winStayCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Lose Switches\t";
  serialOutBuffer += loseSwitchCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Win Switches\t";
  serialOutBuffer += winSwitchCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Lose Stays\t";
  serialOutBuffer += loseStayCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Bad Trials Post Reward\t";
  serialOutBuffer += badTrialsCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Trials To 1st Reward\t";
  serialOutBuffer += TrialsUntilFirstReward;
  serialOutBuffer += "\n\n";
  // statsString += "=====================================\n\n";
  // Serial.print(statsString.c_str());
  // TIMING (tested by OM 2022-10-04)
  // ~1.5 ms
}

void printEndOfBlockStats() {
  pausePrint = true;
  serialOutBuffer += "\nBlock Stats\t";
  serialOutBuffer += BlockNum;
  serialOutBuffer += "\n";
  serialOutBuffer += "Block Num\t"; serialOutBuffer += BlockNum; serialOutBuffer += "\t";
  serialOutBuffer += "Block\t";
  if (BlockType == RIGHT_BLOCK) {
    serialOutBuffer += "RIGHT\t";
  } else {
    serialOutBuffer += "LEFT\t";
  }
  serialOutBuffer += "Length\t";
  serialOutBuffer += BlockLength;
  serialOutBuffer += '\t';
  serialOutBuffer += "Trials\t";
  serialOutBuffer += (TrialInBlock - 1);
  serialOutBuffer += '\t';  // NOTE I did trials - 1. It gets incremented in new trial, which is too early
  serialOutBuffer += "Correct\t";
  serialOutBuffer += CorrectTrialCount;
  serialOutBuffer += '\t';
  serialOutBuffer += "Incorrect\t";
  serialOutBuffer += incorrectCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "Rewards\t";
  serialOutBuffer += BlockRewardCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "No Rewards\t";
  serialOutBuffer += BlockNoRewardCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "ENL Penalties\t";
  serialOutBuffer += Num_ENLPenalty;
  serialOutBuffer += "\t";
  serialOutBuffer += "Omissions\t";
  serialOutBuffer += Num_Omissions;
  serialOutBuffer += "\t";
  serialOutBuffer += "Free Rewards\t";
  serialOutBuffer += freeRewardBlockCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Left licks\t";
  serialOutBuffer += leftLickCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Right licks\t";
  serialOutBuffer += rightLickCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Win Stays\t";
  serialOutBuffer += winStayCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Lose Switches\t";
  serialOutBuffer += loseSwitchCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Win Switches\t";
  serialOutBuffer += winSwitchCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Lose Stays\t";
  serialOutBuffer += loseStayCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Bad Trials Post Reward\t";
  serialOutBuffer += badTrialsCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Trials To 1st Reward\t";
  serialOutBuffer += TrialsUntilFirstReward;
  serialOutBuffer += "\n\n";
  // serialOutBuffer += "Tot. rewards: "; serialOutBuffer += Num_Reward; serialOutBuffer += "\n\n\n";
  // Serial.print(statsString.c_str());
  // TIMING (tested by OM 2022-10-04)
  // ~3 ms
}

void lickDetection() {
  // LEFT
  bool leftLickDetected = (digitalRead(LeftLickPin) == LOW);  // LOW
  if (leftLickDetected && !LeftLickOccuring) {
    // If start of new L lick detected:
    LeftLickOccuring = true;
    LeftLickStartTime = millis();
  } else if (!leftLickDetected && LeftLickOccuring) {
    // If end of L lick detected:
    LeftLickOccuring = false;
    unsigned long Lick_Duration = millis() - LeftLickStartTime;

    outputEvent("LickLeft", LeftLickStartTime, Lick_Duration);
    leftLickCounter += 1;
  }

  // RIGHT
  bool rightLickDetected = (digitalRead(RightLickPin) == LOW);  // LOW
  if (rightLickDetected && !RightLickOccuring) {
    // If start of new L lick detected:
    RightLickOccuring = true;
    RightLickStartTime = millis();
  } else if (!rightLickDetected && RightLickOccuring) {
    // If end of L lick detected:
    RightLickOccuring = false;
    unsigned long Lick_Duration = millis() - RightLickStartTime;

    outputEvent("LickRight", RightLickStartTime, Lick_Duration);
    rightLickCounter += 1;
  }
  // Lick Detection END //
}

// Call this to initiate a new opto stimulation
void startOptoStim() {
  if (LaserEnabled) {
    optoStartTime = millis();
    duringOptoStim = true;
    duringOptoStimDelay = true;
    outputEvent("Laser START", millis(), OptoDuration);
  }
}

// This should be called on a regular basis by loop().
// Turns the laser on/off as specified by the delay and duration paramters.
void updateOptoStim() {
  optoIPI = (1000 / optoFrequency) - optoPulseDuration;  // keep track of IPI
  // continuous
  if (duringOptoStim) {
    unsigned long timeInStim = millis() - optoStartTime;
    if (duringOptoStimDelay && (timeInStim > OptoDelay)) {
      // if we've past delay period, turn on laser
      digitalWrite(LaserPin, HIGH);
      duringOptoStimDelay = false;
      optoStartPulse = millis();
      optoPulseActive = true;
      outputEvent("Laser ON", millis(), OptoDuration);

    } else if (timeInStim > (OptoDelay + OptoDuration)) {
      // if we've reached the end of the opto stim duration, turn off laser
      digitalWrite(LaserPin, LOW);
      duringOptoStim = false;
      optoPulseActive = false;
      optoIPIActive = false;
      outputEvent("Laser OFF", millis(), OptoDuration);

    } else if (optoContinuous == false && optoPulseActive == true) {
      optoTimeInPulse = millis() - optoStartPulse;
      if (optoTimeInPulse > optoPulseDuration) {
        // turn laser off after pulse duration has passed
        digitalWrite(LaserPin, LOW);
        optoPulseActive = false;
        optoIPIActive = true;
        optoStartPulse = millis();
      }

    } else if (optoContinuous == false && optoIPIActive == true) {
      optoTimeInPulse = millis() - optoStartPulse;
      if (optoTimeInPulse > optoIPI) {
        // turn laser on after IPI has passed
        digitalWrite(LaserPin, HIGH);
        optoIPIActive = false;
        optoPulseActive = true;
        optoStartPulse = millis();
      }
    }
  }
}

void randomShutterSound() {
  if (millis() - TimerShutterSound >= ShutterSoundInterval) {
    if (ShutterSoundNow == 1) {
      TimerShutterSound = millis();
      digitalWrite(ShutterRed, LOW);
      ShutterSoundNow = 0;
      ShutterSoundInterval = 100 + random(1, 900);  // random opto interval between 0.1~1s
      //Serial.print("Next ShutterSound after ");
      //Serial.print(ShutterSoundInterval / 1000.0);
      //Serial.println("s");
    } else {
      TimerShutterSound = millis();
      digitalWrite(ShutterRed, HIGH);
      ShutterSoundNow = 1;
      ShutterSoundInterval = 500;
    }
  }
}

// Ofer's code. Commands will be sent from Dual_Lick_GUI
void interpretUSBMessage(String message) {

  // 1) Parse message into "<command> and <arg1>"
  // Where <command> is a single character, and <arg1> is a positive integer

  message.trim();  // remove leading and trailing white space
  int len = message.length();
  if (len == 0) {
    Serial.println("#");  // "#" means error, message does not exist
    return;
  }
  char command = message[0];  // the command is the first char of a message
  String parameters = message.substring(1);
  parameters.trim();

  // Handshake
  if (command == '^') {
    Serial.println('^');
    return;
  }

  String intString = "";
  while ((parameters.length() > 0) && (isDigit(parameters[0]))) {
    intString += parameters[0];
    parameters.remove(0, 1);
  }
  long arg1 = intString.toInt();

  // 2) Act on the command

  if (command == 'a') {  // Set min number of trials per block
    minBlock = arg1;
    fillBlockLengths();
  } else if (command == 'b') {  // Set max number of trials per block
    maxBlock = arg1;
    fillBlockLengths();
  } else if (command == 'c') {  // Set fails until reward
    fails2reward = arg1;
  } else if (command == 'd') {  // Set selection time
    ReactionTime = arg1;
  } else if (command == 'e') {  // Left cue
    CueStartTime = millis();
    tone(SpeakerPin, LeftCueFreq);
    digitalWrite(LeftCueIndicatorPin, HIGH);

    while (digitalRead(LeftCueIndicatorPin) == HIGH) {
      if (millis() - CueStartTime >= ToneDuration) {
        noTone(SpeakerPin);
        digitalWrite(LeftCueIndicatorPin, LOW);
      }
    }

    outputEvent("MANUAL_LEFT_CUE", CueStartTime, ToneDuration);
  } else if (command == 'f') {  // Right cue
    CueStartTime = millis();
    tone(SpeakerPin, LeftCueFreq);
    digitalWrite(RightCueIndicatorPin, HIGH);

    while (digitalRead(RightCueIndicatorPin) == HIGH) {
      if (millis() - CueStartTime >= ToneDuration) {
        noTone(SpeakerPin);
        digitalWrite(RightCueIndicatorPin, LOW);
      }
    }

    outputEvent("MANUAL_RIGHT_CUE", CueStartTime, ToneDuration);
  } else if (command == 'g') {  // Change left calibration size
    minLeft = arg1;
    maxLeft = arg1;
    stepLeft = 1;
    fillLeftRewardSizes();
    Serial.print("Min Left\t");
    Serial.println(minLeft);
    Serial.print("Max Left\t");
    Serial.println(maxLeft);
    Serial.print("Left Step\t");
    Serial.println(stepLeft);
    RewardSizeLeft = arg1;
  } else if (command == 'h') {  // Change right calibration size
    minRight = arg1;
    maxRight = arg1;
    stepRight = 1;
    fillRightRewardSizes();
    Serial.print("Min Right\t");
    Serial.println(minRight);
    Serial.print("Max Right\t");
    Serial.println(maxRight);
    Serial.print("Right Step\t");
    Serial.println(stepRight);
    RewardSizeRight = arg1;
  } else if (command == 'i') {  // Set min ENL
    minEnl = arg1;
    fillENLDuration();
  } else if (command == 'j') {  // Set max ENL
    maxEnl = arg1;
    fillENLDuration();
  } else if (command == 'k') {  // Set ENL step
    ENLstep = arg1;
    fillENLDuration();
  } else if (command == 'l') {  // Turn opto on
    LaserEnabled = true;
  } else if (command == 'm') {  // Turn opto off
    LaserEnabled = false;
  } else if (command == 'o') {  // spout start random
    spoutStart = 0;
  } else if (command == 'p') {  // spout start on left
    spoutStart = 1;
  } else if (command == 'q') {  // spout start on right
    spoutStart = 2;
  } else if (command == 'r') {  // min left reward size
    minLeft = arg1;
    fillLeftRewardSizes();
  } else if (command == 's') {  // max left reward size
    maxLeft = arg1;
    fillLeftRewardSizes();
  } else if (command == 't') {  // left reward step
    stepLeft = arg1;
    fillLeftRewardSizes();
  } else if (command == 'u') {  // min right reward size
    minRight = arg1;
    fillRightRewardSizes();
  } else if (command == 'v') {  // max right reward size
    maxRight = arg1;
    fillRightRewardSizes();
  } else if (command == 'w') {  // right reward step
    stepRight = arg1;
    fillRightRewardSizes();
  } else if (command == 'x') { // reward probability
    RewardProb = arg1;
    fillRewardProbList();
  } else if (command == 'A') { // opto continuous?
    if (arg1 == 1) {
      optoContinuous = true;
    } else {
      optoContinuous = false;
    }
  } else if (command == 'B') { // opto probability
    OptoStimProb = arg1;
  } else if (command == 'C') { // correct trials before opto
    numCorrectTrialsBeforeOpto = arg1;
  } else if (command == 'D') { // opto delay
    OptoDelay = arg1;
  } else if (command == 'E') { // opto duration
    OptoDuration = arg1;
  } else if (command == 'F') { // pulse duration
    optoPulseDuration = arg1;
  } else if (command == 'G') { // laser frequency
    optoFrequency = arg1;
  } else if (command == 'H') { // opto during block switch
    if (arg1 == 1) {
      OptoActiveDuringBlockSwitch = true;
    } else {
      OptoActiveDuringBlockSwitch = false;
    }
  } else if (command == 'I') { // opto during enl
    if (arg1 == 1) {
      OptoActiveDuringENL = true;
    } else {
      OptoActiveDuringENL = false;
    }
  } else if (command == 'J') { // opto during cue
    if (arg1 == 1) {
      OptoActiveDuringCue = true;
    } else {
      OptoActiveDuringCue = false;
    }
  } else if (command == 'K') { // opto during consumption
    if (arg1 == 1) {
      OptoActiveDuringConsumption = true;
    } else {
      OptoActiveDuringConsumption = false;
    }
  } else if (command == 'L') { // allow multi-stim in enl
    if (arg1 == 1) {
      AllowMultiOptoStimENL = true;
    } else {
      AllowMultiOptoStimENL = false;
    }
  } else if (command == 'M') { // allow multi-stim in block
    if (arg1 == 1) {
      AllowMultiOptoStimBlock = true;
    } else {
      AllowMultiOptoStimBlock = false;
    }
  } else if (command == 'N') { // set cue penalty duration
    CuePenaltyDuration = arg1;
  } else if (command == 'O') { // set left cue frequency
    LeftCueFreq = arg1;
  } else if (command == 'P') { // set right cue frequency
    RightCueFreq = arg1;
  } else if (command == 'y') { // to save memory, do not call until needed
    saveParams(); // will update EEPROM with current values for variables
  } else if (command == 'z') {  // Send current parameters. setup command
    Serial.print("Min Block\t"); Serial.println(minBlock);
    Serial.print("Max Block\t");  Serial.println(maxBlock);
    Serial.print("Reaction Time\t"); Serial.println(ReactionTime);
    Serial.print("Cue Penalty\t"); Serial.println(CuePenaltyDuration);
    Serial.print("Fails until reward\t"); Serial.println(fails2reward);
    Serial.print("Left Calibration\t"); Serial.println(RewardSizeLeft);
    Serial.print("Right Calibration\t"); Serial.println(RewardSizeRight);
    Serial.print("Min ENL\t");  Serial.println(minEnl);
    Serial.print("Max ENL\t"); Serial.println(maxEnl);
    Serial.print("ENL Step\t"); Serial.println(ENLstep);
    Serial.print("Min Left\t"); Serial.println(minLeft);
    Serial.print("Max Left\t"); Serial.println(maxLeft);
    Serial.print("Left Step\t"); Serial.println(stepLeft);
    Serial.print("Min Right\t"); Serial.println(minRight);
    Serial.print("Max Right\t"); Serial.println(maxRight);
    Serial.print("Right Step\t"); Serial.println(stepRight);
    Serial.print("Reward Prob\t"); Serial.println(RewardProb);
    Serial.print("Spout Start\t"); Serial.println(spoutStart);
    Serial.print("Left Cue Frequency\t"); Serial.println(LeftCueFreq);
    Serial.print("Right Cue Frequency\t"); Serial.println(RightCueFreq);
    Serial.print("Opto on\t"); Serial.println(LaserEnabled);
    Serial.print("Continuous\t"); Serial.println(optoContinuous);
    Serial.print("Opto Probability\t"); Serial.println(OptoStimProb);
    Serial.print("Correct Trials Before Opto\t"); Serial.println(numCorrectTrialsBeforeOpto);
    Serial.print("Opto Delay\t"); Serial.println(OptoDelay);
    Serial.print("Opto Duration\t"); Serial.println(OptoDuration);
    Serial.print("Pulse Duration\t"); Serial.println(optoPulseDuration);
    Serial.print("Laser Frequency\t"); Serial.println(optoFrequency);
    Serial.print("Opto In Block Switch\t"); Serial.println(OptoActiveDuringBlockSwitch);
    Serial.print("Opto In ENL\t"); Serial.println(OptoActiveDuringENL);
    Serial.print("Opto In Cue\t"); Serial.println(OptoActiveDuringCue);
    Serial.print("Opto In Consumption\t"); Serial.println(OptoActiveDuringConsumption);
    Serial.print("Multi-stim In ENL\t"); Serial.println(AllowMultiOptoStimENL);
    Serial.print("Multi-stim In Block\t"); Serial.println(AllowMultiOptoStimBlock);
  }
}

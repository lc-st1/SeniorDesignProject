// Automatic Guitar Tuner Arduino Main Code
// Luca Ciampaglia, Daniel French, Andrew Li
// ECSE 398 Senior Design Project


// Include any necessary libraries
#include "LiquidCrystal.h"
#include "megaAVR_PWM.h"


/* NANO EVERY PINOUT:
D2    Frequency Input Pin     (To Butterworth Filter)
D3    Filter Clock Pin        (From Comparator)
D4    Stepper Direction Pin   (To Stepper Driver)
D5    Stepper Step Pin        (To Stepper Driver)
D6    Microstep Pin 1         (To Stepper Driver)
D7    Microstep Pin 2         (To Stepper Driver)
D8    Microstep Pin 3         (To Stepper Driver)
D9    Menu Button Left        (To Pulled Down Button)
D10   Menu Button Enter       (To Pulled Down Button)
D11   Menu Button Right       (To Pulled Down Button)
D12   Menu Button Back        (To Pulled Down Button)


VIN   9V Input Voltage        (From 9V Battery)
GND   Ground                  (From 9V Battery)
GND   Ground                  (To Stepper Driver, LCD, Buttons)
5V    Logic Supply            (To Stepper Driver, LCD, Buttons)


D13
D14   LCD D4                  (To LCD Screen)
D15   LCD D5                  (To LCD Screen)
D16   LCD D6                  (To LCD Screen)
D17   LCD D7                  (To LCD Screen)
D18   I2C SDA                      
D19   I2C SCL
D20   LCD RS                  (To LCD Screen)
D21   LCD E                   (To LCD Screen)

*/



// FILTERING VARIABLES
megaAVR_PWM* filterPWM;                   // PWM Generator Instance (for filter clock signal)
float filterFreq;                         // Filter frequency (100 * corner freq)
float dutyCycle = 50;                     // Filter clock duty cycle (leave at 50)
const int filterPin = 3;                  // Filter Tuning Pin (Outputs a Clock Signal for the Butterworth Chip)

// FREQUENCY ANALYSIS VARIABLES
const int freqPin = 2;                    // Frequency Detection Pin (Attached Interrupt)
const int maxPulses = 10;                 // Max Pulses to Count for Freq Analysis
volatile int pulseCount = 0;              // Counted Pulses 
volatile long pulseStartTime = 0;         // Start micros of Pulses
volatile bool newMeasurement = false;     // New Frequency Measurement check
volatile long pulseEndTime = 0;           // End micros of pulses
double frequency = 0;                     // Current Average freq
double stringFrequency = 0;               // Target Frequency
double freqErr = 0;                       // Difference in Averaged Freq and Target Freq
bool inTune = false;                      // Status of Current String Tuning
int curString = 0;                        // Current String (0-5 for Low-High)

// MOTOR CONTROL VARIABLES
//megaAVR_PWM* stepperPWM;                  // PWM Generator for Step Control
const int dirPin = 4;                     // Direction Pin of the Motor Driver (HIGH Clockwise)
const int enablePin = 5;
const int stepPin = 6;                    // Step Pin of the Motor Driver (1 Step per Rising Edge)
const int microstepPins[] = { 5, 7, 8 };  // Microstepping Pins (See Motor Driver for more info)
const int stepsPerRevolution = 200;       // NEMA 17 Step Count
int microstepSize = 4;                    // Fractional Microstepping Factor (1/n)
int stepErr = 0;                  // Stepper motor PWM Frequency
bool stepped = 0;                         // Alternating Step
int steps = 0;                            // Steps taken so far
int motorDirection = 1;                   // Current Direction of Motor
const float alpha = 100;                   // Proportional Motor Control Coefficient
float freqAvg = 0;
float freqWindow[20];

// LCD PINS
const int lcdD4Pin = 14;                  // LCD Data 4 Pin
const int lcdD5Pin = 15;                  // LCD Data 5 Pin
const int lcdD6Pin = 16;                  // LCD Data 6 Pin
const int lcdD7Pin = 17;                  // LCD Data 7 Pin
const int lcdRSPin = 20;                  // LCD Register Select Pin
const int lcdEPin = 21;                   // LCD Enable Pin
LiquidCrystal lcd(lcdRSPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);                 // The LCD Object

// MENU VARIABLES
const int buttonLeft = 9;                 // Left Button
const int buttonEnter = 10;               // Select Button
const int buttonRight = 11;               // Right Button
const int buttonBack = 12;                // Back Button
int selectedTuning = 0;                   // Currently selected tuning from menu
int displayedTuning = 0;                  // Tuning currently displayed
bool tuningSelected = false;              // If a tuning is currently selected or not
volatile bool newDisplay = true;         // New Display data
int numTunings = 0;                       // Number of tunings in library
int machineState = 0;                     // State Machine State (0 = Menu, 1 = Confirm Tuning, 2 = String Setup, 3 = Tuning)
unsigned long timer = 0;
unsigned long lastTimer = 0;

// CUSTOM MENU CHARACTERS FOR LCD
byte leftArrow[] = {
  B00000,
  B00111,
  B01110,
  B11100,
  B01110,
  B00111,
  B00000,
  B00000
};
byte rightArrow[] = {
  B00000,
  B11100,
  B01110,
  B00111,
  B01110,
  B11100,
  B00000,
  B00000
};
byte sharp[] = {
  B00000,
  B00010,
  B01011,
  B01110,
  B11010,
  B01111,
  B11010,
  B01000
};
byte flat[] = {
  B01000,
  B01000,
  B01000,
  B01000,
  B01110,
  B01010,
  B01100,
  B01000
};
// TUNING DATA
// read a table or store a list of possible tunings (name of tuning and all 6 string freqs)
// https://fretsuccess.com/what-are-the-guitar-string-frequencies/
// https://guitargearfinder.com/guides/alternate-tunings/
// https://pages.mtu.edu/~suits/notefreqs.html
const String tuningNames[] = {
  "Standard", 
  "Drop D", 
  "Drop C",
  "Step Down D",
  "Open E",
  "Open D",
  "Open G"
  };
const String tuningNotes[][6] = {
  {"E2",    "A2",   "D3",   "G3",   "B3",   "E4"},
  {"D2",    "A2",   "D3",   "G3",   "B3",   "E4"},
  {"C2",    "G2",   "C3",   "F3",   "A3",   "D4"},
  {"D2",    "G2",   "C3",   "F3",   "A3",   "D4"},
  {"E2",    "B2",   "E3",   "G#3",  "B3",   "E4"},
  {"D2",    "A2",   "D3",   "F#3",  "A3",   "D4"},
  {"D2",    "G2",   "D3",   "G3",   "B3",   "D4"}
  };
const float tuningFrequencies[][6] = {
  {82.41,   110.0,    146.83,   196.00,   246.94,   329.63},
  {73.42,   110.0,    146.83,   196.00,   246.94,   329.63},
  {65.41,   98.00,    130.81,   174.61,   220.00,   293.66},
  {73.42,   98.00,    130.81,   174.61,   220.00,   293.66},
  {82.41,   123.47,   164.81,   207.65,   246.94,   329.63},
  {73.42,   110.00,   146.83,   185.00,   220.00,   293.66},
  {73.42,   98.00,    146.83,   196.00,   246.94,   293.66}
  };



void setup() {
  // FILTER CONTROL SETUP
  filterPWM = new megaAVR_PWM(filterPin, 1000, 0);
  
  // FREQUENCY ANALYSIS SETUP 
  interrupts();
  pinMode(freqPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(freqPin), measurePeriod, RISING);

  // STEPPER CONTROLLER SETUP
  //stepperPWM = new megaAVR_PWM(stepPin, 0, 0);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(microstepPins[0], OUTPUT);
  pinMode(microstepPins[1], OUTPUT);
  pinMode(microstepPins[2], OUTPUT);
  for(int i =0; i<20; i++){
    freqWindow[i] =0.0;
  }

  // LCD SETUP
  pinMode(lcdD4Pin, OUTPUT);
  pinMode(lcdD5Pin, OUTPUT);
  pinMode(lcdD6Pin, OUTPUT);
  pinMode(lcdD7Pin, OUTPUT);
  pinMode(lcdRSPin, OUTPUT);
  pinMode(lcdEPin, OUTPUT);
  lcd.begin(16,2);
  lcd.clear();
  lcd.createChar(0, leftArrow);
  lcd.createChar(1, rightArrow);
  lcd.createChar(2, sharp);
  lcd.createChar(3, flat);

  // MENU SETUP
  attachInterrupt(digitalPinToInterrupt(buttonLeft), leftPress, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonEnter), enterPress, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonRight), rightPress, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonBack), backPress, RISING);
  numTunings = int(sizeof(tuningNames)/sizeof(tuningNames[0]));


  // DEBUG
  Serial.begin(9600);
}


void loop() {
  // MAIN PROGRAM STATES:
  /*
    IDLE: device on main menu, tuning not selected yet. can scroll between tuning choices and select one. Once selected, switch to first STRING state.
    STRING 1: tuning selected, begin tuning the lowest string. switch to TUNING state.
    TUNING: tuning a selected string, measure the frequency and adjust the motor accordingly. switch to next STRING state
    STRING 2: tuning selected, begin tuning the next string. switch to TUNING state.
    TUNING: tuning a selected string, measure the frequency and adjust the motor accordingly. switch to next STRING state
    STRING 3: tuning selected, begin tuning the next string. switch to TUNING state.
    TUNING: tuning a selected string, measure the frequency and adjust the motor accordingly. switch to next STRING state
    STRING 4: tuning selected, begin tuning the next string. switch to TUNING state.
    TUNING: tuning a selected string, measure the frequency and adjust the motor accordingly. switch to next STRING state
    STRING 5: tuning selected, begin tuning the next string. switch to TUNING state.
    TUNING: tuning a selected string, measure the frequency and adjust the motor accordingly. switch to next STRING state
    STRING 6: tuning selected, begin tuning the next string. switch to TUNING state.
    TUNING: tuning a selected string, measure the frequency and adjust the motor accordingly. switch to FINISHED state
    FINISHED: return to the IDLE state
  */
  /*
    processes for each state:
    IDLE: filter Freq is 0, stepper freq is 0. interrupts OFF
    STRING 1-6: filter Freq is 0, stepper freq is 0. interrupts OFF
    TUNING: Filter freq is dependent on selected string, stepper freq is set to PID control. interrupts ON
  */
  switch (machineState){
    case 0:
      // let user navigate through library of tunings
      selectedTuning = displayedTuning;
      if(newDisplay){
        newDisplay=false;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(byte(0));
        lcd.setCursor(7-int(sizeof(tuningNames[displayedTuning])/2.0),0);
        lcd.print(tuningNames[displayedTuning]);
        lcd.setCursor(15,0);
        lcd.write(byte(1));
        lcd.setCursor(0,1);

        // Displays all the notes in the current tuning
        // 0 0 1 1 2 2 3 3 4 4 5 5 6 6 0 0
        for(int i = 0; i < 6; i++){
          lcd.setCursor(2*i + 2,1);
          lcd.print(tuningNotes[displayedTuning][i].charAt(0));
          if(tuningNotes[displayedTuning][i].charAt(1) == '#'){
            lcd.setCursor(2*i + 3,1);
            lcd.write(byte(2)); //Replace the '#' symbol with a nicer sharp symbol on the LCD
          }
        }
      }
      digitalWrite(enablePin, HIGH);
      break;
    case 1:
      if(newDisplay){
        newDisplay=false;
        lcd.clear();
        lcd.setCursor(7-int(sizeof(tuningNames[displayedTuning])/2.0),0);
        lcd.print(tuningNames[displayedTuning]);
        lcd.setCursor(0,1);
        lcd.print("CONFIRM TUNING?");
      }
      curString = 0;
      digitalWrite(enablePin,HIGH);
      // exit state
      break;
    case 2:
      if(curString > 5){
        machineState = 0;
      }
      if(newDisplay){
        newDisplay = false;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("ATTACH TO STRING");
        lcd.setCursor(3,1);
        lcd.print(curString+1);
        lcd.setCursor(7,1);
        lcd.print("CONTINUE");
        lcd.setCursor(15,1);
        lcd.write(byte(1));
      }
      // wait for user to attach motor to correct tuning peg
      // let user press button to proceed
      // exit state
      steps = 0;
      digitalWrite(enablePin, HIGH);
      break;
    case 3:
      stringFrequency = tuningFrequencies[selectedTuning][curString];
      filterFreq = 110 * stringFrequency;
      filterPWM->setPWM(filterPin, filterFreq, dutyCycle);

      if(newDisplay){
        newDisplay = false;
        lcd.clear();
        lcd.setCursor(1,0);
        lcd.print(tuningNotes[selectedTuning][curString]);
        if(tuningNotes[selectedTuning][curString].charAt(1) == '#'){
              lcd.setCursor(2,0);
              lcd.write(byte(2));
        }
        lcd.setCursor(8,0);
        lcd.print(stringFrequency);
        lcd.setCursor(14,0);
        lcd.print("Hz");
        lcd.setCursor(0,1);
        lcd.print("TUNING: ");
        lcd.setCursor(14,1);
        lcd.print("Hz");
      }
      
      lcd.setCursor(8,1);
      lcd.print(freqAvg);
      lcd.setCursor(14,1);
      lcd.print("Hz");
      
      // Calculate the frequency offset and turn motor accordingly
      
      // FInd rolling average of 20 frequency samples
      if(frequency != freqWindow[0]){
        freqAvg = 0;
        for(int i =0; i<19; i++){
          freqWindow[19-i]=freqWindow[18-i];
          freqAvg+=freqWindow[19-i];
        }
        freqWindow[0]=frequency;
        freqAvg += frequency;
        freqAvg = freqAvg/20.00;
      }
      //Serial.println(freqAvg);

      // Enable the motor and calculate the error, then begin turning
      digitalWrite(enablePin, LOW);
      freqErr = stringFrequency - freqAvg;
      if(frequency > 40){
        stepErr = alpha * freqErr;   // Alpha is static value defined initially, the proportional gain
        if(abs(freqErr)>0.5){
          digitalWrite(dirPin, !(stepErr > 0));   //HIGH is CCW, LOW is CW
          stepped = !stepped;
          digitalWrite(stepPin, stepped);
          steps++;
        }

        if(abs(freqErr) < 1.5){
          //stepperPWM->setPWM(stepPin, 0, dutyCycle);
          digitalWrite(enablePin, HIGH);
          digitalWrite(dirPin, LOW);
          digitalWrite(stepPin, HIGH);
          curString++;
          machineState = 2;
          newDisplay = true;
        }
      }
      // listen for frequency
      // determine error
      // move motor based on error
      // repeat until within tolerance
      // exit state
    break;
    
  }
  timer++;

  // Individual S tring Tuning Loop:
  //while (abs(freqErr) > 0.5){


      // Read current frequency
      // Apply digital filtering

      // Calculate frequnecy error
      //freqErr = stringFrequency - frequency;
      // Actuate motor based on error
      //motorDirection = constrain(100 * freqErr, 0, 1);
      //stepMotor(int(alpha * freqErr), motorDirection);

      // Break once string is within frequency tolerance
    //}
  // Update screen with next instructions

  //  Move to next string note or break if all strings have been tuned
}

// Measure Period ISR
void measurePeriod() {
  if(pulseCount == 0){
    pulseStartTime = micros();
    pulseCount++;
  }
  else if (pulseCount > 0 && pulseCount < maxPulses){
    pulseCount++;
  }
  else if(pulseCount == maxPulses){
    pulseEndTime = micros();
    newMeasurement = true;
    pulseCount = 0;
    frequency = 1.0/(0.000001*(pulseEndTime-pulseStartTime)/maxPulses);
  }
}

// Set Microstepping Size
void setMicroStep(int size) {
  digitalWrite(microstepPins[0], size & 1);
  digitalWrite(microstepPins[1], size & 2);
  digitalWrite(microstepPins[2], size & 4);
}

// Turning the stepper motor number of steps
void stepMotor(int steps, bool direction) {
  digitalWrite(dirPin, direction);
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20);
  }
}

// BUTTON ISRs
void leftPress(){
  newDisplay = true;
  if(timer-lastTimer > 200){
    newDisplay = true;
    if(machineState == 0){
      if(displayedTuning <= 0){
        displayedTuning = numTunings-1;
      }
      else{
        displayedTuning = displayedTuning - 1;
      }
    }
    lastTimer=timer;
  }
  newDisplay = true;
}
void enterPress(){
  newDisplay = true;
  if(timer-lastTimer > 200){
    newDisplay = true;
    if(machineState == 0){
      machineState = 1;
    }
    else if (machineState == 1){
      //curString = 0;
      machineState = 2;
    }
    else if(machineState == 2){
      machineState = 3;
    }
    lastTimer=timer;
  }
  newDisplay = true;
}
void rightPress(){
  newDisplay = true;
  if(timer-lastTimer > 200){
    newDisplay = true;
    if(machineState == 0){
      if(displayedTuning >= numTunings-1){
        displayedTuning = 0;
      }
      else{
        displayedTuning = displayedTuning + 1;
      }
    }
    lastTimer=timer;
  }
  newDisplay = true;
}
void backPress(){
  newDisplay = true;
  if(timer-lastTimer > 200){
    newDisplay = true;
    if(machineState != 0){
      machineState = machineState -1;
    }
    lastTimer=timer;
  }
  newDisplay = true;
}

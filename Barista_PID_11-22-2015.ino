/*
  Aroma PID v0.1
  Arduino code to run on PID controller for Rancilio Silvia.
  Runs on the Atmel ATMega328.

  by Cameron Charles, Seattle Circuit Design
  written: 11/23/2014
*/

// include libraries
#include <PID_v1.h>          
#include <LiquidCrystal.h>

// define the pins

  // analog inputs
const int TEMP_SENSE_PIN = A3;

  // digital inputs
const int FLOAT_SWITCH_PIN = A2;
const int BREW_SWITCH_PIN = A4;
const int STEAM_SWITCH_PIN = A5;
const int STEAM_ACTIVE_PIN = A0;  // check wiring, this may actually be A0

  // digital outputs
const int BOILER_RELAY_PIN = 4;
const int PUMP_RELAY_PIN = 2;
const int PANEL_LED_PIN = 6;
const int BUZZER_PIN = 3;

  // display
const int RS_PIN = 7;
const int RW_PIN = 8;
const int EN_PIN = 9;
const int D4_PIN = 10;
const int D5_PIN = 11;
const int D6_PIN = 12;
const int D7_PIN = 13;

// initialize the display
LiquidCrystal lcd(RS_PIN, EN_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

// messages for LCD user interface
const char MSG_LOCKED_TOP[] = " Enter  ";
const char MSG_LOCKED_BOTTOM[] = " Code:  ";
const char MSG_WELCOME_TOP[] = "  Ciao  ";
const char MSG_WELCOME_BOTTOM[] = " bella! ";
// const char MSG_TMP[] = "       C";
const char MSG_BREW_HEATING[] = "Heat: Br";
const char MSG_STEAM_HEATING[] = "Heat: St";
const char MSG_BREW_COOLING[] = "Cooling ";
const char MSG_BREW_READY[] = "Ready:Br";
const char MSG_STEAM_READY[] = "Ready:St";
const char MSG_BREW_SHOT[] = "Shot: ";
const char MSG_STEAM_ACTIVE[] = "Steaming";
const char MSG_FILL_WATER[] = "Low Wtr!";

// other UI variables
const unsigned long  READY_BEEP_DURATION = 250; // duration of ready beep in milliseconds
const unsigned long  START_BEEP_DURATION = 250; // duration of start beep in milliseconds
const unsigned long  WATER_BEEP_DURATION = 500; // duration of low water beep in milliseconds
const int TANK_RESERVE = 30; // time limit for running the pump after a low water condition is detected during a shot
const int BUTTON_ON = 0;

// temperature set points in Celsius
typedef struct {double minimum; double target; double maximum;} TemperatureSetpoint;
const TemperatureSetpoint BREW_TEMP = {93, 95, 97};
const TemperatureSetpoint STEAM_TEMP = {125, 130, 135};
TemperatureSetpoint setpoint;

// define state variables
enum STATE {WELCOME, BREW_HEATING, BREW_READY, BREW_SHOT, BREW_COOLING, STEAM_HEATING, STEAM_READY, STEAM_ACTIVE, FILL_WATER, SHOT_LOW_WATER};
enum STATE state;             // variable for state machine
int brewButtonState;           // BUTTON_ON for pulling a shot
int steamButtonState;         // BUTTON_ON for steam mode
int steamSwitchState;         // BUTTON_ON when steam valve is open
int floatSwitchState;         // '1' for low water, '0' for adequate water
int boilerInterlock = 0;        // '0' for business as usual, '1' to shut the boiler off due to low water levels
unsigned long startShot;       // absolute start time of the current shot (in milliseconds)
int brewTimer;        // holds the duration of the shot being pulled (in seconds)
unsigned long lowWaterTimeLimit;  // time limit for shutting off the pump when low water is detected during a shot
unsigned long endBeep = 0;    // time in milliseconds to stop the current beep

// PID variables
double targetTemp;          // temperature that we are regulating to
double actualTemp;          // actual boiler temperature
double relayDrive;               // PWM drive level for the boiler

double propTerm;  // added to observe internal PID workings
double intTerm;
double diffTerm;

const double KP = 45;            // proportional gain
const double KI = 0.3;              // integral gain
const double KD = 500;           // derivative gain

PID espressoPid(&actualTemp, &relayDrive, &targetTemp, KP, KI, KD, DIRECT, &propTerm, &intTerm, &diffTerm);
const int WINDOW_SIZE = 1000;     // length (in ms) of PWM interval for SSR
unsigned long windowStartTime;

/*************************** Initialization ***************************/

void setup() {
  
  // Configure the pins (don't need to configure analog inputs, don't need to configure display pins)
  pinMode(BREW_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEAM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEAM_ACTIVE_PIN, INPUT_PULLUP);
  pinMode(FLOAT_SWITCH_PIN, INPUT_PULLUP);
  
  pinMode(BOILER_RELAY_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(PANEL_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RW_PIN, OUTPUT);

  Serial.begin(9600);    // initialize UART for sending temperature logging data

  analogReference(EXTERNAL);  // set the ADC reference to be the 2.5V LDO
  
  // Initialize relay output pins
  digitalWrite(BOILER_RELAY_PIN, HIGH); // Driving high because Q1 is damaged on this PCBA
  digitalWrite(PUMP_RELAY_PIN, LOW);

  windowStartTime = millis();              
  targetTemp = setpoint.target;
  espressoPid.SetTunings(KP, KI, KD);
  espressoPid.SetOutputLimits(0, WINDOW_SIZE);    // tell the PID to range between 0 and the full window size
  espressoPid.SetSampleTime(WINDOW_SIZE);         // tell the PID to update once every Window
  espressoPid.SetMode(AUTOMATIC);                // turn the PID on

  state = WELCOME;
 
  digitalWrite(RW_PIN, LOW);
  lcd.begin(8, 2);
}

/***************************** Main Loop *****************************/

void loop() {
  
  targetTemp = setpoint.target;   // update the setpoint in case it changed in the state machine last loop
  unsigned long now = millis();   // used for SSR PWM
    
  // If the window has expired, evaluate PID variables and print to display and serial port
  if (now - windowStartTime > WINDOW_SIZE && boilerInterlock == 0){ // if the window has expired and we are not pumping cold water into the boiler (in which case we don't want to be accumulating the integral term)
    windowStartTime += WINDOW_SIZE;         // update the current window start time
    
    actualTemp = readTemp();    // read the temperature
    
    if (state != BREW_SHOT) {
      espressoPid.Compute();          // calculate the new PID output if there is no cold water entering the tank
    }
    else {
      relayDrive = 1000;              // if there is cold water entering the tank (brew or hot water) then skip the calculations (don't want to accumulate integral term) and use feedforward to put the boiler on full.
    }
    
    String stringTemp = tempToString(actualTemp);
    displayTemp();        // write the temperature to the display

    // format the temperature and PID variables for output to the serial port
    stringTemp = stringTemp + "," + String(long(propTerm)) + "," + String(long(intTerm)) + "," + String(long(diffTerm)) + "," + String(long(relayDrive));
    Serial.println(stringTemp);            
  }
  
  // SSR PWM
  if (((relayDrive > now - windowStartTime) || (state == BREW_SHOT) || (state == SHOT_LOW_WATER) || (state == STEAM_ACTIVE)) && (boilerInterlock == 0)) {
    digitalWrite(BOILER_RELAY_PIN, LOW); // Driving low because Q1 is damaged on this PCBA
  }
  else {
    digitalWrite(BOILER_RELAY_PIN, HIGH); // Driving high because Q1 is damaged on this PCBA
  } 

  // read the switches ('0' for off, '1' for on)
  brewButtonState = digitalRead(BREW_SWITCH_PIN);
  steamButtonState = digitalRead(STEAM_SWITCH_PIN);
  steamSwitchState = digitalRead(STEAM_ACTIVE_PIN);
  floatSwitchState = digitalRead(FLOAT_SWITCH_PIN);
    
  // state machine
  switch (state){
    
    case WELCOME:
      solidLED(PANEL_LED_PIN);
      displayTopMsg(MSG_WELCOME_TOP);
      displayBotMsg(MSG_WELCOME_BOTTOM);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 0;

      // Two beeps to signal the beginning of session
      setupBeep();
      delay(50);
      setupBeep();
      delay(1000);  // delay to show the welcome message long enough
        
      // read the temperature to determine which state we should start in
      actualTemp = readTemp();
    
      // write temp to display
      displayTemp();

      // set the initial state based on the float and steam switch positions and boiler temperature
      if (floatSwitchState == 0){  // if the water level is low
        state = FILL_WATER;
        endBeep = millis() + WATER_BEEP_DURATION;
      }
      else if (steamButtonState != BUTTON_ON){  // if the water level is high and the switch is in the brew position
        setpoint = BREW_TEMP;
        if (actualTemp < setpoint.minimum){
          state = BREW_HEATING;
        }
        else if (actualTemp > setpoint.maximum){
          state = BREW_COOLING;
        }
        else {
          state = BREW_READY;
        }
      }
      else {                      // otherwise switch must be in the steam position
        setpoint = STEAM_TEMP;
        if (actualTemp < setpoint.minimum){
          state = STEAM_HEATING;  
        }
        else {
          state = STEAM_READY;
        }
      }  
      break;

    case FILL_WATER:
      loopBeep();
      offLED(PANEL_LED_PIN);
      displayTopMsg(MSG_FILL_WATER);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 1;

      if (floatSwitchState == 1){  // if the water tank has been filled then enter the correct state
        boilerInterlock = 0;      // remove the boiler interlock to allow heating to resume
        if (steamButtonState != BUTTON_ON){  // if the switch is in the brew position
          setpoint = BREW_TEMP;
          if (actualTemp < setpoint.minimum){
            state = BREW_HEATING;
          }
          else if (actualTemp > setpoint.maximum){
            state = BREW_COOLING;
          }
          else {
            state = BREW_READY;
          }
        }
        else {                      // otherwise switch must be in the steam position
          setpoint = STEAM_TEMP;
          if (actualTemp < setpoint.minimum){
            state = STEAM_HEATING; 
          }
          else {
            state = STEAM_READY;
          }
        } 
      }
      break;
    
    case BREW_HEATING:
      digitalWrite(BUZZER_PIN, LOW);
      pulseLED(PANEL_LED_PIN);
      displayTopMsg(MSG_BREW_HEATING);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 0;
      setpoint = BREW_TEMP;

      if (steamButtonState == BUTTON_ON){  // if the steam button has been pressed before brew temperature has been reached
        state = STEAM_HEATING;
      }
      else if (actualTemp > setpoint.minimum){  // if the brew temperature has been reached
        state = BREW_READY;
        endBeep = millis() + READY_BEEP_DURATION;
      }
      else if (brewButtonState == BUTTON_ON){ // if the brew button has been pressed
        state = BREW_SHOT;
        brewTimer = -1;
        startShot = millis();
        // updateTimer();
        // digitalWrite(PUMP_RELAY_PIN, HIGH);
      }
      break;
      
    case BREW_READY:
      loopBeep();
      solidLED(PANEL_LED_PIN);
      displayTopMsg(MSG_BREW_READY);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 0;
      setpoint = BREW_TEMP;

      if (actualTemp < setpoint.minimum){ // if we have dropped out of range due to a transient condition (undershoot when cooling from steam)
        state = BREW_HEATING;
      }
      else if (actualTemp > setpoint.maximum){  // if we have risen out of range due to a transient condition (bounce when cooling from steam with hot water flush)
        state = BREW_COOLING;
      }
      
      if (steamButtonState == BUTTON_ON){    // if the steam button has been pressed
        state = STEAM_HEATING;
      }
      else if (brewButtonState == BUTTON_ON){ // if the brew button has been pressed
        state = BREW_SHOT;
        brewTimer = -1;
        startShot = millis();
      }
      break;

    case BREW_SHOT:
      digitalWrite(BUZZER_PIN, LOW);
      solidLED(PANEL_LED_PIN);
      displayTopMsg(MSG_BREW_SHOT);
      digitalWrite(PUMP_RELAY_PIN, HIGH);
      boilerInterlock = 0;
      setpoint = BREW_TEMP;

      updateTimer();
      // displayTimer();
      
      if (floatSwitchState == 0){  // if the water level has dropped below the threshold as a result of the shot
        state = SHOT_LOW_WATER;
        lowWaterTimeLimit = brewTimer + TANK_RESERVE; // allow the pump to run for a set time before shutting it down
      }
      else if (brewButtonState != BUTTON_ON){  // if the brew button has been released
        digitalWrite(PUMP_RELAY_PIN, LOW);
        delay(1000); // delay to give user a chance to see how long the shot took
        if (actualTemp < setpoint.minimum){ // if the temperature has dropped below the lower threshold
          state = BREW_HEATING;
        }
        else if (actualTemp > setpoint.maximum){
          state = BREW_COOLING;
        }
        else {
          state = BREW_READY;
        }
      }
      break;

    case SHOT_LOW_WATER:
      digitalWrite(BUZZER_PIN, LOW);
      solidLED(PANEL_LED_PIN);
      displayTopMsg(MSG_BREW_SHOT);
      digitalWrite(PUMP_RELAY_PIN, HIGH);
      boilerInterlock = 0;
      setpoint = BREW_TEMP;
      
      updateTimer();
      // displayTimer();
      
      if (brewTimer > lowWaterTimeLimit || brewButtonState != BUTTON_ON){ // if we run past the limit set for the pump or the shot finishes normally
        state = FILL_WATER;
        endBeep = millis() + WATER_BEEP_DURATION;
      }
      break;
        
    case STEAM_HEATING:
      digitalWrite(BUZZER_PIN, LOW);
      pulseLED(PANEL_LED_PIN);
      displayTopMsg(MSG_STEAM_HEATING);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 0;
      setpoint = STEAM_TEMP;
      
      if (steamButtonState != BUTTON_ON){    // if the steam button has been released before steam temperature has been reached
        setpoint = BREW_TEMP;
        if (actualTemp > setpoint.maximum){  // if the temperature has risen above the acceptable brewing temperature
          state = BREW_COOLING;
        }
        else if (actualTemp < setpoint.minimum){  // if the temperature is below the acceptable brewing temperature
          state = BREW_HEATING;
        }
        else {    // otherwise we must be ready to brew
          state = BREW_READY;
          endBeep = millis() + READY_BEEP_DURATION;
        }
      }
      else if (actualTemp > setpoint.minimum){  // if we have reached the steam temperature
        state = STEAM_READY;
        endBeep = millis() + READY_BEEP_DURATION;
      }
      else if (steamSwitchState == BUTTON_ON){   // operator decided to start steaming before steam temperature reached
        state = STEAM_ACTIVE;       
      }
      break;
      
    case STEAM_READY:
      loopBeep();
      solidLED(PANEL_LED_PIN);
      displayTopMsg(MSG_STEAM_READY);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 0;
      setpoint = STEAM_TEMP;

      if (steamButtonState != BUTTON_ON){    // if the steam button has been released
        state = BREW_COOLING;
      }
      else if (steamSwitchState == BUTTON_ON){   // user started steaming
        state = STEAM_ACTIVE;
      }
      break;

    case STEAM_ACTIVE:
      digitalWrite(BUZZER_PIN, LOW);
      solidLED(PANEL_LED_PIN);
      displayTopMsg(MSG_STEAM_ACTIVE);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 0;
      setpoint = STEAM_TEMP;

      if (steamSwitchState != BUTTON_ON){   // user finished steaming
        if (actualTemp < setpoint.minimum){  // if we dropped below the steam temperature
          state = STEAM_HEATING;
        }
        else if (actualTemp >= setpoint.minimum){  // if we stayed above steam temperature
          state = STEAM_READY;
        }
      }
      break;
    
    case BREW_COOLING:
      digitalWrite(BUZZER_PIN, LOW);
      offLED(PANEL_LED_PIN);
      displayTopMsg(MSG_BREW_COOLING);
      digitalWrite(PUMP_RELAY_PIN, LOW);
      boilerInterlock = 0;
      setpoint = BREW_TEMP;
  
      if (steamButtonState == BUTTON_ON){    // if the steam button has been pressed again before brew temperature has been reached
        setpoint = STEAM_TEMP;
        if (actualTemp > setpoint.minimum){ // if we never dropped below the acceptable steaming range
          state = STEAM_READY;
          endBeep = millis() + READY_BEEP_DURATION;
        }
        else {        // otherwise we must be heating to steam
          state = STEAM_HEATING;
        }
      }
      else if (brewButtonState == BUTTON_ON){ // if the brew button has been pressed
        state = BREW_SHOT;
        brewTimer = -1;
        startShot = millis();
      }
      else if (actualTemp < setpoint.maximum){  // if we have cooled to the acceptable brewing temperature
        state = BREW_READY;
        endBeep = millis() + READY_BEEP_DURATION;
      }
      break;
      
    default:
      state = STEAM_HEATING;  // in an error case, assign state to "heating to steam" since all other cases can be reached from here
  }
}

/***************************** Functions *****************************/

// Sound a tone on the buzzer while in setup
void setupBeep(){
  digitalWrite(BUZZER_PIN, HIGH);
  delay(START_BEEP_DURATION);
  digitalWrite(BUZZER_PIN, LOW);
}

// Sound a tone on the buzzer while in the main loop 
void loopBeep(){
  unsigned long time_now = millis();
  if (time_now < ::endBeep){
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// Update the shot timer
void updateTimer(){
  unsigned long shotTime = millis() - startShot;
  if ((shotTime - (brewTimer * 1000)) > 1000){
    brewTimer = brewTimer + 1;
    String shotTimeString = String(brewTimer % 100);
    if (brewTimer < 10){
      shotTimeString = " " + shotTimeString;
    }
    lcd.setCursor(6, 0);
    lcd.print(shotTimeString);
  }
}

// Read the value of a panel switch
int readSwitch(int switch_name) {
  int count;
  int switch_state;
  count = analogRead(switch_name);  // read the ADC value
  if(count > 820){
    switch_state = 0; // assign '0' for the switch being off (analog value > 4V)
  }
  else {
    switch_state = 1; // assign '1' for the switch being on (analog value < 4V because of LED current flow)
  }
  return switch_state;
}

// Read the temperature and return it in ADC counts
double readTemp(){
  int count;
  double celsius;
  
  count = analogRead(TEMP_SENSE_PIN); // read a second time for the legitimate input value
  
  if (count > 573) // Low temp range
  {
    celsius = 10.0 + ((float)(1023 - count)) * 0.1775;
  } 
  else if (count < 514) // High temp range
  {
    celsius = 100.2 + ((float)(513 - count)) * 0.1709;
  } 
  else // Mid temp range (brewing)
  {
    celsius = 89.9 + ((float)(573 - count)) * 0.1729;
  }
  return celsius;
}

// Convert the temperature from a double to a String, rounding to one decimal place
String tempToString(double doubleTemp){
  int intTemp = int(doubleTemp + 0.05);
  int fracTemp = int((doubleTemp + 0.05)*10.0) - intTemp*10;
  String stringTemp = String(intTemp);
  stringTemp = stringTemp + "." + fracTemp + " ";
  if (intTemp < 100) {
    stringTemp = " " + stringTemp;
  }
  return stringTemp;
}

// display a message to the top row of the LCD
void displayTopMsg(const char message[]){
  lcd.setCursor(0, 0);
  lcd.print(message);
}

// display a message to the bottom row of the LCD
void displayBotMsg(const char message[]){
  lcd.setCursor(0, 1);
  lcd.print(message);
}

// display the temperature on the LCD
void displayTemp(){
  int intTemp = int(actualTemp + 0.05);
  int fracTemp = int((actualTemp + 0.05)*10.0) - intTemp*10;
  String stringTemp = String(intTemp);
  stringTemp = stringTemp + "." + fracTemp + " C";
  if (intTemp < 100) {
    stringTemp = " " + stringTemp;
  }
  
  lcd.setCursor(1, 1);
  lcd.print(stringTemp);
}

// display the brew timer on the LCD
void displayTimer(){
  if (brewTimer < 10){
    lcd.setCursor(7, 0);
  }
  else {
    lcd.setCursor(6, 0); 
  }
  lcd.print(brewTimer % 100);
}

// use PWM to pulse the LED
void pulseLED(int pin){
  unsigned long pulseTime = millis()%1000;
  if (pulseTime > 510){
    analogWrite(pin, 255);
  }
  else if (pulseTime > 255){
    analogWrite(pin, pulseTime - 255);
  }
  else {
    analogWrite(pin, 255 - pulseTime);
  }
}

// drive the LED solidly on
void solidLED(int pin){
  analogWrite(pin, 0);
}

// turn the LED off
void offLED(int pin){
  analogWrite(pin, 255);
}


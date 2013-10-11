



#include <SPI.h>
#include <EEPROM.h>
#include <eepromRW.h>

//START OF FOCUS CONTROL INITIALISE
#define MAX_COMMAND_LEN             (5)
#define MAX_PARAMETER_LEN           (6)
#define COMMAND_TABLE_SIZE          (11)
#define TO_UPPER(x) (((x >= 'a') && (x <= 'z')) ? ((x) - ('a' - 'A')) : (x))

// initialize the library with the numbers of the interface pins
int dirPin = 2; // Easy Driver Direction Output Pin
int stepperPin = 3; // EasyDriver Stepper Step Output Pin
int sleepPin = 9;
int ms1Pin = 8;
int ms2Pin = 7;
int enablePin = 6;
int resetPin = 5;
int pfdPin = 4;
unsigned long powerMillis = 0; // used to remember when EasyDriver power was enabled
int motorSteps = 200; //number if steps for the motor to turn 1 revolution


volatile long NoOfSteps = 1000; //required number of steps to make
volatile long Position = 0; //used to keep track of the current motorposition
volatile long MaxStep = 19250; //define maximum no. of steps, max travel
volatile int Speed = 500;
volatile int BoardType = 0; // Boardtypes, default is 0, EasyDriver, 1=L293 chip, 2=LadyAda AFmotor board                           

boolean Direction = true;//True is one way false is other.Change to false if motor is moving in the wrong direction
boolean IsMoving = false;
boolean Absolute = true;
volatile long MaxIncrement=16384;//not yet used
//END OF FOCUS CONTROL INITIALISE

//Serial comms setup
char incomingByte = 0; // serial in data byte
byte serialValue = 0;
boolean usingSerial = true; // set to false to have the buttons control everything
char gCommandBuffer[MAX_COMMAND_LEN + 1];
char gParamBuffer[MAX_PARAMETER_LEN + 1];
long gParamValue;
volatile boolean UPDATE = true;

struct config_t //Memory Structure for Parking, Unparking the Focuser and other config settings 
{
    long parkposition;
    boolean parked;
    boolean stepperdirection;
    long controlboardtype;
} configuration;


typedef struct {
  char const    *name;
  void          (*function)(void);
} 
command_t;


//Serial Comms setup end

//***************************************************
//*****Start of User defined Functions **************
//***************************************************

//START OF FOCUS CONTROL FUNCTIONS
void EasyDriverStep(boolean dir,long steps){
  digitalWrite(sleepPin, HIGH); // enable power to the EasyDriver
  powerMillis = millis(); // remember when power was switched on
  delay(1); // wait a bit after switching on power
  digitalWrite(dirPin,dir);
  delay(1);
  for(int i=0;i<steps;i++){
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(Speed);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(Speed);
  }
}

void ParkFocuserFun (void) {//Park the focuser by setting the Park bit to 1 and the current Focuser Position in Configuration

 if (configuration.parked == false){
 configuration.parkposition = Position;
 configuration.stepperdirection = Direction;
 configuration.parked = true;
 configuration.controlboardtype = BoardType;
 
 EEPROM_writeAnything(0, configuration);
 }
  UPDATE=true; //Update even if the focuser was already parked
}

void FocusINFun (void) {//Move the Stepper IN.
  long Steps = 0;
  if (Absolute == false) {  //If not Absolute move the number of steps
    if ((Position-NoOfSteps)>=0) {
        EasyDriverStep(Direction,NoOfSteps);
        Position=Position-NoOfSteps;
      }
  }
  else if (NoOfSteps < MaxStep) //Absolute :- work out the number of steps to take based on current position
  {
    if (NoOfSteps<Position){
    
      Steps=(Position-NoOfSteps);
      EasyDriverStep(Direction,Steps);
      Position=NoOfSteps;
    }
    else
    {
      Steps=(NoOfSteps-Position);
      EasyDriverStep(!Direction,Steps);
      Position=NoOfSteps;  
    }
  }
  // set the update flag so that the new position is displayed
  IsMoving=true;
  UPDATE=true;
}

void FocusOUTFun (void) {//Move the Stepper OUT.
  long Steps = 0;

  if (Absolute == false) {  //If not Absolute move the number of steps
    if ((Position+NoOfSteps)<=MaxStep) {
        EasyDriverStep(!Direction,NoOfSteps);
        Position=Position+NoOfSteps;
      }
  }
  else if (NoOfSteps < MaxStep) //Absolute :- work out the number of steps to take based on current position
  {
    if (NoOfSteps>Position){
    
      Steps=(NoOfSteps-Position);
      EasyDriverStep(!Direction,Steps);
      Position=NoOfSteps;
    }
    else
    {
      Steps=(Position-NoOfSteps);
      EasyDriverStep(Direction,Steps);
      Position=NoOfSteps;  
    }
  }
  // set the update flag so that the new position is displayed
  IsMoving=true;
  UPDATE=true;
}

void FocusSTEPSFun (void) {//Set the number of Steps.
  NoOfSteps = gParamValue;
  // set the update flag so that the new position is displayed
  UPDATE=true;
}

// function to set the RPM of the stepper motor
// user sends :Speed:500:
void FocusSpeedFun (void) {
  Speed = gParamValue;
  UPDATE=true;
}

// Set max limit for focus travel, for absolute positioning focusers
void FocusSLimitFun (void) {
  MaxStep = gParamValue;
  UPDATE=true;
}

// set current focuser position, used for calibrating absolute positioning focusers
void FocusSPositionFun (void) {
  Position = gParamValue;
  UPDATE=true;
}

// set the focuser mode to relative 0 or absolute positioning 1
void FocusSModeFun (void) {
  switch (gParamValue){
  case 0:
    Absolute=false;
    //Serial.println("Relative Mode"); // debug only
    break;
  case 1:
    Absolute=true;
    //Serial.println("Absolute Mode"); // debug only
    break;
  default:
    //Serial.println("0 or 1 for relative or absolute, try again"); // debug only
    break;  
  }
  UPDATE=true;
}
// to add different motor types, stepper, servo or DC
void FocusSTypeFun(void){
  UPDATE=true;
}

// to add different motor types, stepper, servo or DC
void FocusBoardTypeFun(void){
  BoardType=gParamValue;
  UPDATE=true;
}
//END OF FOCUS CONTROL FUNCTIONS

//Start of serial control functions

void SerialDATAFun (void) {//Update All information over comms if there has been any change in the state of the focuser
  Serial.print("#POS:");  
  Serial.print(Position);
  Serial.println(";");
  Serial.print("#STP:" );
  Serial.print(NoOfSteps);
  Serial.println(";");
  Serial.print("#MDE:");
  if (Absolute){
    Serial.print("1");
  }
  else{  
    Serial.print("0");
  }
  Serial.println(";");
  Serial.print("#LMT:");
  Serial.print(MaxStep);
  Serial.println(";");
  Serial.print("#SPD:");
  if (Speed==0){
    Serial.print(char(Speed));
  }
  else{  
    Serial.print(Speed);
  }
  Serial.println(";");
  if (IsMoving==true) {
    Serial.print("#MOV:");
    Serial.print("1");
    Serial.println(";");
    IsMoving=false;
  }
  Serial.print("#BRD:0");  
  Serial.print(BoardType);
  Serial.println(";");
  Serial.print("#PRK:");  
  if (configuration.parked == 1) Serial.print("01"); else Serial.print("00");
  Serial.println(";");
}


//Set up a command table. when the command "IN" is sent from the PC and this table points it to the subroutine to run
command_t const gCommandTable[COMMAND_TABLE_SIZE] = {
  {
    "IN1",     FocusINFun,  }
  ,
  {
    "OUT",    FocusOUTFun,   }
  ,
  {
    "STP",  FocusSTEPSFun,   }
  ,
  {
    "SPD",  FocusSpeedFun,   }
  ,
  {
    "LMT",  FocusSLimitFun,   }
  ,
  {
    "POS",    FocusSPositionFun,   }
  ,  
  {
    "MDE",   FocusSModeFun,   }
  ,
  {
    "TYP",   FocusSTypeFun,   }
  ,  
  {
    "PRK",   ParkFocuserFun,   }
  ,  
  {
    "BRD",   FocusBoardTypeFun,   }
  ,  
  {
    NULL,      NULL   }
};


//Process Command. This searches the command table to see if the command exits if it does then the required subroutine is run
void cliProcessCommand(void)
{
  int bCommandFound = false;
  int idx;

  /* Convert the parameter to an integer value. 
   * If the parameter is emplty, gParamValue becomes 0. */
  gParamValue = strtol(gParamBuffer, NULL, 0);

  /* Search for the command in the command table until it is found or
   * the end of the table is reached. If the command is found, break
   * out of the loop. */
  for (idx = 0; gCommandTable[idx].name != NULL; idx++) {
    if (strcmp(gCommandTable[idx].name, gCommandBuffer) == 0) {
      bCommandFound = true;
      break;
    }
  }

  /* If the command was found, call the command function. Otherwise,
   * output an error message. */
  if (bCommandFound == true) {
    (*gCommandTable[idx].function)();
  }
}


// When data is in the Serial buffer this subroutine is run and the information put into a command buffer.
// The character : is used to define the end of a Command string and the start of the parameter string
// The character ; is used to define the end of the Parameter string
int cliBuildCommand(char nextChar) {
  static uint8_t idx = 0; //index for command buffer
  static uint8_t idx2 = 0; //index for parameter buffer
  int loopchk = 0;

  nextChar = Serial.read();
  do
  {

    gCommandBuffer[idx] = TO_UPPER(nextChar);
    idx++;
    nextChar = Serial.read();
    loopchk=loopchk+1;
  } 
  while ((nextChar != ':') && (loopchk < 100));

  loopchk=0;

  nextChar = Serial.read();

  do
  {

    gParamBuffer[idx2] = nextChar;
    idx2++;
    nextChar = Serial.read();
  } 
  while ((nextChar != ';')&& (idx2 < 100));



  gCommandBuffer[idx] = '\0';
  gParamBuffer[idx2] = '\0';
  idx = 0;
  idx2 = 0;

  return true;
}

//END of serial control functions



void setup() {

  EEPROM_readAnything(0, configuration); //PARKING:- Read the Position and Parked info
  if (configuration.parked == true) { //If the Focuser was Parked then load the Position information
    Position = configuration.parkposition; //Load the Position information
    Direction = configuration.stepperdirection;
    BoardType = configuration.controlboardtype;
  }

  pinMode(dirPin, OUTPUT); //Initialise Easydriver output
  pinMode(stepperPin, OUTPUT); //Initialise Easydriver output
  pinMode(ms1Pin, OUTPUT); //Initialise Easydriver output
  pinMode(ms2Pin, OUTPUT); //Initialise Easydriver output
  pinMode(resetPin, OUTPUT); //Initialise Easydriver output
  pinMode(enablePin, OUTPUT); //Initialise Easydriver output
  pinMode(sleepPin, OUTPUT); //Initialise Easydriver output
  pinMode(pfdPin, OUTPUT); //Initialise Easydriver output
  
  Serial.begin(19200);// start the serial
  
  NoOfSteps=motorSteps;
  
  pinMode(13,OUTPUT);
  digitalWrite(sleepPin, LOW); //Easydriver Pwer off (Low = powered down)
// MS1/MS2 Truth table
// ms1  ms2  Resol
//  L    L    Full steps
//  H    L    Half steps
//  L    H    Quarter steps
//  H    H    Eight steps
  digitalWrite(ms1Pin, LOW); // see above
  digitalWrite(ms2Pin, LOW); // see above
  digitalWrite(resetPin, HIGH); // Not reset
  digitalWrite(enablePin, LOW); // System enabled
  digitalWrite(pfdPin, HIGH); // Slow decay, if LOW Fast Decay
}

void loop() {
 
  int bCommandReady = false;
  
  //FocuserControl Power off command
  if (millis() > (powerMillis + 20000)) // check if power has been on for more than 20 seconds
  {
      digitalWrite(sleepPin, LOW); // if yes, then disable power
  }
  
  //If There is information in the Serial buffer read it in and start the Build command subroutine
  if (usingSerial && Serial.available() >= 1) {
    // read the incoming byte:
    incomingByte = Serial.read();
    delay(5);
    if (incomingByte == '#') {
      /* Build a new command. */
      bCommandReady = cliBuildCommand(incomingByte);
    }
  }
  else
  {
    incomingByte=0;
    //Serial.flush();
  }

  //If there is a command in the buffer then run the process command subroutine
  if (bCommandReady == true) {
    bCommandReady = false; // reset the command ready flag
    cliProcessCommand(); // run the command
  }
  if ((Position != configuration.parkposition)) {
    configuration.parked = 0;
    EEPROM_writeAnything(0, configuration);
  }
  if (UPDATE){
    UPDATE=false;
    SerialDATAFun();  // Used to send the current state of the focuser to the PC over serial comms
  }
}
//-------------------------------------------------------------------------------------
//- Project           : ArduinoFocus
//- Revision          : 11.03.29
//- Author(s)         : Stormywebber , Frostbyte
//- Requirements      : Arduino UNO Board, AdaFruit AFmotor Board, Stepperengine
//-                   : Arduino IDE, AFmotor library
//-                   : Robofocus client on PC either through ASCOM or direct.
//- Description       : Project target is a computer controlled telescope focussing
//-                     system using the RoboFocus serial command protocol.
//-
//- Todo              : Check and choose good variable and function names
//-                     Store configuration in EEprom
//-                     use better serialLib´s with: transmit buffer. tweak buffer sizes
//-------------------------------------------------------------------------------------

#include <AFMotor.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <MsTimer2.h>

AF_Stepper motor(200, 2);              // 200 steps/rotation on port 2

//-------------------------------------------------------------------------------------
//- Global Constants
//-                                                                                   -
//-------------------------------------------------------------------------------------
#define MINIM_MOTORSPEED      10
#define MAXIM_MOTORSPEED      70
#define SERCOMBAUDRATE        9600
#define TEMPSENSPIN           0
#define MAXSENSORS            3
#define AVGDEPTH              3

#define ROTS                   A3
#define ROTA                   A4
#define ROTB                   A5

//-------------------------------------------------------------------------------------
//- Global Variables definitions and declarations                                     -
//-                                                                                   -
//-------------------------------------------------------------------------------------
unsigned char MySerBuf[9];
unsigned char txframe[9];
unsigned int  MyTmr1, MyTmr2, MyTmr3;
int           Temperature = 600;

struct Ramping {
    boolean      enabled;              //  
    unsigned int Maxspeed;             //  
    unsigned int Minspeed;             //
    unsigned int Curspeed;             //
    unsigned int Loc2;                 //  
    unsigned int Loc3;                 //
    unsigned int SpeedInc;             // speed in- decrease per step
    unsigned int ramp;                 // length of ramp in steps
} rp ;

struct CalibParam {                    // Calibration Parameters aX^2 + bX + c
  float a;
  float b;
  float c;  
} ;

struct AnalogSensors {
   int  SurfTemp;             // Surface Temp                    [-100.0 to 100.0 °C]
   int  AmbTemp;              // Ambient Air Temp                [-100.0 to 100.0 °C]
   int  AmbHum;               // Ambient Air rel.Humidity        [   0.0 to 100.0 % ]
   int  BarPres;              // Ambient Air barometric pressure [ 500   to 1500  mbar]
   int  DewTemp;              // Dewpoint temperature in Celcius  (calculated from above)
   long avgarray[MAXSENSORS]; // values are stored as <<AVGDEPTH  example *8
   CalibParam CalPar[MAXSENSORS] ;
} an ;

struct PowerRelayModule {
    boolean      ch1;                  //  Left channel
    boolean      ch2;                  //  True == powered, false == off
    boolean      ch3;                  //  
    boolean      ch4;                  //  right channel
} pm ;

struct StepperCoordinate {
    boolean      trigger;              //  Trigger to start stepper statemachine
    boolean      powered;              //  Signals if stepper coils are powered
    boolean      active;               //  signals is stepper is moving
    boolean      StopNow;              //  Req immidiate stop of stepper
    boolean      SendFrame;            //  signal to send frame when motor finisched moving
    byte         DutyCycle;
    byte         StepDelay;
    byte         StepSize;
    byte         Backlash;             //  nr of steps to take out slack
    char         LashDir;              //  final moving direction the stepper
    unsigned int LimitLoc;             //  
    unsigned int CurrentLoc;
    unsigned int TargetLoc;
} sc ;

//-------------------------------------------------------------------------------------
//- Setup, configure all hardware and software
//-                                                                                   -
//-------------------------------------------------------------------------------------
void setup() {
  pinMode(ROTA, INPUT);
  pinMode(ROTB, INPUT);
  pinMode(ROTS, INPUT);
  digitalWrite(ROTA, HIGH);                        // enable pullups
  digitalWrite(ROTB, HIGH);
  digitalWrite(ROTS, HIGH);

  Serial.begin(SERCOMBAUDRATE);                    // set up Serial library at 9600 bps
  Serial.println("ArduinoFocus");                  //§DEBUG

  MsTimer2::set(10, TimerInterupt);                // 10ms period
  MsTimer2::start();

  sc.trigger    = false;
  sc.powered    = false;
  sc.active     = false;
  sc.StopNow    = false;
  sc.SendFrame  = false;
  sc.DutyCycle  = 1000 ;                            // == 100.0% =d250 in robofocus
  sc.StepDelay  = 10;
  sc.StepSize   = 1;
  sc.Backlash   = 4;
  sc.LashDir    = 'I';
  sc.LimitLoc   = 65000;
  sc.CurrentLoc = 32000;
  sc.TargetLoc  = 32000;

  rp.enabled    = false;                                     //  
  rp.Maxspeed   = MAXIM_MOTORSPEED;                          //  
  rp.Minspeed   = MINIM_MOTORSPEED;                          //
  rp.Curspeed   = rp.Minspeed;                               //
  rp.SpeedInc   = 1 ;                                        //
  rp.ramp = (rp.Maxspeed - rp.Minspeed) / rp.SpeedInc ;      //
  
  pm.ch1 = pm.ch2 = pm.ch3 = pm.ch4 = false;                 // PowerRelayModule

  motor.setSpeed(MAXIM_MOTORSPEED);                          // rpm   
  motor.step(20,  FORWARD, SINGLE);    delay(50);            //§DEBUG
  motor.step(20, BACKWARD, SINGLE);                          //§DEBUG
  motor.setSpeed(MINIM_MOTORSPEED);                          // rpm   
  motor.release();
}//setup

void LoadParam_E2P()
{ 
    EEPROM.read(0);
    //EEPROM.write(0,0); //~3.3ms
}

//-------------------------------------------------------------------------------------
//- TimerInterupt, called every 10msec
//-                                                                                   -
//-------------------------------------------------------------------------------------
void TimerInterupt()
{ if (MyTmr1) MyTmr1--;
  if (MyTmr2) MyTmr2--;
  if (MyTmr3) MyTmr3--;
}//TimerInterupt

//-------------------------------------------------------------------------------------
//- Main Loop, calls the different statemachines for communication and steppercontrol -
//-                                                                                   -
//-------------------------------------------------------------------------------------
void loop() {

//  EEprom_storage();
    SerialCommunication_StateMachine();
    StepperMotor_StateMachine();
    Sensor_StateMachine();                                   // Read all sensor like Temp,Hum,AirPressure
    Keypad_StateMachine();
  
}//loop

//-------------------------------------------------------------------------------------
//- StepperMotor_StateMachine, 
//-                                                                                   -
//-------------------------------------------------------------------------------------
void StepperMotor_StateMachine()
{  
    static unsigned char Stepperstate = 0 ;                  // Statemachine
  
    if ( sc.StopNow )
    { Stepperstate = 6;                                      // Immidiate stop of stepper
      sc.TargetLoc = sc.CurrentLoc;
    }
  
    switch (Stepperstate) {
    case 0: //-----------------------------------------= Idle state
          if ( sc.trigger )
          { sc.active = true;                                // flag motor as moving
//          Serial.print(sc.CurrentLoc);                     //§DEBUG
            Stepperstate++;
          }                                                  //§TODO Check for limit,zero,65000
          break;
     case 1: //-----------------------------------------= Analyse stepper action to be done

          if      ( sc.TargetLoc > sc.CurrentLoc )           // Outward movement
          {  if      (sc.LashDir == 'I') Stepperstate = 4;
             else if (sc.LashDir == 'O') Stepperstate = 3;   // No backlash needed
             else                        Stepperstate = 3;   // No backlash used
             CalcMotorRamping();
          }
          else if ( sc.TargetLoc < sc.CurrentLoc )           // Inward
          {  if      (sc.LashDir == 'I') Stepperstate = 5;   // No backlash needed
             else if (sc.LashDir == 'O') Stepperstate = 2;
             else                        Stepperstate = 5;   // No backlash used
             CalcMotorRamping();
          }
          else                           Stepperstate = 6;   // Do nothing
          break;    
    case 2: //-----------------------------------------= Move Inward + 'O'backlash 
          if (sc.CurrentLoc <= (sc.TargetLoc-sc.Backlash) )
          { Stepperstate = 3;
          }
          else
          { motor.step(1, FORWARD, SINGLE); sc.CurrentLoc--;
            Serial.write('I');                                // ascom driver does not care if its 'I' or 'O'
            UpdateMotorSpeed(Stepperstate);
          }                                                   // just wants to know if its moving
          break;
    case 3: //-----------------------------------------= Move Outwards
          if (sc.CurrentLoc >= sc.TargetLoc )
          { Stepperstate = 6;
          }
          else
          { motor.step(1, BACKWARD, SINGLE); sc.CurrentLoc++;
            Serial.write('O');
            UpdateMotorSpeed(Stepperstate);
          }
          break;
    case 4: //-----------------------------------------= Move Outwards + 'I'backlash
          if (sc.CurrentLoc >= (sc.TargetLoc+sc.Backlash) )
          { Stepperstate = 5;
          }
          else
          { motor.step(1, BACKWARD, SINGLE); sc.CurrentLoc++;
            Serial.write('O');
            UpdateMotorSpeed(Stepperstate);
          }
          break;
    case 5: //-----------------------------------------= Move Inwards
          if (sc.CurrentLoc <= sc.TargetLoc )
          { Stepperstate = 6;
          }
          else
          { motor.step(1, FORWARD, SINGLE); sc.CurrentLoc--;
            Serial.write('I');
            UpdateMotorSpeed(Stepperstate);
          }
          break;
    case 6: //-----------------------------------------=  
          sc.active = false;                                 // set flag motor as NOT moving
          sc.trigger = false;                                // Reset trigger flag
          sc.SendFrame = true;                               // Tell serialStatemachine to send frame.
          rp.enabled = false;                                // Disable ramping
          motor.release();
          Stepperstate = 0;
//        Serial.print('_');Serial.print(sc.CurrentLoc);     //§DEBUG
          break;
    default: //----------------------------------------=
          Stepperstate = 0;
          break;
    }//switch
    
}//StepperMotor_StateMachine

//-------------------------------------------------------------------------------------
//- UpdateMotorSpeed() ,  Update motor speed
//- Note:                                                                             -
//-------------------------------------------------------------------------------------
void UpdateMotorSpeed(unsigned char state )
{
  if ( rp.enabled )
  {
    if ( state == 2 || state == 5 )            // Inward movements
    {
           if (sc.CurrentLoc > rp.Loc2) { IncMotorSpeed(); }
      else if (sc.CurrentLoc > rp.Loc3) { DecMotorSpeed(); }
      else                              { rp.Curspeed = rp.Minspeed ; rp.enabled = false ;}
    }
    else if ( state == 3 || state == 4 )       // Outward movements
    {
           if (sc.CurrentLoc < rp.Loc2) { IncMotorSpeed(); }
      else if (sc.CurrentLoc < rp.Loc3) { DecMotorSpeed(); }
      else                              { rp.Curspeed = rp.Minspeed ; rp.enabled = false ;}
    }
    else                                { rp.Curspeed = rp.Minspeed ; rp.enabled = false ;}
  }
  motor.setSpeed(rp.Curspeed);
  
}//UpdateMotorSpeed

void IncMotorSpeed(void)
{
  rp.Curspeed += rp.SpeedInc;
  if (rp.Curspeed > rp.Maxspeed) rp.Curspeed = rp.Maxspeed ;
}//IncMotorSpeed

void DecMotorSpeed(void)
{
  rp.Curspeed -= rp.SpeedInc;
  if (rp.Curspeed < rp.Minspeed) rp.Curspeed = rp.Minspeed;
}//IncMotorSpeed

//-------------------------------------------------------------------------------------
//- boolean CalcMotorRamping() ,  Calculate Pos2 and Pos3 for ramping
//- Note:                                                                             -
//-------------------------------------------------------------------------------------
boolean CalcMotorRamping(void)
{
  rp.ramp = (rp.Maxspeed - rp.Minspeed) / rp.SpeedInc ;  // how fast to speedup/dowm
  rp.enabled = true;
  rp.Curspeed = rp.Minspeed;
  motor.setSpeed(rp.Minspeed);
  
  if ( sc.TargetLoc > sc.CurrentLoc )                       // Outward movement
  {  rp.Loc3 = sc.TargetLoc - ( sc.Backlash<<1 );
     if (( rp.Loc3 - sc.CurrentLoc) <=  rp.ramp )
       rp.enabled = false;
     else if (( rp.Loc3 - sc.CurrentLoc) <= ( rp.ramp<<1 ))
       { rp.Loc2 = (rp.Loc3 - sc.CurrentLoc) >>1 ;          // halveway
         rp.Loc2 += sc.CurrentLoc;
       }
     else
       rp.Loc2 = (rp.Loc3 - rp.ramp);
  }
  else if ( sc.TargetLoc < sc.CurrentLoc )                  // Inward movement
  {  rp.Loc3 = sc.TargetLoc + ( sc.Backlash<<1 );
     if (( sc.CurrentLoc - rp.Loc3) <=  rp.ramp )
       rp.enabled = false;
     else if (( sc.CurrentLoc - rp.Loc3) <= ( rp.ramp<<1 ))
     { rp.Loc2 = (sc.CurrentLoc - rp.Loc3) >>1 ;
       rp.Loc2 += rp.Loc3 ;
     }
     else
       rp.Loc2 = (rp.Loc3 + rp.ramp);
  }

  Serial.write('§');Serial.print(sc.CurrentLoc);
  Serial.write('_');Serial.print(rp.ramp);                   //§Debug
  Serial.write('_');Serial.print(rp.Loc2);                   //§Debug
  Serial.write('_');Serial.print(rp.Loc3);                   //§Debug
  Serial.write('§');
  
  return(rp.enabled);
}//CalcMotorRamping()

//-------------------------------------------------------------------------------------
//- SerialCommunication_StateMachine, Receive, validate, 
//- Note: Flowchart of statemachineflow available                                                                                  -
//-------------------------------------------------------------------------------------
void SerialCommunication_StateMachine()
{
  static unsigned char Serialstate = 0 ;                      // Statemachine

  if (sc.SendFrame)                                           // req. send end of moving frame FG?xxx, FI?xx, FO?
  { sc.SendFrame=false;                                       // reset flag
    if ( (Serialstate <= 1) && (sc.active==false) )           // if 'no frame in' AND 'stepper not moving'
    {  Str2Bytecat(txframe,"FD0");
       MyUtoA(sc.CurrentLoc);                                 // 
       AddChecksum(txframe);
       Serial.write(txframe, 9);                              // Send delayed response frame
    }
  }//fi
  
  switch (Serialstate) {
    case 0: //-----------------------------------------=  Wait for first character
          if ( Serial.available() > 0 )
          { if (Serial.peek() == 'F')
            {  Serialstate++;
               MyTmr1 = 1000;                                 // Start timeout timer for frame
            }
            else
            {  Serial.read();                                 // flush one char.
            }
          }
          break;
    case 1: //-----------------------------------------=  Wait for full command frame (9 chars)
          if ( Serial.available() >= 9 ) 
          {
              int checksum = 0;                               // Buffer Copy and Checksum validation
              for (int i=0; i <= 8; i++)                      // Checksum is Sum of first 8 char in frame
              {  MySerBuf[i] = Serial.read();                 // Read frame from serial buffer
                 checksum += MySerBuf[i];                     
              }
              checksum -= MySerBuf[8];                        // correct for add of checksum itself
    
              if ( (byte)checksum == MySerBuf[8] )
              {  Serialstate++;                               // Checksum OK, go to Next State
              }
              else 
              {  Serialstate--;                               // Checksum FALSE, go to pervious State
                 Serial.println("Checkum wrong");             // §debug
              }
          }
          else if ( MyTmr1==0 )                               // timeout for frame
          {   Serialstate--;                                  // go to pervious State
              Serial.println("Frame Timeout");                // §debug
          }
          break;
    case 2: //-----------------------------------------=  Is the Stepper still executing a command ?
          if (sc.active)
          { sc.StopNow = true;                                // YES,
          }                                                   // Request stop motor, and wait for it
          else
             Serialstate++;                                   // No
          break;          
    case 3: //-----------------------------------------=  process received frame
          switch( ProcessFrame() ){
            case 1 :
            { Serial.write(txframe, 9);                       // Send response frame 
              break;
            }
            case 2 :
            { sc.SendFrame=false;                             // Send response frame 
              break;
            }                                                 // after motor move
          }
          Serialstate++;
          break;
    case 4: //-----------------------------------------=  dummy state
          Serialstate = 0;
          break;
    default: //----------------------------------------=  
          Serial.println("ErrState_SCSM");                   // §debug
          Serialstate = 0;
          break;
    }//switch
}//SerialCommunication_StateMachine

//-------------------------------------------------------------------------------------
//- ProcessFrame, 
//- Note: called from SerialCommunication_StateMachine() in state 2                   -
//- returns : 0- no reply , 1-reply directly, 2-reply after motor move 
//-------------------------------------------------------------------------------------
unsigned char ProcessFrame(void)
{ unsigned char transmit = 0;
  unsigned int endnumber = 0;
  char tmp[10];
  
  switch(MySerBuf[1])                                        // FCNNNNNNX
  {
    case 'V': //-----------------------------------------= Firmware Version
    { Str2Bytecat(txframe,"FV000111");                       // build response frame
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    case 'G': //-----------------------------------------= Goto position
    { ExtractNumber(&endnumber);
      if (endnumber == 0)
      { Str2Bytecat(txframe,"FD0");                          // build response frame
        MyUtoA(sc.CurrentLoc);
        AddChecksum(txframe);
        transmit = 1;                                        // frame can be transmitted
      }
      else
      { sc.TargetLoc = endnumber;
        if ( sc.TargetLoc > sc.LimitLoc ) sc.TargetLoc = sc.LimitLoc;
        sc.trigger = true;                                   // signal StepperMotor_StateMachine()
        Str2Bytecat(txframe,"FD0");                          // build part of response frame
        transmit = 2;                                        // frame transmitted later
      }
      break;
    }
    case 'I': //-----------------------------------------= Go inwards
    { ExtractNumber(&endnumber);
      if (sc.CurrentLoc > endnumber)                         // sc.target wil be > 0
         sc.TargetLoc = sc.CurrentLoc - endnumber;
      else sc.TargetLoc = 1;
      sc.trigger = true;                                     // signal StepperMotor_StateMachine()
      Str2Bytecat(txframe,"FI0");                            // build part of response frame
      transmit = 2;                                          // frame transmitted later
      break;
    }
    case 'O': //-----------------------------------------= Go outwards
    { ExtractNumber(&endnumber);
      sc.TargetLoc = sc.CurrentLoc + endnumber;
      if ( sc.TargetLoc > sc.LimitLoc ) sc.TargetLoc = sc.LimitLoc;
      sc.trigger = true;                                     // signal StepperMotor_StateMachine()
      Str2Bytecat(txframe,"FO0");                            // build part of response frame
      transmit = 2;                                          // frame transmitted later
      break;
    }
    case 'S': //-----------------------------------------= Set position coordinate
    { ExtractNumber(&endnumber);
      if (endnumber != 0)
      { sc.CurrentLoc = endnumber;
        if (sc.CurrentLoc > 64000) sc.CurrentLoc = 64000;
      }
      Str2Bytecat(txframe,"FS0");                            // build response frame
      MyUtoA(sc.CurrentLoc);
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    case 'L': //-----------------------------------------= Limit travel to position
    { ExtractNumber(&endnumber);
      if (endnumber != 0)
      { sc.LimitLoc = endnumber;
        if (sc.LimitLoc > 65000) sc.LimitLoc = 65000;
      } 
      Str2Bytecat(txframe,"FL0");                            // build response frame
      MyUtoA(sc.LimitLoc);
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    case 'P': //-----------------------------------------= Set discrete outputs
    { 
      if (!( (MySerBuf[4]=='0')&&(MySerBuf[5]=='0')&&(MySerBuf[6]=='0')&&(MySerBuf[7]=='0') ))
      { pm.ch1 = pm.ch2 = pm.ch3 = pm.ch4 = false;
        if (MySerBuf[4] == '2') pm.ch1 = true;
        if (MySerBuf[5] == '2') pm.ch2 = true;
        if (MySerBuf[6] == '2') pm.ch3 = true;
        if (MySerBuf[7] == '2') pm.ch4 = true;
      }
      Str2Bytecat(txframe,"FP001111");                       // build response frame
      if (pm.ch1) txframe[4] = '2';
      if (pm.ch2) txframe[5] = '2';
      if (pm.ch3) txframe[6] = '2';
      if (pm.ch4) txframe[7] = '2';
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    case 'B': //-----------------------------------------= Set Backlash
    { ExtractNumber(&endnumber);
      if (endnumber != 0)
      { sc.Backlash= (byte)(endnumber & 0x00FF);
  
             if ( MySerBuf[2] == '2' ) sc.LashDir = 'I';     // Inwards
        else if ( MySerBuf[2] == '3' ) sc.LashDir = 'O';     // Outwards
        else                           sc.LashDir = 'D';     // Disabled
        if (sc.Backlash==0)
        { sc.LashDir = 'D'; sc.Backlash++;                   // Disabled
        }
      }
  
      Str2Bytecat(txframe,"FB1");                            // FB1xxxxx build response frame
      if (sc.LashDir == 'I') txframe[2] = '2';               // FB2xxxxx
      if (sc.LashDir == 'O') txframe[2] = '3';               // FB3xxxxx
      MyUtoA(sc.Backlash);
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    case 'C': //-----------------------------------------= Set configuration
    { if ((MySerBuf[7] == '0') && (MySerBuf[6] == '0') && (MySerBuf[5] == '0'))
      {
        if (MySerBuf[2] != '0') 
        { sc.DutyCycle = MySerBuf[2];                        // [2] dutycycle  not [5]
          if (sc.DutyCycle > 250) sc.DutyCycle = 250;
        }
        if (MySerBuf[3] != '0')
        { sc.StepDelay = MySerBuf[3];                        // [3] stepdelay  not [6]
          if (sc.StepDelay > 64) sc.StepDelay = 64;
          if (sc.StepDelay == 0) sc.StepDelay =  1;
        }
        if (MySerBuf[4] != '0')
        { sc.StepSize  = MySerBuf[4];                        // [4] stepsize   not [7]
          if (sc.StepSize > 64) sc.StepSize = 64;
          if (sc.StepSize == 0) sc.StepSize =  1;
        }
      }
      Str2Bytecat(txframe,"FC123000");                       // build response frame
      txframe[2] = sc.DutyCycle;
      txframe[3] = sc.StepDelay;
      txframe[4] = sc.StepSize;
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    case 'T': //-----------------------------------------= Get Temperature
    { Str2Bytecat(txframe,"FT0");                            // build response frame
      MyUtoA(Temperature);
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    case 'X': //-----------------------------------------= Extended commandset
    { Str2Bytecat(txframe,"FXtended");                       // build response frame
      AddChecksum(txframe);
      transmit = 1;                                          // frame can be transmitted
      break;
    }
    default:  //-----------------------------------------= unknown command
    { transmit = 0;
      Serial.println("unknown cmnd");                        // §debug
      break;
    }   
  }//switch
    
  return( transmit );
}//ProcessFrame

//-------------------------------------------------------------------------------------
//- AddChecksum, Add a checksum to the serial transmit buffer
//-                                                                                   -
//-------------------------------------------------------------------------------------
unsigned char AddChecksum(unsigned char *str)
{
  unsigned char tempbyte = 0;
  for (int i=0; i<=7 ; i++ )  tempbyte +=  str[i];
  str[8] = tempbyte;
  return(tempbyte);
}//CalcChecksum

//-------------------------------------------------------------------------------------
//- Str2Bytecat Copy/Concatenate ascii char from a string to a bytebuffer.
//-                                                                                   -
//-------------------------------------------------------------------------------------
void Str2Bytecat(unsigned char *out, char *in)
{ unsigned char i = 0;
  while ( in[i] )                                        // check for EndOfString token
  { out[i] = (unsigned char)in[i];
    i++;
  }
}//Str2Bytecat

//-------------------------------------------------------------------------------------
//- ExtractNumber() , get number from frame ascii->bin
//-                                                                                   -
//-------------------------------------------------------------------------------------
boolean ExtractNumber(unsigned int *out)
{
  const unsigned int multiply[] = { 0,0,0,10000,1000,100,10,1,0 };
  boolean error = false;                                                 // 012345678
  for (int i=3; i<=7 ; i++ )                                             // FV?XXXXXZ
    if ( isdigit(MySerBuf[i]) )
    { *out += multiply[i] * (unsigned int)(MySerBuf[i] & 0x0F); 
    }
    else
    { error = true;
      break; 
    }  
    return(error);
}//ExtractNumber

//-------------------------------------------------------------------------------------
//- Sensor_StateMachine() , measurement of all (analog) sensors 
//-                          like Temp, Humidity, Barometric pressure, etc            -
//-------------------------------------------------------------------------------------
void Sensor_StateMachine(void)
{
  static unsigned char Sensorstate = 9;                     // Bootstate
  static unsigned char Senscnt = 255;
    
  switch (Sensorstate) {
    case 0: //-----------------------------------------= Wait for quiet moment 
          if (!sc.active)                                   // if Idle
          { if (Senscnt++ >= MAXSENSORS) Senscnt=0;
            Sensorstate++;
          }
          break;
    case 1: //-----------------------------------------= Do analog measurement and Average value 
          long tmp, Old;
          tmp = analogRead(Senscnt);                          // read analog pin
          Old = an.avgarray[Senscnt];                         // fast and accurate averaging  
          Old = ( Old - ( Old>>3 - tmp )) ;                   // fixed to (7*old+new)/8
          an.avgarray[Senscnt] = Old;                         // does not throw away bits in avg routine
          Sensorstate++;
          break;
    case 2: //-----------------------------------------= Calibrate , scale and limit
          float tmp1, tmp2;
          tmp2  = (float)(an.avgarray[0]>>3);      
          tmp1  = an.CalPar[Senscnt].a * pow(tmp2,2);         //   aX^2
          tmp1 += an.CalPar[Senscnt].b * tmp2;                // + bX
          tmp1 += an.CalPar[Senscnt].c ;                      // + c
          Sensorstate=0;
          switch (Senscnt){
            case 0: an.SurfTemp = (int)(tmp1) ; break;
            case 1: an.AmbTemp  = (int)(tmp1) ; break;
            case 2: an.AmbHum   = (int)(tmp1) ; break;
            case 3: an.BarPres  = (int)(tmp1) ; break;         //disabled
          }//switch
          break;
    case 3: //-----------------------------------------= 
          Sensorstate=0;
          break;
    case 9: //-----------------------------------------= Boot
          for ( byte i=0; i <MAXSENSORS ;i++)
          {  an.avgarray[i] = (analogRead(i))<<3 ;            // fill average buffers, contains real value*8
          }                                                   // extract real value : 
          Sensorstate=0;                                      //  (int)(an.avgarray[i]>>3) ;
          break;
    default:  //-----------------------------------------= unknown state
          Sensorstate=0;
          break;
  }//switch
  
}//Sensor_StateMachine

//-------------------------------------------------------------------------------------
//- Keypad_StateMachine
//-                                                                                   -
//-------------------------------------------------------------------------------------
void Keypad_StateMachine(void)
{
  static boolean keymode = false;
  static unsigned char keystate = 0;
  
  switch ( keystate ) {
    case 0: //-----------------------------------------=  Wait for keypress
          if ( digitalRead(ROTS) == LOW )
          { MyTmr3 = 10; // start 100 ms timer 
            keystate++;
          }
          else                                                   // poll rotary switch
          { 
            if ( digitalRead(ROTA) == LOW )
            {
              if ( digitalRead(ROTB) == HIGH ) 
              {   //if (sc.active) break;
                  sc.TargetLoc = sc.CurrentLoc + 1;                 // slow mode 1 step
                  if (keymode) sc.TargetLoc += 7;                   // fast mode -> 8steps
                  sc.trigger = true;
                  keystate=3;
              }
              else
              {   //if (sc.active) break;
                  sc.TargetLoc = sc.CurrentLoc - 1;                 // slow mode 1 step
                  if (keymode) sc.TargetLoc -= 7;                   // fast mode -> 8steps
                  sc.trigger = true;
                  keystate=3;
              }
            }
          }//else
          break;
    case 1: //-----------------------------------------=  check is key still pressed
          if ( !MyTmr3 ) break ;                          // wait for timer
          if ( digitalRead(ROTS) == LOW )
          { keymode = ~keymode;                         // toggle keymode
            keystate++;
          }
          else keystate=0;
          break;
    case 2: //-----------------------------------------=  Wait for key release
          if ( digitalRead(ROTS) == HIGH )
          { keystate = 0;
          }
          break;
    case 3: //-----------------------------------------=  Wait for rotery signal lock
          if ( digitalRead(ROTA) == HIGH ) keystate = 0;
          break;
    default: //-----------------------------------------=  
          keystate = 0;
          break;
  }//switch
}//Keypad_StateMachine


//-------------------------------------------------------------------------------------
//- MyUtoA, Unsigned integer to Ascii string zero padded to 5 digits, d300 -> "00300"
//-                                                                                   -
//-------------------------------------------------------------------------------------
void MyUtoA(unsigned int number)
{
    char tmp[10];

    utoa(number,tmp ,10);                                // uInt to String ,base10
    
    switch ( strlen(tmp) ) {
      case 0: //-----------------------------------------= empty string
      { Str2Bytecat(&txframe[3],"00000");
        break;
      }
      case 1: //-----------------------------------------= singles
      { Str2Bytecat(&txframe[3],"0000");                   // FG000004
        Str2Bytecat(&txframe[7], tmp );                    // maybe txframe[7]=tmp[0];
        break;
      }
      case 2: //-----------------------------------------= Decimals
      { Str2Bytecat(&txframe[3],"000");
        Str2Bytecat(&txframe[6], tmp );
        break;
      }
      case 3: //-----------------------------------------= Hundreds
      { Str2Bytecat(&txframe[3],"00");
        Str2Bytecat(&txframe[5], tmp );
        break;
      }
      case 4: //-----------------------------------------= Thousends
      { Str2Bytecat(&txframe[3],"0");                      // maybe txframe[3]='0';
        Str2Bytecat(&txframe[4], tmp );
        break;
      }
      case 5: //-----------------------------------------= TenThousends
      { Str2Bytecat(&txframe[3], tmp );
        break;
      }
      case 6: //-----------------------------------------= Hundredthousends
      { Str2Bytecat(&txframe[3], &tmp[1] );                   // !!exception clip to TenThousends
        break;
      }
      default:
      { Str2Bytecat(&txframe[3],"00000");
      }
    }//switch
    
}//MyUtoA



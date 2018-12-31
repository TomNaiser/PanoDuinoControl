

/*
  PanoDuinoControlV181231
  Author: Thomas Naiser
  
  31.12.18 Big Cleanup. Removed LSM303 acceleration sensor from project
  12.08.17 added additional camera trigger functionality for Sony Camera (exposure brackets -> HDR)
  12.08.17 its now possible to set the servo speed
  21.07.17 Added shutter pins for Sony Camera - Remote trigger cable (via optocoupler) focus and release
  07.03.16 Closed Loop Servo Control (PID) mit AS 5048B als Sensor
  30.05.14 Version number can be requested
  21.07.13 Improve  angle measurement by averaging over ten values
  19.07.13 read AS5048b rotary encoder (pan- and tilt angles)
  26.12.12 Check availablility of acceleration sensor. It can now be configured for working without LSM 303...
  22.12.12 Use of LSM 303(compass and acceleration sensor)
  15.12.12 Test: send diagnostic data to PC 
  02.10.2012 Remote shutter for Lumix FZ200 (pins 50 and 53)

*/

/**************************************************************************/

// We need the ams_as5048b library to interface the AS5048B magnetic rotary encoder from AMS over the I2C bus
#include <ams_as5048b.h>
#include <Wire.h>

//____________________________________________________________


#include <Wire.h>
#include <math.h>

#define AvNum 30

#define NumAv 2  //Number of values for averaging - rotary encoders

//AS5048B rotary encoders
//I2C Adresses of the sensors
#define PanSensorADDRESS 66
#define TiltSensorADDRESS 64

//global variables
boolean SendDebugData = true; //True: Send Debug Data (CSV params) to serial (plot  with Kst etc.): False: send no debug data
unsigned long timelast; //abs time at which the previous command was received
unsigned long time;
unsigned long timedif;  //time differenc between current command call and previous command call
boolean newCmdFlag;     //This flag indicates that the startByte of a new Command string is received after a long pause
const int shutterPin =  28;      // the number of the digital Pin for triggering the Canon Camera via CHDK
const int shutterPinLumix1 = 51; // Pin number for remote shutter pin 1 (Focus) of the LUMIX camera
const int shutterPinLumix2 = 53; // Pin number for remote shutter pin 2 (release) of the LUMIX camera
const int shutterPinSony=24;//remoteShutter Pin Sony A7
const int focusPinSony=26; //remoteFocus Pin Sony A7

int shutter = 0; //shutter indicator - for debug purposes

//State of the camera flash pin (optionally the flash hotshoe can be used to detect if (or when) the camera has really triggered a photo)
const byte CamFlashInterruptPin = 19;
volatile byte CamFlashState = HIGH;
boolean CamReadyForNewImage=true;
int shutterCount=0;


/* Global variables */
int led = 13;                //led pin : used as indicator for movement

float tcheading; //tilt-compensated heading
float pitch;   //orientation - tilt
float roll;    //orientation - roll
float degpitch; //tilt in degree

int j;
int loopcount;

//Variables for rotary encoder readout
boolean PanAxisSensorAvailable;
boolean TiltAxisSensorAvailable;
int HighByte;
int result;
float angle;
float PanAngle;
float TiltAngle;
float sumangle;
int FW_status;
boolean x;


const double pi = 3.14159265;

int pwm_x = 3;  //PWM control for motor outputs 1 and 2
int pwm_y = 9;  //PWM control for motor outputs 3 and 4
int dir_x = 2;  //direction control for motor outputs 1 and 2
int dir_y = 12;  //direction control for motor outputs 3 and 4

byte PWMOutput_x;
double Error_x[10];
double Accumulator_x;
double PID_x;
double PTerm_x;
double ITerm_x;
double DTerm_x;
double AccuDampFactor_x = 0;
int PIDOut_x;
double ActualPosition_x = 0;
double TargetPos_x = 0.0;
double TargetOffset_x = 70.0;
double PID_x_max = 100;
double AbsPos_x;
bool axisAtTarget_x = false;
float ShutDownThreshold_x=6.0; //1.0;  //Shut down the Motor control if the means squared encoder noise is smaller than this threshold  - this is do avoid ringing motor noise when the motor is on hold

byte PWMOutput_y;
double Error_y[10];
double Accumulator_y;
double PID_y;
double PTerm_y;
double ITerm_y;
double DTerm_y;
double AccuDampFactor_y = 0;
int PIDOut_y;
double ActualPosition_y = 0;
double TargetPos_y = 0.0;
double TargetOffset_y = 70.0;
double PID_y_max = 100;
double AbsPos_y;
bool axisAtTarget_y = false;
float ShutDownThreshold_y=3.0;

double SpeedX=0.6;
double SpeedY=0.4;

long count;

double  TotalOffset_x = 0;
double  TotalOffset_y = 0;
double  PreviousPosition_x = 0;
double  PreviousPosition_y = 0;

int zeroCrossCount_x = 0;
int zeroCrossCount_y = 0;

AMS_AS5048B mysensor_x(66);
AMS_AS5048B mysensor_y(64);

double PX=0;
double IX=0;
double DX=0;

double PY=0;
double IY=0;
double DY=0;


void setup() {
  delay (5000); //Time to connect and upload data 
  //_______________________________________________________________
  FW_status = 0;
  loopcount = 0;
  // initialize both serial ports:
  Serial.begin(57600);   //Arduino USB-Serial
  Serial3.begin(57600);  //Bluetooth module
  

  if (!Serial) FW_status = FW_status | 0x01; //no usb
  if (!Serial3) FW_status = FW_status | 0x04; // no bluetooth

  Serial.println("PanoDuinoControlV181231");
  Serial.println("by Thomas Naiser 2018");

  pinMode(led, OUTPUT);  //used as an indicator for movement
  j = 0;                 // acceleration-buffer-pointer =0;

  delay(30);

  timelast = millis();   //for timer initialization
  newCmdFlag = false;
  pinMode(shutterPin, OUTPUT);  // Init the Shutter Pins - Canon Powershot
  digitalWrite(shutterPin, LOW);

  pinMode(shutterPinLumix1, OUTPUT);  // Init the Shutter Pins - Lumix
  digitalWrite(shutterPinLumix1, LOW);
  pinMode(shutterPinLumix2, OUTPUT);  // Init the Shutter Pins
  digitalWrite(shutterPinLumix2, LOW);
  
  //Trigger an Interrupt if the trigger pin at the camera hot shoe is a the rising edge of the pulse
  pinMode(CamFlashInterruptPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(CamFlashInterruptPin), CamShutterDetected_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CamFlashInterruptPin), CamShutterDetected_ISR, FALLING);

  const int shutterPinSony=24;//remoteShutter Pin Sony A7
  const int focusPinSony=26; 

  pinMode(shutterPinSony, OUTPUT);  // Init the Shutter Pins - Sony A7
  digitalWrite(shutterPinSony, LOW);
  pinMode(focusPinSony, OUTPUT);  // Init the Shutter Pins
  digitalWrite(focusPinSony, LOW);

  //_______________________________________________________________


  count = 0;
  //Start serial
  //Serial.begin(115200);
  //while (!Serial) ; //wait until Serial ready


  //init AMS_AS5048B object
  mysensor_x.begin();
  //set clock wise counting
  mysensor_x.setClockWise(true);

  //init AMS_AS5048B object
  mysensor_y.begin();
  //set clock wise counting
  mysensor_y.setClockWise(true);

  //mysensor_y.zeroRegW(-1200); //Zero Offset
  mysensor_y.zeroRegW(-1700); //Zero Offset  - set encoder position to zero when the tilt axis is in horizontal position


  TotalOffset_x = 0;
  TotalOffset_y = 0;
  PreviousPosition_x = 0;
  PreviousPosition_y = 0;

  pinMode(pwm_x, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_y, OUTPUT);
  pinMode(dir_x, OUTPUT);
  pinMode(dir_y, OUTPUT);

  //At setup time motors aren't moving
  analogWrite(pwm_x, 0);
  analogWrite(pwm_y, 0);

  TotalOffset_x = 0;
  TotalOffset_y = 0;
}

//Interrupt Routine which is executed when the camera flash (hot shoe) is detected -> thus indication that an image was made
void CamShutterDetected_ISR() {
  //
  Serial.println("ISR called");
  CamFlashState = LOW;
  CamReadyForNewImage=true;
  shutterCount++;
  Serial.println(shutterCount);
}


void GetError_x(void)
{
 byte i = 0;
  double Delta;


  mysensor_x.updateMovingAvgExp();
  ActualPosition_x = mysensor_x.angleR(U_DEG, false);

  //Calculate Absolute Angle (without discontinuities)
  Delta = ActualPosition_x - PreviousPosition_x;

  //In case of 0/360 - crossing
  if (Delta > 180)
  {
    TotalOffset_x = TotalOffset_x - 360;
    zeroCrossCount_x--;
  }

  if (Delta < -180)
  {
    TotalOffset_x = TotalOffset_x + 360;
    zeroCrossCount_x++;
  }
  AbsPos_x = ActualPosition_x + TotalOffset_x;

  double DesiredPosition = TargetPos_x;

  // shift error values
  for (i = 9; i > 0; i--)
    Error_x[i] = Error_x[i - 1];
  // load new error into top array spot
  Error_x[0] = (double)DesiredPosition - AbsPos_x;

  PreviousPosition_x = ActualPosition_x;
}

void GetError_y(void)
{
  byte i = 0;
  double Delta;

  mysensor_y.updateMovingAvgExp();
  ActualPosition_y = mysensor_y.angleR(U_DEG, false);

  //Calculate Absolute Angle (without discontinuities)
  Delta = ActualPosition_y - PreviousPosition_y;

  if (Delta > 180)
  {
    TotalOffset_y = TotalOffset_y - 360;
    zeroCrossCount_y--;
  }

  if (Delta < -180)
  {
    TotalOffset_y = TotalOffset_y + 360;
    zeroCrossCount_y++;
  }

  AbsPos_y = ActualPosition_y + TotalOffset_y;

  double DesiredPosition = TargetPos_y;

  // shift error values
  for (i = 9; i > 0; i--)
    Error_y[i] = Error_y[i - 1];
  // load new error into top array spot
  Error_y[0] = (double)DesiredPosition - (double)ActualPosition_y - TotalOffset_y;

  PreviousPosition_y = ActualPosition_y;
}

void CalculatePID_x(void)
{
  // Set constants here
  PTerm_x =  70;
  ITerm_x = 0.25;
  DTerm_x = 100;
  AccuDampFactor_x = 1;
  double Delta_x = Error_x[0] - Error_x[1]; //Differential part

  // Calculate the 
  PX = Error_x[0] * PTerm_x;   // start with proportional gain
  PID_x=PX;
  Accumulator_x += Error_x[0];  // accumulator is sum of errors
  Accumulator_x = Accumulator_x * AccuDampFactor_x; //Dampening the AccumulatorGrowth
  
  IX= ITerm_x * Accumulator_x; // add integral gain and error accumulation
  PID_x+=IX;
  
  DX= DTerm_x * Delta_x; // differential gain comes next
  PID_x+=DX;
  
  if (Accumulator_x > 100) Accumulator_x = 100;
  if (Accumulator_x < -100) Accumulator_x = -100;

  // limit the PID to the range we have for the PWM variable
  if (PID_x >= PID_x_max)
    PID_x = PID_x_max;
  if (PID_x <= -PID_x_max)
    PID_x = -PID_x_max;  
}

void CalculatePID_y(void)
{
  // Set constants here
  PTerm_y = 75;
  ITerm_y = 0.25;
  DTerm_y = 200;


  AccuDampFactor_y = 1;
  double Delta_y = Error_y[0] - Error_y[1]; //Differential part

  // Calculate the PID
  PY = Error_y[0] * PTerm_y;   // start with proportional gain
  PID_y=PY;
  Accumulator_y += Error_y[0];  // accumulator is sum of errors
  Accumulator_y = Accumulator_y * AccuDampFactor_y; //Dampening the AccumulatorGrowth
  IY= ITerm_y * Accumulator_y; // add integral gain and error accumulation
  PID_y+=IY;
  
  DY= DTerm_y * Delta_y; // differential gain comes next
  PID_y+=DY;
  
  if (Accumulator_y > 100) Accumulator_y = 100;
  if (Accumulator_y < -100) Accumulator_y = -100;

  // limit the PID to the range we have for the PWM variable
  if (PID_y >= PID_y_max)
    PID_y = PID_y_max;
  if (PID_y <= -PID_y_max)
    PID_y = -PID_y_max;
}

//Get mean squared error of the position deviation
double mse_x()
{
  double mse = 0;
  // shift error values
  for (int i = 9; i > 0; i--)
    mse = mse + Error_x[i] * Error_x[i];
  return mse;
}

//Get mean squared error of the position deviation
double mse_y()
{
  double mse = 0;
  // shift error values
  for (int i = 9; i > 0; i--)
    mse = mse + Error_y[i] * Error_y[i];
  return mse;
}

//________________________________________________________________

//Set  the  zero degree point (pan-Axis) at the current Position
void SetZeroPos_x()
{
  mysensor_x.setZeroReg();
  Serial.println("SetZeroReg x");
}

//Set  the  zero degree point (tilt-Axis) at the current Position
void SetZeroPos_y()
{
  mysensor_y.setZeroReg();
  Serial.println("SetZeroReg y");
}

//Check if the target-position has been reached
bool CheckTargetReached_x()
{
  bool targetReached = false;
  if (abs(Error_x[0]) < 1.5 && mse_x() < 10.0)
  {
    targetReached = true;
  }

  if (targetReached == true)
  {
    FW_status = FW_status | 0x0100;
  }
  else
  {
    FW_status = FW_status & 0xFEFF;
  }
  return  targetReached;
}

bool CheckTargetReached_y()
{
  bool targetReached = false;
  if (abs(Error_y[0]) < 1.5 && mse_y() < 10.0)
  {
    targetReached = true;
  }

  if (targetReached == true)
  {
    FW_status = FW_status | 0x0200;
  }
  else
  {
    FW_status = FW_status & 0xFDFF;
  }
  return  targetReached;
}

void ResetMovement_x()
{
  for (int i = 0; i < 10; i++)  {
    Error_x[i] = 0;
  }
  Accumulator_x = 0;
  PID_x = 0;
  PTerm_x = 0;
  ITerm_x = 0;
  DTerm_x = 0;
  PIDOut_x = 0;
}

void ResetMovement_y()
{
  for (int i = 0; i < 10; i++)  {
    Error_y[i] = 0;
  }
  Accumulator_y = 0;
  PID_y = 0;
  PTerm_y = 0;
  ITerm_y = 0;
  DTerm_y = 0;
  PIDOut_y = 0;
}

//Read FloatValue from Serial-Port
float GetFloatFromSerial()
{
  float floatValue;
  int ByteCount = 0;
  byte  floatBuf[4];
  int inByte;

  while (ByteCount < 4) //Read the next 4 Bytes
  {
    if (Serial3.available())
    {
      inByte = Serial3.read(); //read byte from bluetooth
      floatBuf[ByteCount] = inByte;
      ByteCount++;
    }
  }
  //Calculate Float-Value
  byte * f_p = (byte *) &floatValue;

  f_p[0] = floatBuf[0];
  f_p[1] = floatBuf[1];
  f_p[2] = floatBuf[2];
  f_p[3] = floatBuf[3];

  return floatValue;
}



//send Debug values to serial
void SendDebugVals()
{
  //float PanAngle;
  //float TiltAngle;
  //float AcMean;
  //float AcVar
  if (SendDebugData)
  {
    /*
      Serial.print(millis(), DEC);
      Serial.write(';');

      Serial.print(PanAngle, DEC);
      Serial.write(';');

      Serial.print(TiltAngle, DEC);
      Serial.write(';');

      Serial.print(AcMean, DEC);
      Serial.write(';');

      Serial.print(AcVar, DEC);
      Serial.write(';');

      Serial.println(shutter, DEC);*/
  }

}


//ToDo: Überarbeiten
int checkstatus()
{
  if (!Serial) FW_status = FW_status | 0x01; //no usb

  if (!Serial3) FW_status = FW_status | 0x04; // no bluetooth

  return FW_status;
}





//________________________________________________________________

void loop() {
  //----------------------------------------------------------------
  loopcount++;  //counts the loops
  //  SendDebugVals();

  if (loopcount % 2 == 0) //Do communication only in one of 2 loops
  {
    //Parse and direct commands
    // read from from Bluetooth send to Arduino->PC and pololu
    if (Serial3.available()) 
    {
      //if the time between subsequent bytes is larger than 5 ms than  the NewCmdFlag is set true!
      time = millis();
      timedif = time - timelast;
      newCmdFlag = false;
      if (timedif > 5) //its more than 10 ms ago that the last Byte was received  -Changed from 10 to 5 ms
      {
        newCmdFlag = true;
      }

      int inByte = Serial3.read();
      if ((newCmdFlag == true) && (!(inByte == 0x80))) //if there is a new Command Sequence (the last byte came in long ago...)  which doesnt start with 0x80
      {
        //Important: In each case a certain time delay is necessary (200 ms works, but less might be ok too)
        //Then it could be the trigger
        if (inByte == 0xFF) //This is the signal for the camera trigger
        {
          //Serial.println("0xFF");
          shutter = 1;
          SendDebugVals();

          digitalWrite(shutterPin, HIGH);
          digitalWrite(shutterPinLumix1, HIGH);
          digitalWrite(shutterPinLumix2, HIGH);
          digitalWrite(focusPinSony,HIGH);
          digitalWrite(shutterPinSony,HIGH);
          delay(40);
      
          CamReadyForNewImage=false;
          Serial.print("0");
          digitalWrite(shutterPin, LOW);
          digitalWrite(shutterPinLumix2, LOW);
          digitalWrite(shutterPinLumix1, LOW);
          delay(80);

          digitalWrite(shutterPinSony,LOW);
          digitalWrite(focusPinSony,LOW);
          
          digitalWrite(shutterPin, HIGH);
          delay(200);
          digitalWrite(shutterPin, LOW);
          shutter = 0;
          Serial.println("Shutter was activated");
          SendDebugVals();
        }

        //Separate Camera Trigger High and Low Commmands.  This enables the PC-Application to control the Pulse length (useful for Bulb and Bracketed exposures )
        if (inByte==0xEE) //This is the Signal for Cameratrigger (Sony) - High
        {
          shutter=1;
          digitalWrite(focusPinSony,HIGH);
          digitalWrite(shutterPinSony,HIGH);
          SendDebugVals();
        }

        if (inByte==0xEF) //This is the Signal for Cameratrigger (Sony) - Low
        {
          shutter=0;
          digitalWrite(focusPinSony,LOW);
          digitalWrite(shutterPinSony,LOW);
          SendDebugVals();
        }

        //Get the Pan-Angle  from the AS5048B rotary encoder (via I2C) and send result (angle in degree- float value) to the bluetooth interface
        if (inByte == 0xFD) //Command Byte
        {
          Serial3.print("1 ");
          Serial3.println (AbsPos_x);
        }

        if (inByte == 0xFC) //request Pitch (tilt) angle 
        {
          Serial3.print("2 ");
          Serial3.println (AbsPos_y);
        }

        //Version request
        if (inByte == 0xFA)
        {
          checkstatus();
          Serial.println("PanoDuinoControlV181231");
        }

        if (inByte == 0xFB)
        {
          checkstatus();
          Serial3.println(FW_status, DEC);
        }


        //MoveServo Pan-Axis (x)
        if (inByte == 0xF8)
        {
          TargetPos_x = GetFloatFromSerial();
          ResetMovement_x();
          if (TargetPos_x > 360 || TargetPos_x < -360)
          {
            Serial.println("OutOfRange");
            Serial.println((long)TargetPos_x);
          }
        }


        //MoveServo Tilt-Axis (y)
        if (inByte == 0xF9)
        {
          TargetPos_y = GetFloatFromSerial();
          ResetMovement_y();
          if (TargetPos_y > 360 || TargetPos_y < -360)
          {
            Serial.println("OutOfRange");
            Serial.println((long)TargetPos_y);
          }
        }

        //Set current x-Pos to zero angle
        if (inByte == 0xF6)
        {
          SetZeroPos_x();
        }

        //Set current y-Pos to zero angle
        if (inByte == 0xF7)
        {
          SetZeroPos_y();
        }

        //Is the  x-Axis still moving, mse_x() tells us...
        if (inByte == 0xF5)
        {
          Serial3.print("3 ");
          Serial3.println(mse_x(), DEC);
        }

        //Is the  y-Axis still moving, mse_y() tells us...
        if (inByte == 0xF6)
        {
          Serial3.print("4 ");
          Serial3.println(mse_y(), DEC);
        }

        //Is the  x-Axis has reached its target position return true, else false
        if (inByte == 0xF3)
        {
          int targetReached = 0;
          if (CheckTargetReached_x() == true) targetReached = 1;
          else targetReached = 0;
          Serial3.print("5 ");
          Serial3.println(targetReached);
        }

        //Is the  y-Axis has reached its target position return true, else false
        if (inByte == 0xF4)
        {
          int targetReached = 0;
          if (CheckTargetReached_y() == true) targetReached = 1;
          else targetReached = 0;
          Serial3.print("6 ");
          Serial3.println(targetReached);
        }

        //GetShutDownThreshold x axis (jitter level below which the position is considered as constant => shut down H-bridge output)
        if (inByte == 0xF1)
        {       
          ShutDownThreshold_x = GetFloatFromSerial(); 
        }

        //GetShutDownThreshold  y axis
        if (inByte == 0xF2)
        {
          ShutDownThreshold_y = GetFloatFromSerial();
        }

        //Read Speed x-Axis
        if (inByte == 0xE8)
        {
          SpeedX = GetFloatFromSerial();
          if (SpeedX <0.0 || SpeedX >1.0)
          {
            Serial.println("OutOfRange");
            Serial.println(SpeedX);
          }
        }

        //Read Speed y-Axis
        if (inByte == 0xE9)
        {
          SpeedY = GetFloatFromSerial();
          if (SpeedY <0.0 || SpeedY >1.0)
          {
            Serial.println("OutOfRange");
            Serial.println(SpeedY);
          }
        }
    
    //Request if the last exposure has taken place, return 0  if this is not the case - otherwise return  1
        if (inByte ==0xF0)
        {   
          Serial3.print("9 ");
          //Serial.println("Cam status rqst");
          //Serial.println("8 ");
          if (CamReadyForNewImage==false)
          {
            Serial3.println(0);
            Serial.print("CRFNI=");
            Serial.println("0");
          }
          if (CamReadyForNewImage==true)
          {
            Serial3.println(1);
            Serial.print("CRFNI=");
            Serial.println("1");
          }
        }
        
      //Reset camera ready status from PC
        if (inByte==0xEF)
        {
         // Serial3.print("9 ");
         // Serial.println("Reset Cam rdy status");
         // Serial.println("9 ");
            //After a camera shutter feedback has been detected:  

          CamFlashState=HIGH;
      /*    Serial.println(" ");
          Serial.println("inByte=0xEF");
          Serial.println("CFlashSt=High");*/
          

          CamReadyForNewImage=true; 
          Serial.println("CamReset - ready for new image");
          
        }
      }
      timelast = millis();
    }
  }
  //----------------------------------------------------------------

  //Send status in every 10th loop
  if (loopcount % 10 == 0)
  {
    //Send Status
    CheckTargetReached_x();
    CheckTargetReached_y();
    Serial3.print("7 ");
    Serial3.println(FW_status);
  }

  double mseVal_x;
  double mseVal_y;


  if (TargetPos_x > 360) TargetPos_x = TargetPos_x - 360;

  GetError_x();
  CalculatePID_x();

  PIDOut_x = (int)(abs(PID_x)*SpeedX);

  delay(10);
  PID_x = -1 * PID_x;
  if (PID_x >= 0)
  {
    digitalWrite(dir_x, HIGH);
  }
  if (PID_x < 0)
  {
    digitalWrite(dir_x, LOW);
  }

  mseVal_x = mse_x();

  double Delta_x = Error_x[0] - Error_x[1]; //Differential part

  if (mseVal_x < ShutDownThreshold_x)PIDOut_x = 0; //Switch off motor if the mean squared  error of the position  is  very small : Silencing

  //Send PID output to H-Bridge
  analogWrite(pwm_x, PIDOut_x);


  //************************************************************************************
  //y-Axis *****************************************************************************

  // if (loopcount % 70 == 0)
  //   TargetPos_y = random(50,90);
  if (TargetPos_y > 360) TargetPos_y = TargetPos_y - 360;

  GetError_y();
  CalculatePID_y();

  PIDOut_y = (int)(abs(PID_y)*SpeedY);

  delay(10);
  if (PID_y >= 0)
  {
    digitalWrite(dir_y, HIGH);
  }
  if (PID_y < 0)
  {
    digitalWrite(dir_y, LOW);
  }

  mseVal_y = mse_y();

  double Delta_y = Error_y[0] - Error_y[1]; //Differential part

  if (mseVal_y < ShutDownThreshold_y)PIDOut_y = 0; //Motor Silencing: Switch off motor if the mean squared  error of the position  is  very small 

  //Send PID output to H-Bridge
  analogWrite(pwm_y, PIDOut_y);

 /* Serial.print(mseVal_x);
  Serial.print(',');
  Serial.println(mseVal_y); */

  int tarReached_x;
  int tarReached_y;

  if (CheckTargetReached_x())
  {tarReached_x=1;}
  else  {tarReached_x=0;}
  
  if (CheckTargetReached_y())
  {tarReached_y=1;}
  else {tarReached_y=0;}

/*
  Serial.print(tarReached_x);
  Serial.print(',');
  Serial.print(tarReached_y);
  Serial.print(',');
  
  Serial.print(TargetPos_x);
  Serial.print(',');
  Serial.print(TargetPos_y);
  Serial.print(',');

  Serial.print(ActualPosition_x);
  Serial.print(',');
  Serial.print(ActualPosition_y);
  Serial.print(',');

  Serial.print(AbsPos_x);
  Serial.print(',');
  Serial.print(AbsPos_y);


  Serial.print(',');
  Serial.print(FW_status);

  Serial.print(',');
  Serial.print(PIDOut_x);
*/
  //Serial.print(',');
  Serial.print(Error_x[0]);

  Serial.print(',');
  Serial.print(Error_y[0]);

  Serial.print(',');
  Serial.print(mse_x());
  
  Serial.print(',');
  Serial.println(mse_y());
  
  /*
  Serial.print(',');
  Serial.print(PIDOut_y);


  Serial.print(',');
  Serial.print(PY);
  
  Serial.print(',');
  Serial.print(IY);

  Serial.print(',');
  Serial.println(DY);
  */
  count = count + 1;
}








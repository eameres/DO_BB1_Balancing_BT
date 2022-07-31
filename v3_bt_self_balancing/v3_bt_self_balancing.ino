//  EraserMice - DO-BB1 driver code
//  July 2022 - Eric Ameres (aka erasermice)
//  input is from USB host shield with BlueTooth Dongle and a connected PS3 controller
//  can work with dual analog sticks (SixAxis) or single analog stick with analog trigger( Move Navigator )
//  IMU is MPU 6050
//
//  Unlike the original DO-BB1 system, this controls the DFPlayer directly, without a nano and separate RC channels
//  
//  I run this on an Arduino Mega with an old sparkfun USB shield  
//  so I avoid using normal servo library which ties up timer 1
//
// requires USBHostShield library for Bluetooth/PS3
// requires Servo_Hardware_PWM library to avoid timer conflicts between libraries
// requires DFRobotDFPlayerMini library for audio/MP3 playback support
// uses built in "wire" library to talk I2C with MPU6050 IMU for balancing


// features:

// movement is controlled by main analog stick, stick-button enables/disables IMU
//
// analog trigger controls servo according to last combination of D-pad buttons pressed
//
// select sound using X/O buttons, use L1 to play/pause

// 7/30/22  - added tilt trim, hold PS button and use up/down d-pad buttons to adjust
//          - adjust volume by holding PS button and using X/O buttons
// 
// todo : 
// - save tilt trim, volume settings in EEPROM
// - set joystick limits using PS button 
// - add LED support of some sort, maybe "neopixels", if not something simpler
// - add voice response messages for configuration and status

#include <Servo_Hardware_PWM.h>
Servo tiltServo;  // create servo object to control a servo
Servo panServo;  // create servo object to control a servo

#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;

#include <Wire.h>
#include "DFRobotDFPlayerMini.h"

DFRobotDFPlayerMini myDFPlayer;
bool soundPlaying = false;
void printDetail(uint8_t type, int value);

#define IMU_ENABLED
#define DFPLAYER
#define RUNMOTORS

// might no longer be applicable !
//#define myDebug
//#define myDebugGyro
//#define myDebugRC
//#define myDebugSpeeds
//#define myDebugPS3Analog

////////////////VARIABLE DEFINITION///////////////

const int dir1pin =47; //Motor Direction pin (goes to DIR1)
const int spe1pin =46; //Motor Speed pin (goes to PWM1)
const int dir2pin =45; //Motor Direction pin (goes to DIR2)
const int spe2pin =44; //Motor Speed pin (goes to PWM2)

const int panPWM = 6; // DO-BB1 has a spinning dome continuous rotation servo
const int tiltPWM = 5; // DO-BB1 uses a bar servo similar to DO

const int MPU = 0x68; // MPU6050 I2C address

float timePrev;
const float rad_to_deg = 180/3.141592654;

const int LoopInterval = 4;
const float tauScale = 1.0;

//  for estimating the IMU error profile

float AccErrorX, AccErrorY, AccErrorZ;
float GyroErrorX, GyroErrorY, GyroErrorZ;

int nextSound = 0,mp3Count = 0;

float heartBeat;

bool IMU_Enabled = true; // enable the balancing by default

void printDetail(uint8_t type, int value){ // detailed response from the DFPlayer
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      
      soundPlaying = false; //  ******* important to us, the mp3 file playback finished
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void calculate_IMU_error() {
  
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  float accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
  float roll, pitch, yaw;

  int c = 0;

  AccErrorX = AccErrorY = AccErrorZ = 0;
  GyroErrorX = GyroErrorY = GyroErrorZ = 0;

#ifdef IMU_ENABLED
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  
  // Read accelerometer values 200 times, rejecting bad attempts
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0)
      if (Wire.requestFrom(MPU, 6, true) == 6){
        AccX =  (Wire.read() << 8 | Wire.read()) ;
        AccY =  (Wire.read() << 8 | Wire.read()) ;
        AccZ =  (Wire.read() << 8 | Wire.read()) ;
        // Sum all readings
        AccErrorX += AccX;
        AccErrorY += AccY;
        AccErrorZ += AccZ;
        c++;
      }
      delay(5);
  }
  //Divide the sums by count to get the error value
  AccErrorX /= c;
  AccErrorY /= c;
  AccErrorZ /= c;
  
  c = 0;
  // Read gyro values 200 times, rejecting bad attempts
  while (c < 200) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    if (Wire.endTransmission(false) == 0)
      if (Wire.requestFrom(MPU, 6, true) == 6){
        GyroX =  (Wire.read() << 8 | Wire.read());
        GyroY =  (Wire.read() << 8 | Wire.read());
        GyroZ =  (Wire.read() << 8 | Wire.read());
        // Sum all readings
        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        GyroErrorZ += GyroZ;
        c++;
      }
      delay(5);
  }
  //Divide the sums by count to get the error value
  GyroErrorX /= c;
  GyroErrorY /= c;
  GyroErrorZ /= c;
  
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  #endif
}

void setup() 
{
  
  // set all IO ports to INPUT_PULLUP as a safe default
  
#if defined( PORTA )
  PORTA = 0xFF;
#endif
#if defined( PORTB )
  PORTB = 0xFF;
#endif
#if defined( PORTC )
  PORTC = 0xFF;
#endif
#if defined( PORTD )
  PORTD = 0xFF;
#endif

  Serial.begin(250000); // initialize the main serial port for debugging
  
  Serial.println ("\n***\nHello DO-BB1 ! \n");
  Serial.println ("\n***\nHello EraserMice! \n");
  
  Serial1.begin(9600); // initialize the second serial port for the DF MP3 player comms
  
#ifdef DFPLAYER   
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(Serial1)) {  //Use Serial1 to communicate with mp3, check for errors
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online.")); 
  
  Serial.println(myDFPlayer.readState()); //read mp3 state
  Serial.println(myDFPlayer.readVolume()); //read current volume
  Serial.println(myDFPlayer.readEQ()); //read EQ setting
  Serial.println(mp3Count = myDFPlayer.readFileCounts(DFPLAYER_DEVICE_SD)); //read EQ setting

  myDFPlayer.volume(15);  //Set volume value. From 0 to 30
  myDFPlayer.play(2);  //Play the first mp3
  
#endif

#ifdef IMU_ENABLED
  Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////

  Wire.setWireTimeout(5000, true); // enable 5000 microsecond timeout and reset the comms if timeout, prevents hanging on error !!
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  
  if (Wire.endTransmission(true) == 0){
    Serial.print(F("\r\nI2C comms active, init of IMU sent"));
  }else{
    Serial.print(F("\r\nWire comms failed, I2C init failed"));
    while (1); //halt, no I2C comms
  }
  
  pinMode(2,INPUT); // interrupt pin from the IMU
  
#endif

  //////////////// Motor Control PIN MODE DEFINITIONS//////////////////////
  
  pinMode(dir1pin,OUTPUT);
  pinMode(spe1pin,OUTPUT);
  pinMode(dir2pin,OUTPUT);
  pinMode(spe2pin,OUTPUT);

  tiltServo.attach(tiltPWM);  // attach the tilt servo to a servo object
  panServo.attach(panPWM);    // attach the pan servo to a servo object

  TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  heartBeat = timePrev = millis(); ///////////////STARTS COUNTING TIME IN MILLISECONDS/////////////
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    myDFPlayer.play(5);  //Play a different mp3 file to indicate error
    while (1); //halt, no USB host present
  }
  
  myDFPlayer.play(3);  //Play another mp3
  
  Serial.print(F("\r\nUSB Library Started"));
  Serial.print(F("\r\nWaiting for PS3 Controller"));
  
  while (!(PS3.PS3Connected || PS3.PS3NavigationConnected))
    Usb.Task();

  myDFPlayer.play(4);  //Play another mp3
  
  Serial.print(F("\r\nPS3 Controller Active "));
  
  calculate_IMU_error();
  
  myDFPlayer.play(5);  //Play another mp3
  
  Serial.println(F("\r\nIMU Calibrated "));
  Serial.println ("Finished Setup");
}

int timeOut = 0;

// I use function pointers in a "clever" way to avoid running code before initialization
// in order for the right "handler" to get installed, the initialization has to run and install it

typedef void (*GeneralFunction)(int s1,int d1,int s2,int d2);

void _doNothing(int s1,int d1,int s2,int d2) ;
void _writeToMotors(int s1,int d1,int s2,int d2) ;

GeneralFunction writeToMotors = _doNothing;

void _doNothing(int s1,int d1,int s2,int d2) 
{
  if (PS3.PS3Connected || PS3.PS3NavigationConnected)
    writeToMotors = _writeToMotors;
}

void _writeToMotors(int s1,int d1,int s2,int d2) 
{
  digitalWrite(dir1pin,d1);
  analogWrite(spe1pin,s1); //increase the speed of the motor from 0 to 255
  
  digitalWrite(dir2pin,d2);
  analogWrite(spe2pin,s2); //increase the speed of the motor from 0 to 255
}

//  similar to motor init procedure, 
// we "install" a do nothing PID function that does re-init and installs the real version

typedef float (*PID_Function)(float,float);

float initialPID(float target, float current) ;
float calculatePID(float target, float current);

PID_Function doPID = initialPID;
  
float errorSum = 0;       // Used to accumulate error (integral)
float lastTime = 0;    // Records the time the function was last called
float oldValue = 0;    // The last sensor value
  
float initialPID(float target, float current) {
  
  lastTime = millis();
  errorSum = 0;
  oldValue = 0;
  
  doPID = calculatePID;
  
  return 0.0; // don't do any compensation yet!
}

float calculatePID(float target, float current) { 

  const float Kp = 25;         // (P)roportional Tuning Parameter
  const float Ki = 0;          // (I)ntegral Tuning Parameter        
  const float Kd = .8;         // (D)erivative Tuning Parameter      
  
  const float maxPID = 200;    // The maximum value that can be output, in original code, was basically 700 !

  // Calculate the time since function was last called
  float thisTime = millis();
  float dT = (thisTime - lastTime)/1000;

  // Calculate error between target and current values
  float error = current - target;

  // Calculate the integral term (accumulated error)
  errorSum += error; 
  errorSum = constrain(errorSum,-500,500); // maximum error to accumulate

  // Multiply each term by its constant, and add it all up
  float result = (Kp * error) + (Ki * (errorSum * dT)) + (Kd * (current - oldValue) / dT);

/*
  Serial.print(" target: ");
  Serial.print( target);
  Serial.print(" current: ");
  Serial.print(current);
  Serial.print(" result: ");
  Serial.println(result);
*/  
  // Set old variable to equal new ones
  oldValue = current;
  lastTime = thisTime;

  // Limit PID value to maximum values
  // note that 2100 is a "magic" value used in the original code, subject to further research
  //
  result = map(result,-2100,2100,-maxPID,maxPID);
  
  return result ;
}

void loop() 
{
  float Acc_rawX, Acc_rawY, Acc_rawZ;
  float Acceleration_angle[2];
  
  float Gyr_rawX, Gyr_rawY, Gyr_rawZ;
  float Gyro_angle[2];
  
  static float Total_angle[2] = {0.0, 0.0 };
  static int tiltOffset = 0;

  int motorspeed1 = 0;
  int motordirection1 = HIGH;
  int motorspeed2 = 0 ;
  int motordirection2 = HIGH;

  Usb.Task();
  
  static int dPad = 0;
  int xAxis = 0,yAxis = 0;
  float pan = 90;
  float tilt = 90;
  float elapsedTime;
  
  if ((millis() - heartBeat) > 1000){
      Serial.print(".");
      heartBeat = millis();
  }
  
  // first check for button presses that don't directly effect the motion
  
  if (PS3.PS3Connected || PS3.PS3NavigationConnected){ // same for sixaxxis, or navigator

    if (PS3.getButtonClick(L3)){
      IMU_Enabled = !IMU_Enabled;
    }
    
    if (PS3.getButtonClick(CIRCLE)) {
      if (PS3.getButtonPress(PS))
        myDFPlayer.volumeUp();
      else
        if (nextSound < mp3Count) 
          nextSound++;
    }
    if (PS3.getButtonClick(CROSS)){
      if (PS3.getButtonPress(PS))
        myDFPlayer.volumeDown();
      else
        if (nextSound > 0) 
          nextSound--;
    }
    
    if (PS3.getButtonPress(PS)){
      if (PS3.getButtonClick(UP)) 
        tiltOffset++;
      else if (PS3.getButtonClick(DOWN)) 
        tiltOffset--;
    }
    
#ifdef DFPLAYER   
    if (PS3.getButtonClick(L1)){
    
      myDFPlayer.readState(); //read mp3 state
      
      if (!soundPlaying){
        myDFPlayer.play(nextSound);  //Play the current mp3
        soundPlaying = true;
      }else{
        myDFPlayer.pause();  //Play the first mp3
        soundPlaying = false;
      }
    }
#endif
  }else {
    writeToMotors(0, LOW, 0, LOW); // stop the motors if we don't have a controller connected !
    return; // no controller attached yet, do nothing else, return
  }
  static float timerValue = 0;
  
  if (millis() - timerValue < LoopInterval) // wait until minimum interval has passed to run the loop
    return;

  if (millis() - timerValue > 100){ // if more than 100 ms have passed, reset the timer and abort the loop, this must be the first time through
    timerValue = millis();
    return;
  }
  
  timerValue = millis();


  if (PS3.PS3Connected){ // sixaxis PS3 controller attached (dual analog sticks)
      int xAxisRaw,yAxisRaw;
    
      xAxisRaw = PS3.getAnalogHat(RightHatX) - 127;
      yAxisRaw = (255 - PS3.getAnalogHat(RightHatY)) - 127;

      // create a dead zone of -12 to 12 on each axis
      
      if (abs(xAxisRaw) < 12)
        xAxisRaw = 0;

      if (xAxisRaw > 0) xAxisRaw = map(xAxisRaw,12,127,0,127);
      else if (xAxisRaw < 0) xAxisRaw = map(xAxisRaw,-12,-127,0,-127);

      if (abs(yAxisRaw) < 12)
        yAxisRaw = 0;
        
      if (yAxisRaw > 0) yAxisRaw = map(yAxisRaw,12,127,0,127);
      else if (yAxisRaw < 0) yAxisRaw = map(yAxisRaw,-12,-127,0,-127);
        
      xAxis = constrain(yAxisRaw + xAxisRaw,-127,127);
      yAxis = constrain(yAxisRaw - xAxisRaw,-127,127);
        
      xAxisRaw = PS3.getAnalogHat(LeftHatX) - 127;
      yAxisRaw = (255 - PS3.getAnalogHat(LeftHatY)) - 127;
      
      pan = ((xAxisRaw+127)/255.0)*180.0;
      tilt = ((yAxisRaw+127)/255.0)*180.0;
  }else if (PS3.PS3NavigationConnected){ // single handed navigation control (PS3 move nav)
      
      int xAxisRaw,yAxisRaw;
    
      xAxisRaw = PS3.getAnalogHat(LeftHatX) - 127;
      yAxisRaw = (255 - PS3.getAnalogHat(LeftHatY)) - 127;

      /*
      Serial.print("xraw ");
      Serial.print(xAxisRaw);
      Serial.print("yraw ");
      Serial.print(yAxisRaw);
      */

      
      // create a dead zone of -12 to 12 on each axis
      
      if (abs(xAxisRaw) < 12)
       xAxisRaw = 0;

      if (xAxisRaw > 0) xAxisRaw = map(xAxisRaw,12,127,0,127);
      else if (xAxisRaw < 0) xAxisRaw = map(xAxisRaw,-12,-127,0,-127);

      if (abs(yAxisRaw) < 12)
        yAxisRaw = 0;
        
      if (yAxisRaw > 0) yAxisRaw = map(yAxisRaw,12,127,0,127);
      else if (yAxisRaw < 0) yAxisRaw = map(yAxisRaw,-12,-127,0,-127);

      /*
      Serial.print("xrawClipped ");
      Serial.print(xAxisRaw);
      Serial.print("yrawClipped ");
      Serial.print(yAxisRaw);
      */

      //  "mix" the 2 axes to translate from joystick to "tank-style" control, could be fancier, perhaps proportional.
      
      xAxis = constrain(yAxisRaw + xAxisRaw,-127,127);
      yAxis = constrain(yAxisRaw - xAxisRaw,-127,127);

      // make note of the last D-pad buttons clicked to set the "mode" of the analog trigger and servo control
      //
      if (PS3.getButtonClick(UP) || PS3.getButtonClick(DOWN) || PS3.getButtonClick(LEFT) || PS3.getButtonClick(RIGHT)){
          dPad = 0;
          if (PS3.getButtonPress(LEFT)) 
            dPad += 1;
          else if (PS3.getButtonPress(RIGHT)) 
            dPad += 2;
          
          if (PS3.getButtonPress(UP)) 
            dPad += 4;
          else if (PS3.getButtonPress(DOWN)) 
            dPad += 8;
      }

      yAxisRaw = xAxisRaw = PS3.getAnalogButton(L2); // get the magnitude of the servo control

      if (dPad & 1)
        xAxisRaw += 127;
      else if (dPad & 2)
        xAxisRaw = 127 - xAxisRaw;
      else 
        xAxisRaw = 127;

      if (dPad & 4)
        yAxisRaw += 127;
      else if (dPad & 8)
        yAxisRaw = 127 - yAxisRaw;
      else 
        yAxisRaw = 127;

      xAxisRaw = constrain( xAxisRaw, 0,255);
      yAxisRaw = constrain( yAxisRaw, 0,255);
      
      pan = (xAxisRaw/255.0)*180.0;
      tilt = ((yAxisRaw/255.0)*180.0) + tiltOffset; // apply the tilt offset ("trim")
  }
    
  panServo.write(pan);
  tiltServo.write(tilt);

  // scale the axes (-127 - 127) to the motor speeds  (-255 - 255)
  //
  motorspeed1 = xAxis *2;
  motorspeed2 = yAxis *2;
  
#ifdef myDebug

  bool pDebug;

  pDebug = true;

#endif


#ifdef myDebugSpeeds

  if(pDebug){
    Serial.print (",speed1: ");
    Serial.print (motorspeed1);
    Serial.print (",speed2: ");
    Serial.print (motorspeed2);
  }
#endif

#ifdef IMU_ENABLED

  /*////////////////////////WARNING//////////////////////
   * DO NOT USE ANY DELAYS INSIDE THE LOOP OTHERWISE THE BOT WON'T BE 
   * ABLE TO CORRECT THE BALANCE FAST ENOUGH
   * ALSO, DONT USE ANY SERIAL PRINTS. BASICALLY DONT SLOW DOWN THE LOOP SPEED.
   * 
   * ea: I very sparingly use Serial.print for a heartbeat (above), only once per second, 
   *     will change that to an LED blink eventually
  */
    float nTime = millis();
    
    elapsedTime = (nTime - timePrev) / 1000; // elapsed time since last attempt
    timePrev = nTime;
    
    // get the RAW ACCELEROMETER DATA FROM IMU
        
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); 
    if (Wire.endTransmission(false) == 0)
    {
      if (Wire.requestFrom(0x68,6,true) == 6)
      {
        Acc_rawX=  (Wire.read()<<8|Wire.read()); 
        Acc_rawY=  (Wire.read()<<8|Wire.read());
        Acc_rawZ=  (Wire.read()<<8|Wire.read()); 
      }
      else{ 
        //Serial.println("I2C IMU acc rcv err"); 
        return;
      }
    }
    else{ 
      //Serial.println("I2C IMU acc tx err"); 
      return;
    }
    
    //Acc_rawX -= AccErrorX; 
    Acc_rawY -= AccErrorY; 
    //Acc_rawZ -= AccErrorZ;

    //Acc_rawX = Acc_rawZ = 0;

    /*
    Serial.print("   Acc_rawX: ");
    Serial.print(Acc_rawX);
    Serial.print(" AccErrorX: ");
    Serial.print(AccErrorX);
    
    Serial.print("   Acc_rawY: ");
    Serial.print(Acc_rawY);
    Serial.print(" AccErrorY: ");
    Serial.print(AccErrorY);
    
    Serial.print("   Acc_rawZ: ");
    Serial.print(Acc_rawZ);
    Serial.print(" AccErrorZ: ");
    Serial.println(AccErrorZ);
    */
    // CONVERT RAW DATA TO ANGLES
    
    
    if ((Acc_rawX != 0) && (Acc_rawY != 0)){
      Acceleration_angle[0] = atan2( (Acc_rawY/16384.0),(Acc_rawZ/16384.0))*rad_to_deg;
      Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    }

    /*
    Serial.print(" Acceleration_angle[0]: ");
    Serial.println(Acceleration_angle[0]);
    */
    // get the RAW GYRO DATA FROM IMU
        
    Wire.beginTransmission(0x68);
    Wire.write(0x43); 
    if (Wire.endTransmission(false) == 0)
    {
      if (Wire.requestFrom(0x68,6,true) == 6)
      {
        Gyr_rawX=  (Wire.read()<<8|Wire.read()); 
        Gyr_rawY=  (Wire.read()<<8|Wire.read()); 
        Gyr_rawZ=  (Wire.read()<<8|Wire.read()); 
      }
      else{ 
        //Serial.println("I2C IMU gyro rcv err"); 
        return;
      }
    }
    else{ 
      //Serial.println("I2C IMU gyro tx err"); 
      return;
    }
    
  #ifdef myDebug
  if(pDebug){
    Serial.print (",AccX: ");
    Serial.print(Acc_rawX); 
    Serial.print(",AccY: ");
    Serial.print(Acc_rawY); 
    Serial.print (",GyroX: ");
    Serial.print(Gyr_rawX); 
    Serial.print(",GyroY: ");
    Serial.print(Gyr_rawY);
  }
  #endif
  
    Gyr_rawX -= GyroErrorX;
    //Gyr_rawY -= GyroErrorY;
    //Gyr_rawZ -= GyroErrorZ;

    //Gyr_rawX = Gyr_rawZ = 0;
    
    ////////////////////CONVERTING RAW DATA TO ANGLES///////////////////////
    Gyro_angle[0] = Gyr_rawX/131.0; 
    //Gyro_angle[1] = Gyr_rawY/131.0;

    /*
    Serial.print(" Gyro_angle[0]: ");
    Serial.print(Gyro_angle[0]);
    Serial.print(" GyroErrorX: ");
    Serial.println(GyroErrorX);
    */
    
    //////////////////////////////COMBINING BOTH ANGLES USING COMPLIMENTARY FILTER////////////////////////
    const float tau=0.075;
    float a=0.0;
    
    a=tau/(tau+(elapsedTime/tauScale)); // running at 4ms intervals, this sets high coeff at 1.67 (i.e. .978 and 0.022) (4ms is 1.33 which yields .983 and .017)
    
    Total_angle[0] = a *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + (1-a)*Acceleration_angle[0];
    //Total_angle[1] = a *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + (1-a)*Acceleration_angle[1];// really don't care about the orthogonal axis (roll)
    
    ////TOTAL_ANGLE[0] IS THE PITCH ANGLE WHICH WE NEED////////////

    // Serial.print("angle: ");
    // Serial.println(Total_angle[0]);

    float PID = doPID(0, Total_angle[0]); // target is to hold stable at calibrated 0 degrees
      
    if (IMU_Enabled){
      motorspeed1 -= PID; // add the user input and PID/IMU correction, motorspeed is from input, PID from the IMU
      motorspeed2 -= PID; // add the user input and PID/IMU correction, motorspeed is from input, PID from the IMU
    }

#endif 
    
    if (motorspeed1<0) {
      motordirection1 = LOW;
      motorspeed1=-motorspeed1;
    }    
    else {
     motordirection1 = HIGH;  
    }
    
    if (motorspeed2<0) {
      motordirection2 = LOW;
      motorspeed2=-motorspeed2;
    }
    else{
      motordirection2 = HIGH;  
    }

    // in case that pesky PID through it out of 0-255 range
    //
    motorspeed1 = constrain(motorspeed1,0,255); 
    motorspeed2 = constrain(motorspeed2,0,255); 
    
    writeToMotors(motorspeed1, motordirection1, motorspeed2, motordirection2);
    
    if (myDFPlayer.available()) {
      // this happens so seldomly it won't cause IMU problems either, I'll eventually change
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
}



       

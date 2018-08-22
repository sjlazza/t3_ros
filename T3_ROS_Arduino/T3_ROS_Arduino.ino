//Arduino Mega based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 4 ESCs for T3_Multirotor
//Modified for
//ROS competible system

//reference_publish:  http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//reference_subscribe:http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
//reference_array:    http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller

//To interact w. ROS on computer, run rosrun rosserial_python serial_node.py /dev/"@devicename@"

// Arduino Due can change its frequency by modifying the varient.h file
// C:\Users\sjlaz\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.10\variants\arduino_due_x
// Reference:         https://folk.uio.no/jeanra/Microelectronics/GettingTheBestOutOfPWMDue.html

// Update Logs
// 2016.05.04 First build
// 2016.05.11 Changed SW functions ex)calcSW
// 2017.01.18 Code modified for Arduino Due
// 2018.08.09 Code modified to operate under the ROS system w. rosserial
// 2018.08.15 Remapped the analogwrite->PWM relationship
// 2018.08.18 Activated 7th R/C channel

// Seung Jae Lee
// Seoul National University
// Department of Mechanical & Aerospace Engineering

// Loop Frequency : around 500 Hz
// Generated PWM Frequency : 455.846 Hz

// For ROS==============================================================================
#define USE_USBCON //use the following line if you have an arduino w. native port

#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
//--------------------------------------------------------------------------------------

#if defined(ARDUINO) && ARDUINO >=100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <Servo.h>

// For ROS==============================================================================
ros::NodeHandle nh;

std_msgs::Int16MultiArray RC;
ros::Publisher RC_readings("RC_readings", &RC);
//--------------------------------------------------------------------------------------

//Assign channel in pins================================================================
#define MASTERSWITCH_PIN      A3
#define AUTOMANUALSWITCH_PIN  A2
#define RC_ROLL_PIN           A7
#define RC_PITCH_PIN          A6
#define RC_YAW_PIN            A4
#define RC_THRUST_PIN         A5
#define RC_SERVO_SW_PIN       A1
//-------------------------------------------------------------------------------------

// Assign channel out pins=============================================================
#define PROP_ONE_OUT_PIN 7
#define PROP_TWO_OUT_PIN 6
#define PROP_THR_OUT_PIN 5
#define PROP_FOU_OUT_PIN 4
//-------------------------------------------------------------------------------------

//Flags===============================================================
#define SW_FLAG       1
#define ROLL_FLAG     2
#define PITCH_FLAG    4
#define YAW_FLAG      8
#define THRUST_FLAG   16
#define MSSW_FLAG     32
#define SERVO_FLAG    64

volatile uint8_t bUpdateFlagsShared;
//---------------------------------------------------------------------

//Set volatile variables===============================================================
// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals

volatile uint16_t unSWInShared;
volatile uint16_t unMSSWInShared;
volatile uint16_t unROLLInShared;
volatile uint16_t unPITCHInShared;
volatile uint16_t unYAWInShared;
volatile uint16_t unTHRUSTInShared;
volatile uint16_t unSERVOInShared;
//------------------------------------------------------------------------------------

//Pwm Rising Time recording variables=================================================
// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile

uint32_t ulSWStart;
uint32_t ulMSSWStart;
uint32_t ulROLLStart;
uint32_t ulPITCHStart;
uint32_t ulYAWStart;
uint32_t ulTHRUSTStart;
uint32_t ulSERVOStart;
//-----------------------------------------------------------------------------------

//Input variables====================================================================
int u1, u2, u3, u4;
//-----------------------------------------------------------------------------------


//Write PWM signals to motor ESCs===========================================================
void writePWM_Kill() {
  u1 = 1000;
  u2 = 1000;
  u3 = 1000;
  u4 = 1000;

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );

  writePWM(u1, u2, u3, u4);
}

void writePWM_Odroid(const std_msgs::Int16MultiArray &cmd_msg) {
  u1 = cmd_msg.data[0];
  u2 = cmd_msg.data[1];
  u3 = cmd_msg.data[2];
  u4 = cmd_msg.data[3];

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );
}

void writePWM(int ua1, int ua2, int ua3, int ua4) {

  ua1 = constrain( ua1, 1000, 2000 );
  ua2 = constrain( ua2, 1000, 2000 );
  ua3 = constrain( ua3, 1000, 2000 );
  ua4 = constrain( ua4, 1000, 2000 );

  //Write PWM Values to motor=============================================================
  analogWrite(PROP_ONE_OUT_PIN, ua1 * 1.866891 + 0.868104);
  analogWrite(PROP_TWO_OUT_PIN, ua2 * 1.866891 + 0.868104);
  analogWrite(PROP_THR_OUT_PIN, ua3 * 1.866891 + 0.868104);
  analogWrite(PROP_FOU_OUT_PIN, ua4 * 1.866891 + 0.868104);
  //--------------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------------------


//Interrupt service routine=================================================================

void calcSW() {
  if (digitalRead(AUTOMANUALSWITCH_PIN)) {
    ulSWStart = micros();
  }
  else  {
    unSWInShared = (uint16_t)(micros() - ulSWStart);
    bUpdateFlagsShared |= SW_FLAG;
  }
}

void calcMSSW() {
  if (digitalRead(MASTERSWITCH_PIN))  {
    ulMSSWStart = micros();
  }
  else  {
    unMSSWInShared = (uint16_t)(micros() - ulMSSWStart);
    bUpdateFlagsShared |= MSSW_FLAG;
  }
}

void calcROLL() {
  if (digitalRead(RC_ROLL_PIN))  {
    ulROLLStart = micros();
  }
  else  {
    unROLLInShared = (uint16_t)(micros() - ulROLLStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}

void calcPITCH() {
  if (digitalRead(RC_PITCH_PIN))  {
    ulPITCHStart = micros();
  }
  else  {
    unPITCHInShared = (uint16_t)(micros() - ulPITCHStart);
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}

void calcYAW() {
  if (digitalRead(RC_YAW_PIN))  {
    ulYAWStart = micros();
  }
  else  {
    unYAWInShared = (uint16_t)(micros() - ulYAWStart);
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void calcTHRUST() {
  if (digitalRead(RC_THRUST_PIN))  {
    ulTHRUSTStart = micros();
  }
  else  {
    unTHRUSTInShared = (uint16_t)(micros() - ulTHRUSTStart);
    bUpdateFlagsShared |= THRUST_FLAG;
  }
}

void calcSERVO() {
  if (digitalRead(RC_SERVO_SW_PIN)) {
    ulSERVOStart = micros();
  }
  else  {
    unSERVOInShared = (uint16_t)(micros() - ulSERVOStart);
    bUpdateFlagsShared |= SERVO_FLAG;
  }
}
//-------------------------------------------------------------------------------------------

// For ROS==================================================================================
ros::Subscriber<std_msgs::Int16MultiArray> PWMs("PWMs", writePWM_Odroid);
//------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(57600);

  // For ROS========================================================================
  nh.initNode();
  //RC.layout.dim=(std_msgs::MultiArrayDimension *)
  //malloc(sizeof(std_msgs::MultiArrayDimension) *2);
  //RC.layout.dim[0].label="RC";
  //RC.layout.dim[0].size=5;
  //RC.layout.dim[0].stride=1*5;
  //RC.layout.data_offset=0;
  RC.data = (short int *)malloc(sizeof(short int) * 6);
  RC.data_length = 6;
  nh.advertise(RC_readings);
  nh.subscribe(PWMs);
  //--------------------------------------------------------------------------------

  //Attach Interrupts===============================================================
  // used to read the channels
  attachInterrupt(AUTOMANUALSWITCH_PIN, calcSW, CHANGE);
  attachInterrupt(MASTERSWITCH_PIN, calcMSSW, CHANGE);
  attachInterrupt(RC_ROLL_PIN, calcROLL, CHANGE);
  attachInterrupt(RC_PITCH_PIN, calcPITCH, CHANGE);
  attachInterrupt(RC_YAW_PIN, calcYAW, CHANGE);
  attachInterrupt(RC_THRUST_PIN, calcTHRUST, CHANGE);
  attachInterrupt(RC_SERVO_SW_PIN, calcSERVO, CHANGE);
  //-------------------------------------------------------------------------------

  //Read&Write Resolution Setting==================================================
  analogReadResolution(12);
  analogWriteResolution(12);
  //-------------------------------------------------------------------------------

  //Pinmode Setting================================================================
  pinMode(PROP_ONE_OUT_PIN, OUTPUT);
  pinMode(PROP_TWO_OUT_PIN, OUTPUT);
  pinMode(PROP_THR_OUT_PIN, OUTPUT);
  pinMode(PROP_FOU_OUT_PIN, OUTPUT);

  //pinMode(9,OUTPUT); //used for checking loop time
  //pinMode(13,OUTPUT);//used for latency time check
  //-------------------------------------------------------------------------------
}

void loop() {

  //Input value=====================================================================
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unSWIn;
  static uint16_t unMSSWIn;
  static uint16_t unROLLIn;
  static uint16_t unPITCHIn;
  static uint16_t unYAWIn;
  static uint16_t unTHRUSTIn;
  static uint16_t unSERVOIn;
  //---------------------------------------------------------------------------------

  //Local copy of update flags======================================================
  static uint8_t bUpdateFlags;
  //---------------------------------------------------------------------------------

  //Update RCinput====================================================================

  if (bUpdateFlagsShared) {
    noInterrupts();
    bUpdateFlags = bUpdateFlagsShared;

    if (bUpdateFlags & SW_FLAG) {
      unSWIn = unSWInShared;
    }
    if (bUpdateFlags & MSSW_FLAG) {
      unMSSWIn = unMSSWInShared;
    }
    if (bUpdateFlags & ROLL_FLAG) {
      unROLLIn = unROLLInShared;
    }
    if (bUpdateFlags & PITCH_FLAG) {
      unPITCHIn = unPITCHInShared;
    }
    if (bUpdateFlags & YAW_FLAG) {
      unYAWIn = unYAWInShared;
    }
    if (bUpdateFlags & THRUST_FLAG) {
      unTHRUSTIn = unTHRUSTInShared;
    }
    if (bUpdateFlags & SERVO_FLAG) {
      unSERVOIn = unSERVOInShared;
    }
    
    bUpdateFlagsShared = 0;
    interrupts();
  }
  //----------------------------------------------------------------------------------------

  //Kill UAV================================================================================
  if ( unMSSWIn < 1500 ) {
    writePWM_Kill();
  }
  //----------------------------------------------------------------------------------------

  //Under Odroid's command==================================================================
  else {
    writePWM(u1, u2, u3, u4);
  }
  //----------------------------------------------------------------------------------------

  //Make constraint from 1000 to 2000 preventing overshoot==================================
  unSWIn = constrain( unSWIn, 1000, 2000 );
  unMSSWIn = constrain(unMSSWIn, 1000, 2000 );
  unROLLIn = constrain( unROLLIn, 1000, 2000 );
  unPITCHIn = constrain( unPITCHIn, 1000, 2000 );
  unYAWIn = constrain( unYAWIn, 1000, 2000 );
  unTHRUSTIn = constrain( unTHRUSTIn, 1000, 2000 );
  unSERVOIn = constrain( unSERVOIn, 1000, 2000 );
  //------------------------------------------------------------------------------------------


  // For ROS (Write to odroid ROS)==========================================================
  RC.data[0] = unROLLIn;
  RC.data[1] = unPITCHIn;
  RC.data[2] = unYAWIn;
  RC.data[3] = unTHRUSTIn;
  RC.data[4] = unSWIn;
  RC.data[5] = unSERVOIn;
  
  RC_readings.publish(&RC);
  nh.spinOnce();
  //----------------------------------------------------------------------------------------

  //For checking the latency btn cmd and response===========================================
  //if(unTHRUSTIn==u1){
  //  digitalWrite(13,HIGH);
  //}
  //else{
  //  digitalWrite(13,LOW);
  //}
  //----------------------------------------------------------------------------------------
  
  delay(2);
}


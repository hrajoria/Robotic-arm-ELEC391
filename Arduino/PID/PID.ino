/*
  PID

  This code is designed to control the postion of the motor. 
  Specifically to minimize the error between the actual and desired positions.


  This code is based on the Arduino example code "ReadAnalogVoltage".
  This code is based on the professor's example during the 01/02/2020 lecture.

  Ebi Sadeghi
  
  ----------------------------------------------------------------------------------
  
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
*/

#include "Math.h"

// Shoulder Motor Parameters
#define SHOULDER_MOTOR_PIN 13
#define SHOULDER_INPUT_PIN A0

#define SHOULDER_KP 12
#define SHOULDER_KD 1
#define SHOULDER_KI 1

int shoulder_e;
int shoulder_rc;

#define SHOULDER_SET_POINT PI/2 // radians, temporary

// Arm Motor Parameters
#define ARM_MOTOR_PIN 13
#define ARM_INPUT_PIN A0

#define ARM_KP 12
#define ARM_KD 1
#define ARM_KI 1

int arm_e;
int armr_rc;

#define ARM_SET_POINT PI/2 // radians, temporary

// Wrist Motor Parameters
#define WRIST_MOTOR_PIN 13
#define WRIST_INPUT_PIN A0

#define WRIST_KP 12
#define WRIST_KD 1
#define WRIST_KI 1

int wrist_e;
int wrist_rc;

#define WRIST_SET_POINT PI/2 // radians, temporary

// Finger Motor Parameters
#define FINGER_MOTOR_PIN 13
#define FINGER_INPUT_PIN A0

#define FINGER_KP 12
#define FINGER_KD 1
#define FINGER_KI 1

int finger_e;
int finger_rc;

#define FINGER_SET_POINT PI/2 // radians, temporary



// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int shoulder_sensorValue = analogRead(SHOULDER_INPUT_PIN);
  
  // Update the values
  shoulder_e = CalcErr(shoulder_sensorValue, SHOULDER_SET_POINT);
  shoulder_rc = RunPid(shoulder_e, SHOULDER_KP);

  // print out the value you read:
  digitalWrite(SHOULDER_MOTOR_PIN, shoulder_rc);
  Serial.println(shoulder_rc);

  // Delay by 1 millisecond
  delay(1);
}

int CalcErr(int sensorValue, int set_point) {
  int err;

  return err = set_point - sensorValue;
}

int RunPid(int e, int KP) { 
  return e*KP;
}

  // for more advanced PID
  /*
    x = (K_P)*(e) + K_I*integral{e} + K_D*derivative{e}
  
    integral{e} += (e_1 + e_2)/2 * (t_2 - t_1),
    or
    integral{e} += (e_1 + e_2)/2 * Δt,

    derivative(e) = (e_1 - e_2)/Δt
  ,
  */

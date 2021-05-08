#ISR Frequency count
//assignment(=) instances = 25 -> 25*4 = 100 cycles;
//pinMode instances = 4 -> 4*6 = 24 cycles;
//attachInterrupt = 1 -> 6 cycles
//for loops = 2 -> 2*5 = 10 cycles
//multiplication instances = 7 -> 7*18 = 126 cycles
//division instances = 2 -> 2*20 = 40 cycles
//+/- instances = 6+4 = 10 -> 10*18 = 180 cycles
//setup = 12; loop = 10 
//if = 1; else = 1; return = 2 -> 11 + 10 + 3*2 = 27 cycles
//analog and digital read and write = 3 -> 5 + 8 + 6 = 19 cycles
100+24+6+10+126+40+180+12+10+27+19 = 554 cycles

#include <math.h>
#define interruptPin 2
#define directionPin 3
#define setValPin 5
#define PIDout 6
#define CPR 128 //(counts per revolution - review this number)

double prevError = 0; //GLOBAL variable, at setup it's set to 0 and it will remember its previous value at every next function call.
double prevTime = 0;   
double pulse_count = 0;
double sensorValue = 0; //angle in degrees
double resolution = (2*pi)/CPR;    //how many radians are traversed in one pulse
double Kp = 5; //pre determined, tuned values fed into microcontroller
double Ki = 5;
double Kd = 5;
double setpoint, input, output;
int filter_pole = 5000;
n = 8; //# of samples we want to average to get a filtered derivative
double ept[n];  //ept vector used for D in PID later to get a filtered derivative. 
double unweighted_differential_outputs[n] = { 0 };  //initially assign all indices to 0

void setup() {
  // put your setup code here, to run once:
  pinMode(setValPin, INPUT);  //desired angle
  pinMode(PIDout, OUTPUT);    //output from the PID controller
  pinMode(directionPin, INPUT);
  pinMode(interruptPin, INPUT); //Jump to the ISR whenever this pin goes grom low to high and increment/decrement counter based on ISR condition
  attachInterrupt(digitalPinToInterrupt(interruptPin), pulse, RISING); //Whenever the pin goes from low to high, we check the status of directionPin to determine CW or CCW rotation

  //ept vector: vector of exponential decays as we average n number of samples for getting a *filtered* derivative -  will be used in the loop later. 
  for (int i = 1; i <= n; i++) {
     ept(i) = filter_pole*1.132*exp(-filter_pole*i*Ts);
  }
  
  Serial.begin(9600);

}
  
void loop() {
  // put your main code here, to run repeatedly:
  setpoint = analogRead(setValPin);        //desired angle
  sensorValue = resolution*pulse_count;   //actual angle -> converted from current pulse count to degrees based on optical encoder resolution 
  double error = CalcErr(sensorValue, setpoint);  //Desired - actual
  output = RunPid(error, Kp, Kd, Ki);        
  analogWrite(PIDout, output);          //output pin overwrites itself with PID modifications to the error
}

//Interrupt: whenever the interruptPin rises i.e. goes high, ISR starts and we look at the value of directionPin: if high, anticlockwise. Otherwise clockwise
void pulse(){
    int directionstate = digitalRead(directionPin);  
    if (directionstate == 1){
      pulse_count++;
    }
    else{
      pulse_count--;
    }
} //pulse_count can be converted to an angle using the resolution which gives an indication of sensorValue (actual angle)

double CalcErr(double setpoint, double sensorValue) {
  double error;
  return error = setpoint - sensorValue;
}

int RunPid(int error, int Kp, int Kd, int Ki) { 
  double currTime = millis();     //time since the program started running in ms
  double delT = currTime - prevTime;  //initially prevTime set to 0 - in MATLAB we set dT to a bit more than the ISR runtime to be conservative but here we are calculating the time difference between consecutive error points for each calculation.
  double delErr = error - prevError;  //initially prevError set to 0
  double filtered_derivative;
  
  double errorI  = errorI + ((error + prevError)/2)*delT;
  
  unweighted_differential_outputs[0] = (error - prevError);  //first index is the differential, as the code runs again and again, we will shift the first index over and the first index will get a new differential - so we keep shifting and updating this vector  
  filtered_derivative = dot(ept,unweighted_differential_outputs); //dot product with the ept vector takes care of the averaging and gets us a filtered derivative. 

//This for loop shifts each index one position to the left (index 0 value goes to 1, 1 goes 2, 2 goes to 3....)
  for count = 0:(n-2)
      unweighted_differential_outputs(n-count) = unweighted_differential_outputs(n - count - 1);    %shift values by one index, and in next loop calculate one more new slope at index 1 (starting index)
  end 
  
  Kd_output = Kd*filtered_derivative;
  
  prevError = error;  //for the next iteration, previous error and previous time become the current values.    
  prevTime  = currTime;
  double output = Kp*error + Ki*errorI + Kd_output;
  return output; 
}

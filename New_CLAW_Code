//Updated for new CLAW PCB
//correct pin definitions
//all inputs and outputs working according to CLAW_PCB_Function Test.ino
//Updated Servo.h to PWMServo.h. PWM Servo uses angle directly, so I created "mnimum_angle" and "maximum_angle", which are correct according to test code.
//Code compiles but usually the servo does not do anything and trying to change the angle such as using a command like "A140" does not do anything.
//work in progress


#include <DRV8833.h>
#include <HX711.h>
#include <string>
#include <PWMServo.h> //CHANGED

/******* Motor Driver Definition *******/
DRV8833 driver = DRV8833();
const int inputA1 = 9, inputA2 = 10;  //[9,10] for VCA
#define SLP 4

PWMServo servo;  // servo motor for pinching force CHANGED TO PWMSERVO
#define SERVO_PIN 6
const int minimum_angle = 115;
const int maximum_angle = 165;


int count = 0;

/******* Load Cell Definition *******/
HX711 scale;

float rawForce = 0;
float filtForceNew = 0; // current force
float filtForceLast = 0; // last force to calculate dF/dt

/******* Force PID Loop *******/

float defaultDesiredForce = 0.5f;
float stiffnessCoeficient = 10.00f;
float desForce = defaultDesiredForce; // desired Force we want to create

float currentForce = 0.0f; // predicted current force
float deltaForce = 0.0f; // predicted force change amount
float errorForce = 0.0f; // force error
float dErrorForce = 0.0f; // dF/dt for PD control
float dT = 0.011f; // 88Hz Force Sampling Rate

float timeSinceLastUpadte333Hz = 0.0f; // Making 333Hz PD loop   
float timeBetweenUpdates333Hz = 0.003; // Making 333Hz PD loop

double Kp = 2.0; //3.3
double Kd = 0.03; //0.2

int dAngle = 0; // Angle change command based on Force PD loop.
float dAngleFloat = 0.0f; // Angle change command based on Force PD loop.
int setAngle = 20; // Current angle & desired angle. we assume PID loop inside servo is perfect (open-loop).
int previousAngle = 0;
int speedLimitTouch = 4;
int speedLimitPinch = 5;

/******* Proxmity and Potentiometer Definition *******/ 
int proximity = 0;
int proximity_offset = 0;
int horizontal = 0;
int vertical = 0;

/******* Sampling Rate Measurement *******/
uint32_t delt_t = 0, sumCount = 0;        // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

/******* Voise Coil parametrs ******/
float timeSinceLastUpadte = 0.0f;       
float updateFrequency = 100.0f;
float timeBetweenUpdates;

int   patternLength = 500;
int   hapticPattern[ 500 ];
int   patternIndex = 0;

/******* Modes parametrs ******/
int startAngle = 140;
float stiffness = 0;    // 0 - 

int object_angle = minimum_angle;
// int gun_angle_1 = 250;
// int gun_angle_2 = 350;

const int DISABLED_MODE = 0;
const int PINCHING_MODE = 1;
const int TOUCHING_MODE = 2;
const int GUN_MODE = 3;

int mode=DISABLED_MODE;

void setup() {
  
  Serial.begin(2000000);
  Serial.setTimeout(1);

  //while (!Serial); // Wait for the serial port to connect.

  /******* Load Cell Setup *******/
  
  scale.begin(18, 19); //scale.begin(SDA, SCK); // parameter "gain" is ommited; the default value 128 is used by the library

  scale.set_scale(-210.f);  // UPDATED USING CALLIBRATION SKETCH FROM SPARKFUN: https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide
  scale.tare(); // reset the scale to 0
  
  /******* Motor Driver Setup *******/
  servo.attach(SERVO_PIN);
  
  analogWriteResolution(10);
  
  driver.attachMotorA(inputA1, inputA2); // Texturing Voice Coil
//  driver.attachMotorB(inputB1, inputB2); // Main Body Vibration
  driver.motorAStop();
  driver.motorBStop();

  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH);

  /******* Voice Coil Setup *******/
  for ( int i = 0; i < patternLength; ++i ){ 
    hapticPattern[i] = 0;
  }

  timeBetweenUpdates = 1.0f / updateFrequency;
}

void SetMode(String val)
{
  char md = val.charAt(0);
  switch(md)
  {
    case 'P': mode = PINCHING_MODE; break;
    case 'T': mode = TOUCHING_MODE; break;
    default:
    case 'D': mode = DISABLED_MODE; break;
  }
}

double DoAngleCommand(String val) { //UPDATED WITH SERVO VALUES 700, 2400
    double angle = val.toFloat();

    if (angle > maximum_angle) {
        angle = maximum_angle;
    } else if (angle < minimum_angle) {
        angle = minimum_angle;
    }

    return angle;
}

//
// Pattern playing amplitude.
// 0 - 350.
//
void DoPatternCommand(String val)
{
  int amp = val.toInt();

  if(amp > 500) {
    amp = 500;
  }

  if(amp < 0) {
    amp = 0;
  }

  for ( int i = 0; i < patternLength; ++i )
  {
    hapticPattern[i] = random(-amp, amp);
  }

  /*
  Serial.println ("New Patern: ");
  for ( int i = 0; i < patternLength; ++i )
  {
    Serial.print ("[");
    Serial.print (i);
    Serial.print ("] ");
    Serial.println (hapticPattern[i]);
  }
  */
}

//
// Pattern playing frequency.
// 20 - 200Hz.
// 
void DoFreqCommand(String val)
{
  float freq = val.toFloat();

  if(freq > 15000)
  {
    freq = 15000;
  }

  else if(freq < 1)
  {
    freq = 1;
  }  

  updateFrequency = freq;
  timeBetweenUpdates = 1.0f / updateFrequency;
}

void loop()
{

  horizontal = analogRead(A3);      // measure horizontal axis of potentiometer
  vertical = analogRead(A2);        // measure vertical axis of potentiometer 
  proximity = analogRead(A6) - proximity_offset;       // measure proximty of thumb

  if (mode == PINCHING_MODE || mode == DISABLED_MODE) 
  {
    //if (filtForceNew > 0.40){
    //  defaultDesiredForce = 0.30f;
    //}

    if (filtForceNew < 1.8){
      defaultDesiredForce = 2.0f;
    }
    
    //else if (filtForceNew < -0.15){
    //  defaultDesiredForce = 1.3f;
    //}

    else{
      defaultDesiredForce = 0.4f;
    }
  }

  /*
  if (mode == TOUCHING_MODE)
  {
    defaultDesiredForce = -0.5f;
  }
  */
  
  //
  // In Gun Mode, the 'object_angle defines the static angle of the device.
  // To generate a sword - just set object_angle to 0.
  //
  
  if (mode == GUN_MODE)
  {
    defaultDesiredForce = 0.0f;
  }
  
  //
  // This loop updates the force mesurement.
  // Currently, this loop is 88Hz due to the ADC limit.
  //
  
  if (scale.is_ready()) {
    // 
    // Force filter ( 4 to 1 ratio )
    //
    rawForce = scale.get_units();      // measure index finger force.
    filtForceNew = 0.75 * filtForceLast + 0.25 * rawForce;
    currentForce = filtForceNew;
    dErrorForce = (filtForceNew - filtForceLast) / dT; // dF/dt
    filtForceLast = filtForceNew;
  }
  
  //
  // This is PD loop for force control.
  // The loop is 333 hz and predict next force based on the filtered 88hz measured force.
  //
  
  timeSinceLastUpadte333Hz += deltat;
  
  if (timeSinceLastUpadte333Hz>=timeBetweenUpdates333Hz)
  {
    //
    // If there is an object - generate a resistive spring force, according to the penetration distance.
    // current angle is setAngle. current object surface is at object_angle.
    //
    
    if (setAngle >= object_angle) 
    {
      desForce = defaultDesiredForce;
    }
   
    else
    {
       float penetration = object_angle - setAngle;
       desForce = penetration * stiffness;
    }

    deltaForce = dErrorForce * timeSinceLastUpadte333Hz;
    currentForce = currentForce + deltaForce;
    
    errorForce = desForce - currentForce;

    dAngleFloat = Kp * errorForce - Kd * dErrorForce; // angle change amount based on force PD loop.
                                                 // Kd term is negative since desired dF/dt is always zero. 

    if ( abs(errorForce) < 0.008 )
    {
      dAngleFloat = 0;
    }

    previousAngle = setAngle;
    dAngle = dAngleFloat;
    setAngle = setAngle + dAngle;

    if (stiffness>=(10/stiffnessCoeficient)) 
    {
      //
      // If stiffness command was bigger than '1000' - then it is a rigid object.
      //
        if (setAngle < object_angle)
        {
            setAngle = object_angle; //minimum_angle;
        }
    }
    
    if (mode==PINCHING_MODE){
      if (dAngle > speedLimitPinch && previousAngle < object_angle){
        setAngle = previousAngle + speedLimitPinch;
        if (setAngle > object_angle){
          setAngle = object_angle;
        }
      }
    }
    
    if (mode==TOUCHING_MODE){
      
      setAngle = object_angle; // Touching mode is position controlled only!
      
      /*
      if (setAngle > object_angle){
        setAngle = setAngle - speedLimitTouch;
        if (setAngle < object_angle){
          setAngle = object_angle;
        }
      }

      if (dAngle > speedLimitTouch){
        setAngle = previousAngle + speedLimitTouch;
        if (setAngle > object_angle){
          setAngle = object_angle;
        }
      }
      */
    }

    // if (mode==GUN_MODE){
    //   if (filtForceNew > 4.0)
    //   {
    //     setAngle = gun_angle_1;
    //     driver.motorBForward(700);
    //   }
    //   else
    //   {
    //     setAngle = gun_angle_2;
    //     driver.motorBStop();
    //   }
    //   //setAngle = object_angle;
    // }

    if (setAngle > maximum_angle)
    {
      setAngle = maximum_angle;
    }
    
    if (setAngle <  minimum_angle)
    {
        setAngle = minimum_angle;
    }
    
    servo.write(setAngle);                  // sets the servo position according to the scaled value
      
    timeSinceLastUpadte333Hz = 0.0f;
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  
  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  timeSinceLastUpadte += deltat;       
  
  if (timeSinceLastUpadte>=timeBetweenUpdates)
  {
    /* This is the tiime to update the coil */

    //Serial.print ("[");
    //Serial.print (patternIndex);
    //Serial.print ("] ");

    if (timeBetweenUpdates < 1.0) {
    
      if (hapticPattern[ patternIndex ]>=0)
      {
        analogWriteFrequency(9, 21000);
        analogWriteFrequency(10, 21000);
        driver.motorAForward(hapticPattern[ patternIndex ]);
        //Serial.print ("F ");
        //Serial.println (hapticPattern[ patternIndex ]);
      }
      else
      {
        analogWriteFrequency(9, 21000);
        analogWriteFrequency(10, 21000);
        driver.motorAReverse( -hapticPattern[ patternIndex ] );
        //Serial.print ("R ");
        //Serial.println ( hapticPattern[ patternIndex ]);
      }
    }

    else {
      driver.motorAStop();
    }

    patternIndex += 1;
    if (patternIndex>=patternLength)
    {
      patternIndex = 0;
    }
    timeSinceLastUpadte = 0.0f;
  }
    
  while (Serial.available()){
    
    String st = Serial.readString();
    char commnd = st.charAt(0);
    String val = st.substring(1, st.length() );

    //Serial.println ("Command " + commnd);
    //Serial.print ("value " + val);

    //
    // Roation angle
    //
    
    if (commnd == 'A')  {
      setAngle = DoAngleCommand(val);
    }

    if (commnd == 'S') {
      stiffness = (val.toFloat())/(100.0f * stiffnessCoeficient);
      if (stiffness < 0.01f) stiffness = 0.01f;
    }
    
    if (commnd == 'O')  {
      
      object_angle = DoAngleCommand(val);
      if (object_angle < minimum_angle)
          object_angle = minimum_angle;

     /* wishfulAngle  = DoAngleCommand(val);
      if (wishfulAngle<minimum_angle)
          wishfulAngle = minimum_angle;*/
    }
    
    //
    // A pattern: a sequence of '50' numbers (0 to 255).
    //
       
    if (commnd == 'P')  {
      DoPatternCommand(val);
    }

    if (commnd == 'M')  {
      SetMode(val);
    }
    
    //
    // Frequency (20 - 200)
    //
    
    if (commnd == 'F')  {
      DoFreqCommand(val);
    }
  }

  //if ((count % 10 == 0))
  {
    Serial.print (setAngle);
    Serial.print (" ");
    Serial.print (proximity, DEC);
    Serial.print (" ");
    Serial.print (filtForceNew, 2);
    Serial.print (" ");
    Serial.print (stiffness*(100.0f*stiffnessCoeficient), 2); 
    Serial.print (" ");
    Serial.print (vertical, DEC);
    Serial.print (" ");
    Serial.println (horizontal, DEC); // horizontal
    Serial.flush();
    //Serial.print (" ");
    //Serial.println (count, DEC);
    //Serial.println((float)sumCount/sum, 2); // rate
    //count = 0;
  }
  count++;
  
  sumCount = 0;
  sum = 0;
}

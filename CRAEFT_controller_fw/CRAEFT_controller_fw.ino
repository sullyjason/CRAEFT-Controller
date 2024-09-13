/* IMPORT LIBRARIES ***********************************************************/
#include <DRV8833.h>
#include <HX711.h>
#include <string>
#include <PWMServo.h> //CHANGED

/* PIN DEFINITIONS ************************************************************/
#define MOTOR_SLEEP             4
#define VIBRATION_COIL_INPUT_A1 9
#define VIBRATION_COIL_INPUT_A2 10
#define SERVO_PIN               6
#define THUMBSTICK_X            A3
#define THUMBSTICK_Y            A2
#define PUSHBUTTON              15
#define LIGHT_SENSOR            A6
#define LOADCELL_DATA           18
#define LOADCELL_CLOCK          19

/* GLOBAL VARIABLES ***********************************************************/

#define DEBUG 1


/* Main loop timing */
float deltat = 0.0f; 
float lastLoopTime = 0.0f;

/* Operating modes */
typedef enum {
    DISABLED_MODE =0,
    PINCHING_MODE,
    TOUCHING_MODE,
} operating_mode_t;

operating_mode_t op_mode = DISABLED_MODE;


/* Vibration coil parameters */
DRV8833 driver = DRV8833();
#define HAPTIC_PATTERN_LEN      500         // Haptic pattern array length
#define HAPTIC_AMPLITUDE_MIN    0
#define HAPTIC_AMPLITUDE_MAX    150
#define HAPTIC_FREQUENCY_MIN    1
#define HAPTIC_FREQUENCY_MAX    15000

float haptic_updateFrequency = 100.0f;      // Frequency of the vibration pattern
float haptic_timeBetweenUpdates = 0;
float haptic_timeSinceLastUpdate = 0;

int haptic_pattern[HAPTIC_PATTERN_LEN];     // Pattern array for vibration
int haptic_patternIndex = 0;                // Index in the vibration pattern
int haptic_amp = 0;                         // Set amplitude for VCA vibration

/* Servo parameters */
PWMServo servo;
#define SERVO_ANGLE_MIN         115
#define SERVO_ANGLE_MAX         165
#define INIT_SERVO_ANGLE        140

int dAngle = 0;            // Angle change command based on Force PD loop.
float dAngleFloat = 0.0f;  // Angle change command based on Force PD loop.
int setAngle = INIT_SERVO_ANGLE;         // Current angle & desired angle. we assume PID loop inside servo is perfect (open-loop).
int previousAngle = INIT_SERVO_ANGLE;
int objectAngle = SERVO_ANGLE_MIN; // Servo angle at which the object collides with the finger

/* Load cell parameters */
HX711 scale;
#define LOADCELL_TIME_BETWEEN_SAMPLES     0.011 // Force sampling rate of 88Hz
#define BASE_LOADCELL_CALIBRATION         -2100
#define OZ_TO_GRAMS                       453.592

int loadcell_calibration_scale = BASE_LOADCELL_CALIBRATION;

/* Thumb control parameters */
int thumbstick_position[2] = {0};  // thumbstick x and y
int pushbutton_press = 0;          // pushbutton state
int light_sensor_value = 0;        // value of light sensor

/* PID control loop */
#define PID_TIME_BETWEEN_UPDATES    0.003f // run PID control loop at 333Hz
#define PID_KP                      2.0f
#define PID_KD                      0.03f

#define PID_STIFFNESS_MIN           0.01f
#define PID_STIFFNESS_DEFAULT       1.0f
#define PID_STIFFNESS_COEFFICIENT   10.00f

// pid loop timing variables
float pid_timeSinceLastUpdate = 0.0f;

float stiffness = PID_STIFFNESS_DEFAULT;

float rawForce = 0;       // raw force reading
float filtForceNew = 0;   // current force
float filtForceLast = 0;  // last force to calculate dF/dt
float defaultDesiredForce = 0.5f;
float desiredForce = defaultDesiredForce;  // desired Force we want to create

float currentForce = 0.0f;  // predicted current force
float deltaForce = 0.0f;    // predicted force change amount
float errorForce = 0.0f;    // force error
float dErrorForce = 0.0f;   // dF/dt for PD control

int speedLimitPinch = 5;


/* SETUP **********************************************************************/
void setup() {
    Serial.begin(2000000);  // Serial communication
    Serial.setTimeout(1);

    /* setup hardware components */
    servo_setup();
    vibration_coil_setup();
    loadcell_setup();
    thumb_controls_setup();
}


/* MAIN LOOP ******************************************************************/
void loop()
{
    /* check for input commands from Serial port */
    processSerialCommands();

    /* update desired force based on operating mode */
    updateDefaultDesiredForce();

    /* At a rate of 88Hz : update force reading */
    updateForceReading();

    /* At a rate of 333Hz: update main PID loop */
    mainPIDloop();
    
    /* At a rate defined by user command: update the vibration coil */
    playHapticPattern();

    /* update the thumb controls values */
    updateThumbControls();

    /* communicate current state */
   sendCurrentState();
}






/* SETUP FUNCTIONS  ***********************************************************/

/* Servo */
void servo_setup()
{
  servo.attach(SERVO_PIN);  // Attach the servo to the defined pin
}

/* Vibration coil */
void vibration_coil_setup()
{
  driver.attachMotorA(VIBRATION_COIL_INPUT_A1, VIBRATION_COIL_INPUT_A2);  // Attach VCA to motor A of DRV8833
  driver.motorAStop();  // Ensure motor is stopped initially

  pinMode(MOTOR_SLEEP, OUTPUT);
  digitalWrite(MOTOR_SLEEP, HIGH);  // Enable the motor driver (DRV8833)

  haptic_timeBetweenUpdates = 1.0f / haptic_updateFrequency;
}

/* Loadcell */
void loadcell_setup()
{
    scale.begin(LOADCELL_DATA, LOADCELL_CLOCK);
    scale.set_scale(loadcell_calibration_scale);
}

/* Thumb controls */
void thumb_controls_setup()
{
    // thumbstick 
    pinMode(THUMBSTICK_X, INPUT);
    pinMode(THUMBSTICK_Y, INPUT);

    // push button 
    pinMode(PUSHBUTTON, INPUT_PULLUP);

    // light sensor
    pinMode(LIGHT_SENSOR, INPUT);
}

/* HARDWARE HANDLING FUNCITONS ************************************************/

/* Servo */
void updateServo()
{
    servo.write(setAngle);
}

/* Vibration coil */
void playHapticPattern()
{   
    // how long has it been since we've last triggered the coil?
    haptic_timeSinceLastUpdate += (micros() - haptic_timeSinceLastUpdate) / 1000000.0f;
    
    if (haptic_timeSinceLastUpdate >= haptic_timeBetweenUpdates)
    {
        if (haptic_amp > 0) // if the amplitude is non-zero
        {
            if (haptic_pattern[haptic_patternIndex] >= 0) 
            {
                driver.motorAForward(haptic_pattern[haptic_patternIndex]);  // Forward direction vibration
            } else {
                driver.motorAReverse(-haptic_pattern[haptic_patternIndex]);  // Reverse direction vibration
            }
        } else {
            driver.motorAStop();  // Stop vibration if amplitude is 0
        }

    haptic_patternIndex += 1;
    
    if (haptic_patternIndex >= HAPTIC_PATTERN_LEN) {
      haptic_patternIndex = 0;  // Loop the pattern
    }

    haptic_timeSinceLastUpdate = 0.0f;
  }
}

/* Loadcell */
void updateForceReading()
{
    // the loadcell is too slow to be read at every loop, we sample it at 88 Hz
    loadcell_timeSinceLastUpdate += (micros() - loadcell_timeSinceLastUpdate) / 1000000.0f;
    
    if(loadcell_timeSinceLastUpdate >= LOADCELL_TIME_BETWEEN_SAMPLES)
    {
        if (scale.is_ready()) 
        {
            rawForce = scale.get_units();     // read load cell 
            filtForceNew = 0.75 * filtForceLast + 0.25 * rawForce; // blend the raw reading with previous ones
            currentForce = filtForceNew;
            dErrorForce = (filtForceNew - filtForceLast) / LOADCELL_TIME_BETWEEN_SAMPLES;  // dF/dt
            filtForceLast = filtForceNew;
        }
    }
}

/* Thumb controls */
void updateThumbControls()
{
    thumbstick_position[0] = analogRead(THUMBSTICK_X);
    thumbstick_position[1] = analogRead(THUMBSTICK_Y);
    pushbutton_press = digitalRead(PUSHBUTTON);
    light_sensor_value = analogRead(LIGHT_SENSOR);
}


/* COMMUNICATION HANDLING FUNCTIONS *******************************************/

void processSerialCommands()
{   
    /* check if a new command is available
     A command is always a letter followed by a value (number or letter) ex:A120 (angle, 120 deg)
     Available commands:
     - Axxx : define desired servo angle [SERVO_ANGLE_MIN to SERVO_ANGLE_MAX]
     - Oxxx : set angle at which finger collides with object [SERVO_ANGLE_MIN to SERVO_ANGLE_MAX]
     - Sxxx : set stiffness [positive number]
     - Pxxx : set haptic pattern amplitude [ HAPTIC_AMPLITUDE_MIN to HAPTIC_AMPLITUDE_MAX]
     - Fxxx : set haptic pattern frequency [ HAPTIC_FREQUENCY_MIN to HAPTIC_FREQUENCY_MAX]
     - Mxxx : set controller mode  (expects a letter :  P, T, D)
    */
    while (Serial.available())
    {
        String st = Serial.readString();
        char command = st.charAt(0);
        String val = st.substring(1, st.length());

        switch (command)
        {
            case 'A':
                setAngle = getAngle(val);
                #if DEBUG
                    Serial.println("Recieved command A");
                #endif 
                break;
            case 'O':
                objectAngle = getAngle(val);
                #if DEBUG
                    Serial.println("Recieved command O");
                #endif 

                break;
            case  'S':
                setPIDstiffness(val);
                #if DEBUG
                    Serial.println("Recieved command S");
                #endif 
                break;
            case  'P':
                createHapticPattern(val);
                #if DEBUG
                    Serial.println("Recieved command P");
                #endif 
                break;
            case  'F':
                setHapticFrequency(val);
                #if DEBUG
                    Serial.println("Recieved command F");
                #endif
                break;
            case  'M':
                setOperatingMode(val);
                #if DEBUG
                    Serial.println("Recieved command M");
                #endif
                break;
            default:
                break;
        }
    }
}

void sendCurrentState()
{
    Serial.print(setAngle);
    Serial.print(" ");
    Serial.print(light_sensor_value, DEC);
    Serial.print(" ");
    Serial.print(filtForceNew, 2);
    Serial.print(" ");
    Serial.print(stiffness * (100.0f * PID_STIFFNESS_COEFFICIENT), 2);
    Serial.print(" ");
    Serial.print(thumbstick_position[1], DEC); //vertical
    #if DEBUG
    Serial.print(rawForce);
    Serial.print(" ");
    Serial.print(errorForce);
    Serial.print(" ");
    Serial.print(desiredForce);
    Serial.print(" ");
    Serial.print(dAngleFloat);
    Serial.print(" ");
    #endif
    Serial.flush();
}

void setOperatingMode(String val)
{
  char md = val.charAt(0);
  switch (md) {
    case 'P': 
        op_mode = PINCHING_MODE; 
        #if DEBUG
            Serial.println("PINCHING_MODE");
        #endif
        break;
    case 'T': 
        op_mode = TOUCHING_MODE; 
        #if DEBUG
            Serial.println("TOUCHING_MODE");
        #endif
        break;
    case 'D': 
        op_mode = DISABLED_MODE;
        #if DEBUG
            Serial.println("DISABLED_MODE");
        #endif 
        break;
    default:
        op_mode = DISABLED_MODE;
        #if DEBUG
            Serial.println("DISABLED_MODE");
        #endif 
        break;
  }
}

double getAngle(String val)
{
    double angle = val.toFloat();  // Convert the string value to a float

    // Ensure the angle is within the valid range
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    } else if (angle < SERVO_ANGLE_MIN) {
        angle = SERVO_ANGLE_MIN;
    }

  return angle;
}

void createHapticPattern(String val)
{
    haptic_amp = val.toInt();  // Convert the string value to an integer

    // Ensure the amplitude is within the valid range (0 - 500)
    if (haptic_amp > HAPTIC_AMPLITUDE_MAX) {
        haptic_amp = HAPTIC_AMPLITUDE_MAX;
    } else if (haptic_amp < HAPTIC_AMPLITUDE_MIN) {
        haptic_amp = HAPTIC_AMPLITUDE_MIN;
    }

    // Generate a random vibration pattern based on the amplitude
    for (int i = 0; i < HAPTIC_PATTERN_LEN; ++i) {
        haptic_pattern[i] = random(-haptic_amp, haptic_amp);
    }
}

void setHapticFrequency(String val)
{
    haptic_updateFrequency = val.toFloat();  // Convert the string value to a float

    // Ensure the frequency is within the valid range (1 - 15000 Hz)
    if (haptic_updateFrequency > HAPTIC_FREQUENCY_MAX) {
        haptic_updateFrequency = HAPTIC_FREQUENCY_MAX;
    } else if (haptic_updateFrequency < HAPTIC_FREQUENCY_MIN) {
        haptic_updateFrequency = HAPTIC_FREQUENCY_MIN;
    }

    haptic_timeBetweenUpdates = 1.0f / haptic_updateFrequency;  // Recalculate time between updates
}

void setPIDstiffness(String val)
{
    stiffness = (val.toFloat()) / (100.0f * PID_STIFFNESS_COEFFICIENT);
    
    if (stiffness < PID_STIFFNESS_MIN) {
        stiffness = 0.01f;
    }
}

/* PID CONTROL FUNCTIONS ******************************************************/
void updateDefaultDesiredForce()
{
    if (mode == PINCHING_MODE || mode == DISABLED_MODE) 
    {
        if (filtForceNew < 1.8) {
            defaultDesiredForce = 2.0f;
        } else {
             defaultDesiredForce = 0.4f;
        }
    }
    /*
    if (mode == TOUCHING_MODE)
    {
        defaultDesiredForce = -0.5f;
    }
    */
}

void mainPIDloop()
{
    /* update time tracking */
    pid_timeSinceLastUpdate += (micros() - pid_timeSinceLastUpdate) / 1000000.0f;

    if(pid_timeSinceLastUpdate >= PID_TIME_BETWEEN_UPDATES)
    {
        //
    // If there is an object - generate a resistive spring force, according to the penetration distance.
    // Current angle is setAngle. 
    // Current object surface is at object_angle.

        if (setAngle >= object_angle) {
            desiredForce = defaultDesiredForce;
        } else {
            float penetration = object_angle - setAngle;
            desiredForce = penetration * stiffness;
        }

        deltaForce = dErrorForce * pid_timeSinceLastUpdate;
        currentForce = currentForce + deltaForce;

        errorForce = desiredForce - currentForce;
        dAngleFloat = Kp * errorForce - Kd * dErrorForce; // angle change amount based on force PD loop.
                                                          // Kd term is negative since desired dF/dt is always zero.
        
        /* if the measured force is very close to the desired value, assume we're at the right angle*/
        if (abs(errorForce) < 0.008) {
            dAngleFloat = 0;
        }

        /* Update target Angle : basic update*/
        previousAngle = setAngle;
        dAngle = dAngleFloat;
        setAngle = setAngle + dAngle;

        /* Now we decide how to further update it based on the object and mode */

        /*If stiffness command was bigger than '1000' - then it is a rigid object.*/
        if (stiffness >= (10 / PID_STIFFNESS_COEFFICIENT)) 
        {
            if (setAngle < object_angle) {
                setAngle = object_angle;  //minimum_angle;
            }
        }
        
        if (mode == PINCHING_MODE) {
            if (dAngle > speedLimitPinch && previousAngle < object_angle) {
                setAngle = previousAngle + speedLimitPinch;
                
                if (setAngle > object_angle) {
                    setAngle = object_angle;
                }    
            }
        }

        if (mode == TOUCHING_MODE) {
            setAngle = object_angle;  // Touching mode is position controlled only!
        }

        /* update servo position */
        updateServo();

        /* reset time counter */
        pid_timeSinceLastUpdate = 0.0f;
    }

}
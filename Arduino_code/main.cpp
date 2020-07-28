#include <Arduino.h>
#include "sensormodule.h"
#include "MotorControl.h"
#include "PID_v1.h"


/*
Strategy and design.
Board dimensions - 1m x 0.5m
Incline angle of the board - unknown
Sensors and actuators used:
1. 8 ultrasonic distance sensors to measure distance
2. Laser transmitter and receiver module to enforce size requirement of marble
3. Servo motor to control the position of the catcher bin.
4. IR sensor on catcher to sense presence of ball

Assumptions and boundary conditions
1. Marble is released only from the top.
2. Only one marble is tracked at a time.
3. Marble rolls down the plank and does NOT bounce around.
4. Plank is NOT too rough or has imperfections that affect the trajectory of the marble significantly
5. Setup is NOT located in an environment with ambient high frequency noise. (roughly 40KHz in our case)
6. Experiment is conducted under NTP conditions. Distance measurements using the Ultrasonic sensor 
are dependent on the velocity of sound in air and therefore susceptible to temperature and pressure variations.

Naming conventions:
Ultrasonic sensor : US
Laser sensor : LS
Servo motor : motor

State machine definition
--------------------------------------------------------------------------------------------------------------
State 1 - Idle state
State 2 - Sieve
State 2 - Tracking

State functions
---------------------------------------------------------------------------------------------------------------
State 1 - Idle state
    Default state when powered on.
    Timer2 interrupt every 6ms to poll US1.
    If output from US1 detected
        1. marble_detected=1
        2. update position of marble
        3. State transition to state2
        4. Call LS_setup()

State 2 - Sieve state
    If Laser trapwire is set off
        reset sensor state to state1
    else If MUX output is detected
        Immediately transition to State3
        call LS_stop routine.

State 3 - Marble Tracking state 
    Timer2 interrupt every 6ms to poll active US
    If output from US (MUX) is detected
        1. measure distance and update position
        2. increment US
    If Timer1 global Timeout
        1. reset sensor state to state1

    If catcher_sensor interrupt triggered
        1. Update Happy, ;)
        2. Reset sensor state to state1

Psuedo-code
---------------------------------------------------------------------------------------------------------------
setup
{
    call PIN_setup to initialize sensor Ports
    initialize PID control
    initizlie motorcontrol
}

loop
{
    If MUX_output_detected 
        call MUX_output_trigger
        call Timer1_reset to reset global timeout.
        switch(marble_detected)
        case 2 (State2)
            call increment_US routine
        case 1 (State2)
            call LS_stop routine
            call increment_US routine
            state transition to State3
        case 0 (State1)
            marble_detected=1
            call LS_setup routine
            call increment_US routine
            call Timer1_Timeout_setup routine to start global timeout
            State transition to state2
    
    if (catcher_success) or (Sensor_Timeout) or (marble_size_non_conforming)
        call reset_sensor routine
        state transition to State1

    if Trigger_US
        call Trigger_to_US routine
        Trigger_US=0

    >>>>> This is executed in every loop <<<<
    call Calculate_catcher_position_mm  
    PID Compute
    Motorcontrol speedwrite() 
    >>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<

}

====================================================================================================================
*/

long PID_Output_PWM=0;
#define PIN_HBridge_1 8
#define PIN_HBridge_2 9
#define PIN_output_PWM 5
//It is essential that the PWM output be on ports 5 or 6.
// This is to ensure that the PWM output is associated with Timer0.
//Since Timer1 and Timer2 are in use by sensorsmodule, it is not clear if PWM output on the other ports are compatible. 


byte Kp=3, Ki=5, Kd=1;
bool HBridge_direction; //Boolean flag to store direction for H-bridge; 1 for Forward and 0 for backward.

/*
The input, output and set-point for PID are variables that need to be passed as pointers.
In our case, the Input to PID is the current position of the catcher. 
A rotation encoder attached to the motor enables us to measure the current position of the catcher
The output of PID is the PWM value that will be sent to PWM port of H-bridge.
Setpoint is the predicted position of the marble set by sensors module.
*/
//Catcher_position_mm and pos_prediction_marble_mm are defined and updated in sensor

sensor sensor1;
#define PID_Input_distance_mm sensor1.Catcher_position_mm
#define PID_setpoint_distance_mm sensor1.pos_prediction_marble_mm
PID PID_ctrl(&PID_Input_distance_mm, &PID_setpoint_distance_mm, &PID_Output_PWM, Kp, Ki, Kd, DIRECT); //Initializing PID
MotorControl movemotor(PIN_HBridge_1,PIN_HBridge_2,PID_Output_PWM); // Initializing motorcontrol


// ISR definitions
//-------------------------------------------------------------------------------------------------------------------
// ISR to detect output echo pulse from ultrasonic sensors.
// MUX output as defined in sensormodule.h is on port 19 of Arduino. Therefore, the ISR uses the PCINT1_vect interrupt vector.
// If MUX output is connected to D0 to D7 (replace with PCINT2_vect) or D8 - D13 (replace with PCINT0_vect).


ISR(INT0_vect)
{
    if (!(sensor1.Rot_encB))
    {
        sensor1.Rot_encA=1;
    }
    sensor1.Rot_encB=0;
    //As long as Rot_encB was not active, we set Rot_encA to 1.
    //If Rot_encB is active, then port B was active before A, which has been dealt with.
}

ISR(INT1_vect)
{
    if (sensor1.Rot_encA) 
    {
        sensor1.Catcher_distance_increment_mm+=1;
    }
    //If Rot_encA was active first, then it is a clockwise rotation.
    else 
    {
        sensor1.Catcher_distance_increment_mm-=1;
        sensor1.Rot_encB=1;
        //Since PortA was not active, we set a flag to ensure that Rot_encA is not set to 1 again.
    }
    sensor1.Rot_encA=0;
}

ISR(PCINT1_vect)
{
    sensor1.MUX_output_detected = (!sensor1.MUX_output_detected);
    //Flag is set True at the rising edge and reverts back to False at the falling edge
}

// ISR to determine presence of marble in catcher
// Catcher_sensor has been connected to Ports 12 and 13 which are associated with the PCINT0_vect interrupts.
ISR(PCINT0_vect)
{
    sensor1.catcher_success=1;
    sensor1.success_counter++; //update success counter
}


ISR(PCINT2_vect)
{
    sensor1.reset_sensor=1;
}

//ISR to trigger US every 6ms.
ISR(TIMER2_COMPA_vect)
{
    sensor1.Trigger_US=1;
}

//ISR to trigger a global Timeout every 200ms, which is the maximum time a marble can take to cross one sensor.
ISR(TIMER1_COMPA_vect)
{
    sensor1.reset_sensor=1;
    sensor1.misses_counter++; //update misses counter
}

//-------------------------------------------------------------------------------------------------------------------


enum FSM
{
    Idle,
    Sieve,
    Tracking
}state;


void setup()
{
    sensor1.PIN_setup(); //sensors module routine to initialize ports.
    PID_ctrl.SetMode(1); //Set PID to Auto mode
    movemotor.SpeedWrite(1,0); //Initize motor in idle state.
}

void loop()
{
    if (sensor1.MUX_output_detected) //Rising edge detected
    {
        uint8_t Timeout_check = sensor1.MUX_output_trigger(); //Loops till the falling edge of MUX output is detected or a Timeout is detected whichever happens earlier
        if (!Timeout_check) //If a timeout was triggered while measuring echo pulse width
        {
            sensor1.reset_all_sensors();
            state=Idle;
        }
        else
        {
            switch (state)
            {
                case 0:
                    sensor1.marble_detected=1;
                    sensor1.LS_setup();
                    sensor1.increment_US();
                    sensor1.Timer1_Timeout_setup(Timeout_ms);
                    state=Sieve;
                    break;
                
                case 1:
                    sensor1.LS_stop();
                    sensor1.increment_US();
                    state=Tracking;
                    break;

                case 2:
                    sensor1.increment_US();
                    break;
            }
        }
    }

    if (sensor1.catcher_success || sensor1.reset_sensor)
    {
        sensor1.reset_all_sensors();
        state=Idle;
    }

    else if (sensor1.Trigger_US)
    {
        sensor1.Trigger_to_US(US_trigger_interval_us);
        sensor1.Trigger_US=0;
    }
  
    HBridge_direction = sensor1.Calculate_catcher_position(); //Routine retuns a boolean True if Catcher is left of SetPoint and False otherwise
    //Calculate_catcher_position internally updates the input to PID
    if (PID_ctrl.Compute())
        movemotor.SpeedWrite(HBridge_direction,PID_Output_PWM);

    //Compute routine of PID class returs a boolean. True if output value has changed and False otherwise.
    //If output value has changed, we write the 
    
}






///===============================================================================================

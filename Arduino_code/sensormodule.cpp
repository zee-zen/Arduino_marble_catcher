#include "sensormodule.h"

sensor::sensor()
{
    marble_detected=0;
    Trigger_US=0;
    MUX_output_detected=0;
    LS_flag=0;
    Rot_encA=0;
    Rot_encB=0;
    catcher_success=0;
    reset_sensor=0;

    marble_direction_motion=0; //0 for Left-to-right; 1 for right-to-Left ; 
    SR2_serial_input=0;
    SR1_serial_input=1;
    success_counter=0;
    misses_counter=0;
    current_position_marble_mm=0;
    Catcher_position_mm=0;
    pos_prediction_marble_mm=0; //These two variables are passed as Input and setpoint for PID, therefore are defined as long
    Catcher_distance_increment_mm=0;
}

void sensor::PIN_setup()
{
    //This routine should be called from setup() function on main to initialize all the corresponding ports.
    //The corresponding port numbers should be updated on sensormodule.h file.
    pinMode(PIN_SR1_IN,OUTPUT);
    pinMode(PIN_SR1_shift_clck,OUTPUT);
    pinMode(PIN_SR1_SR2_latch_clck,OUTPUT);
    pinMode(PIN_SR2_IN,OUTPUT);
    pinMode(PIN_SR2_shift_clck,OUTPUT);
    pinMode(PIN_MUX_output,INPUT_PULLUP);
    pinMode(PIN_catcher_sensor_output,OUTPUT);
    pinMode(PIN_catcher_sensor_input,INPUT);
    pinMode(PIN_Rot_enc_PORTA,INPUT_PULLUP); 
    pinMode(PIN_Rot_enc_PORTB,INPUT_PULLUP);
    pinMode(PIN_LS_in, OUTPUT);
    pinMode(PIN_LS_out, INPUT);
    
    this->PCint_attach_interrupt(PIN_MUX_output); //Attaching a PinChange interrupt to output from MUX
    this->PCint_attach_interrupt(PIN_catcher_sensor_output); //Attaching a PinChange interrupt to output from catcher sensor

    this->EXINT_attach_Rot_enc();
    //External interrupt to output from Rotation encoder outputs.
    //To detect direction and update total distance increment

    this->Timer2_interrupt_setup(US_polling_interval_ms); //Timer interrupt to poll US
    this->Input_SR2(SR2_serial_input);
    //Initialize MUX output
}

// Attach Pin-Change interrupt on pin (input)
void sensor::PCint_attach_interrupt(uint8_t pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); //enable pin in corresponding PCMSKx (where x is 0, 1 or 2)
    PCIFR |= bit(digitalPinToPCICRbit(pin)); // Clear Flag register of any outstanding interrupts
    PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt of corresponding PCINT on the control register

    // Code adapted from https://playground.arduino.cc/Main/PinChangeInterrupt/.
}

void sensor::EXINT_attach_Rot_enc()
{
    EICRA |= (1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00);
    //Detect rising edge on both INT0 and INT1
    EIMSK |= (1<<INT0) | (1<<INT1);
    //Enable INT0 and INT1
}

// Dettach Pin-change interrupt on Pin (input)
void sensor::PCint_dettach_interrupt(uint8_t pin)
{
    *digitalPinToPCMSK(pin) = 0;
    PCIFR &= ~(bit(digitalPinToPCICRbit(pin)));
    PCICR &= ~(bit(digitalPinToPCICRbit(pin)));
}


void sensor::LS_setup()
{
    digitalWrite(PIN_LS_in, HIGH);
    digitalWrite(PIN_LS_out, LOW);
    this->PCint_attach_interrupt(PIN_LS_out); // Attaching a PinChange interrupt to the Laser trapwire output.
    //When there is a change in the output, an interrupt is triggered. This essentially means that the marble is size non-conformant.

}

void sensor::LS_stop()
{
    this->PCint_dettach_interrupt(PIN_LS_out);
    digitalWrite(PIN_LS_in,LOW);
    digitalWrite(PIN_LS_out,LOW);
}

//Routine to be called if a change in polling period is to be executed.
void sensor::Timer2_interrupt_setup(int period_ms)
{
    TCCR2A = 0; 
    TCCR2B = 0;
    TCNT2 = 0; 
    OCR2A = ((250*period_ms)>>4); // Assuming a prescaler of 1024 and an internal clock freq of 16MHz
    TCCR2A |= (1<<WGM21); // enable CTC on TIMER2
    TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20); // Set prescaler of 1024
    TIMSK2 |= (1<<OCIE2A);

    // Use TIMER2_COMPA_vect for ISR
    // Code adapted from https://www.instructables.com/id/Arduino-Timer-Interrupts/
}

//Routine to be called when system goes to shutdown.
void sensor::Timer2_interrupt_stop()
{
    TIMSK2 &= ~(1<<OCIE2A);
}

//Routine to be called whenever state of the system changes from Idle state.
void sensor::Timer1_Timeout_setup(long period_ms)
{
    //The timeout period is large (roughly 70ms).
    TCCR1A = 0; 
    TCCR1B = 0;
    TCNT1 = 0; 
    OCR1A = ((250*period_ms)>>4); // Assuming a prescaler of 1024 and an internal clock freq of 16MHz
    TCCR1B |= (1<<WGM12); // enable CTC mode on Timer1
    TCCR1B |= (1<<CS12) | (1<<CS10); //Set prescaler of 1024
    TIMSK1 |= (1<<OCIE1A);

    //Use TIMER1_COMPA_vect for ISR
}

//Routine to stop global Timeout. This should be executed whenever systems reverts back to Idle state.
void sensor::Timer1_Timeout_stop()
{
    TIMSK1 &= ~(1<<OCIE1A);
}

//Routine to reset global Timeout counter. Would be called everytime echo_timer_US routine is executed.
void sensor::Timer1_reset()
{
    TIMSK1 &= ~(1<<OCIE1A); // Stop TIMER1
    TCNT1 = 0; // Reset Timer-Counter register
    TIMSK1 |= (1<<OCIE1A); // Restart TIMER1
}

//Routine that will trigger the active US. Would be called everytime Trigger_to_US is called.
void sensor::Input_SR1(uint8_t SR1_input)
{
    digitalWrite(PIN_SR1_SR2_latch_clck,LOW); // Latch clock is set low enable writing to the SR
    shiftOut(PIN_SR1_IN,PIN_SR1_shift_clck,SR1_bitorder,SR1_input); // Serial communication to SR1
    digitalWrite(PIN_SR1_SR2_latch_clck, HIGH); //Transfer SR bits to Latch OUTPUT
    digitalWrite(PIN_SR1_SR2_latch_clck, LOW);
}


//Routine that will increment MUX output. Would be called everytime increment_US routine is called.
void sensor::Input_SR2(uint8_t SR2_intput)
{
    digitalWrite(PIN_SR1_SR2_latch_clck,LOW); // Latch clock is set low enable writing to the SR
    shiftOut(PIN_SR2_IN,PIN_SR2_shift_clck,SR2_bitorder,SR2_serial_input); // Serial communication to SR2
    digitalWrite(PIN_SR1_SR2_latch_clck, HIGH); //Transfer SR bits to Latch OUTPUT
}

//Routine that will send the trigger pulse to the active US and connect it's output to MUX output.
void sensor::Trigger_to_US(uint8_t pulse_us)
{
    Input_SR1(SR1_serial_input); //Setting input to SR1
    delayMicroseconds(pulse_us); // delay of at least 10us to trigger the corresponding US
    shiftOut(PIN_SR1_IN,PIN_SR1_shift_clck,SR1_bitorder,0); //Writing a 0 to SR1 to reflect a low on all states
    digitalWrite(PIN_SR1_SR2_latch_clck,HIGH); // Transfer SR bits to latch OUTPUT.
    //Now wait for PCINT on MUX ouput 
}

//Routine to change to the next US.
//Routine to be run immediately after echo_timer_US();
void sensor::increment_US()
{
    SR1_serial_input = (SR1_serial_input << 1); // Left-shift operation to increment input of SR1
    SR2_serial_input++; //increment
    this->Input_SR2(SR2_serial_input); //Trigger MUX to increment output.
}

//Routine to reset state to idle.
//Routine to be called if reset_sensor=1.
void sensor::reset_all_sensors()
{
    SR1_serial_input = 1;
    SR2_serial_input = 0;
    this->Timer1_Timeout_stop();

    //Setting all flags to zero.
    Trigger_US=0;
    MUX_output_detected=0;
    LS_flag=0;
    reset_sensor=0;
    catcher_success=0;
    
    pos_prediction_marble_mm=0; //Default SetPoint for catcher
}


//Routine to calculate width of the echo output. Routine should be triggered with flag MUX_output_detected.
//Time is of the essence. Therefore, this routine should be given the first priority.

unsigned int sensor::echo_timer_US()
{
    unsigned long curr_time = micros();
    while((MUX_output_detected) && !(reset_sensor)){} 
    // Loop to wait till the falling edge of MUX output is detected or a Timeout, whichever happens first.

    if (!reset_sensor)
    {
        unsigned long final_time = micros();        
        Timer1_reset(); //Reset Timer1 which houses a global Timeout counter.
        return (unsigned int)(final_time - curr_time);
    }
    else return NoECHO;
    // The output is type cast to an unsigned int since anything above 6ms is either an error or timeout.
}

//This routine should be run in succession everytime echo_timer_US is executed.
//Input to this routine should arrive from echo_timer_US.
void sensor::calculate_distance_mm_US(unsigned int delay_us) 
{
    int measured_position_marble_mm  = ((delay_us*180)>>10);
    // approximated the factor of 17/1000 as 18/1024 to simplify operation with ints only
    // Error in estimation (over-estimation) is roughly 3.4%.
    // Addtionally echo_timer_US would typically underestimate the echo pulse width. Therefore, this should be tolerable.
    if (current_position_marble_mm)
    {
        int delta_Y = (measured_position_marble_mm - current_position_marble_mm - 250);
        //unsigned int slope = delta_Y/seperation_distance_US_mm
        unsigned int delta_Y_pred = (1024 - ((SR2_serial_input+1)<<7))*((unsigned int)abs(delta_Y) >> 7);
        if (delta_Y>=0) 
        {
            marble_direction_motion = 1; //Direction Right-to-Left
            pos_prediction_marble_mm = (long) (current_position_marble_mm + delta_Y_pred);
        }
        else
        {
            marble_direction_motion=0; // Direction Left-to-Right
            pos_prediction_marble_mm = (long) (current_position_marble_mm - delta_Y_pred);
        } 
        //Approximate value for final position of marble calculated, ignoring accelration due to gravity.
        //pos_prediction_marble_mm is defined as a long to be used in PID module. Therefore it is type cast as a long
        current_position_marble_mm += delta_Y; // Updating current marble position
        this->check_pos_prediction();
    }

    else
    {
        current_position_marble_mm = measured_position_marble_mm - 256;
        //pos_prediction_marble_mm = current_position_marble_mm;
    }
   
}


void sensor::check_pos_prediction() //Internal check on the predicted value of final position. Run inside calculate_distance_mm_US
{
    if (pos_prediction_marble_mm>250) pos_prediction_marble_mm = 250;
    else if (pos_prediction_marble_mm < -250) pos_prediction_marble_mm=-250;
}
//If the current predicted position for the marble is outside of the plank, the SetPoint for PID is set to the edge of the plank.

//Routine that is called from main every time an output on MUX output is detected.
//Measures the pulse width and calculates the distance.
uint8_t sensor::MUX_output_trigger()
{
    unsigned int delta=this->echo_timer_US();
    if (delta) this->calculate_distance_mm_US(delta);
    return (uint8_t) delta;
    //Output is typecast to uint8_t since it is only for checking timeout error.
}



//This routine should be run in every loop regardless to update position of the Catcher.
//Distance_per_pulse is a float. If the multiplication operation takes a lot of time, then it can be rounded off at the cost of accuracy.

bool sensor::Calculate_catcher_position()
{
    Catcher_position_mm += (long) (Distance_per_pulse * Catcher_distance_increment_mm);
    Catcher_distance_increment_mm=0; //Set increment to zero, so that increments don't get accumulated.
    return ((int) Catcher_position_mm > pos_prediction_marble_mm);
    //Return a boolean value 1 if Catcher is left of marble_predicted_position, which would require a clockwise rotation from motor
    //If return value is 0, then an anti-clockwise rotation is required.

    //Catcher_position_mm += (int) round(Distance_per_pulse)*Catcher_distance_increment_mm;
}

#ifndef sensormodule_H
#define sensormodule_H

	#if defined(ARDUINO) && ARDUINO >= 100
		#include <Arduino.h>
	#else
		#include <WProgram.h>
		#include <pins_arduino.h>
	#endif

	#include <avr/io.h>
	#include <avr/interrupt.h>

	/*
	Overview
	-------------------------------------------------------------------------------------------------------------
	The setup is configured to have 8 ultrasonic sensors that are lined up one side of the plank
	with a equal seperation distance.
	They are interfaced using an 8 bit Shift-register (SR1).
	The 8 output ports from the Shift-register are connected to the Trigger pins of the 8 ultrasonic sensor.
	The echo output pins of the Ultrasonic sensors are connected to a 8-bit MUX with 3 control pins.
	The three control pins of the MUX are connected to the first three output ports of another 8-bit Shift-Register (SR2)
	Input, Serial clock and Register output clock pins of SR1 and SR2 are connected to DIO ports of Arduino.
	Additionally, a Laser sensor to detect balls bigger than 1cm is connected to pin3 and a sensor attached to the catcher is 
	connected to Pin2 of Arduino.
	------------------------------------------------------------------------------------------------------------------

	Working
	-------------------------------------------------------------------------------------------------------------------
	Only one Ultrasonic sensor (US) is active at any given time.
	The choice of sensor is controlled by the Shift-register SR1.
	Initially, the output register of SR1 is configured to read (00000001), such that only US1 is active.
	If an output from the active US is detected, a shift operation on the input to SR1 is performed triggering the next US.
	The input to MUX is controlled by SR2 whose input is designed such that echo output from the active US is directed to MUX output.

	Output-enable pins of both SR1 and SR2 are connected to ground. (i.e the output is always on)
	Reset pins of both SR1 and SR2 are connected to Vcc (Active-low).
	Trigger to the US is provided by output of SR1 which will be briefly written to trigger the corresponding US.
	After a delay of 10us, SR1 is written again to refelect a low on all outputs.

	The output pin of MUX is configured to a Pin-Change interrupt.
	Therefore, if the echo output of the active US goes high, a trigger is generated.
	Similarly, when the echo output of the active US goes low, a trigger is generated.
	Timing the length of this output, we can calculate the distance of the marble from the US.
	-------------------------------------------------------------------------------------------------------------------

	Conventions on spatial position

	Positions are in mm and encoded as int. 
	0 defines the center of the plank.
	a positive value denotes a position on the right of center (as seen from lower side of the plank)
	a negative value denotes a position to the left of center.
	*/


	#define PIN_LS_in 4
	#define PIN_LS_out 7
	//Pins 5 and 6 have been intentionally skipped to allow for PWM output associated with Timer0.
	#define PIN_catcher_sensor_output 13
	#define PIN_catcher_sensor_input 12
	#define PIN_SR1_IN 14
	#define PIN_SR1_shift_clck 15
	#define PIN_SR1_SR2_latch_clck 16
	#define SR1_bitorder LSBFIRST
	#define SR2_bitorder LSBFIRST
	#define PIN_SR2_IN 17   
	#define PIN_SR2_shift_clck 18
	#define PIN_MUX_output 19
	#define PIN_Rot_enc_PORTA 2
	#define PIN_Rot_enc_PORTB 3
	//Ensure that PORTA and PORTB of the rotation encoder are connected to the two External interupt pins.
	//In the case of Arduino UNO, they are pins 2 and 3.
	//Here the definition of PORTA and PORTB is such that a clockwise rotation is when PORTA is triggered before PORTB.
	//And an anti-clockwise rotation is when PORTB is triggered before PORTA.
	//Care must be taken to consider the orientation of the encoder as well, since a clockwise rotation seen from the back side is counter-clockwise.


	#define Max_distance_cm_US 50 //Maximum distance to check for response
	#define US_polling_interval_ms 6
	#define US_echo_time_roundtrip_cm 59
	#define US_trigger_interval_us 10 
	#define NoECHO 0
	#define Timeout_ms 67
	#define seperation_distance_US_mm 125
	#define Motor_encoder_pulses_per_rotation  168 //Built-in encoder as part of DC motor RMCS-2299
	#define Distance_per_pulse 1.122
	//This translates to a distance of about 1.122mm per pulse in our case.
	//The rack and pinion have a gear ratio of 1 and Diameter of pinion is 60mm.

class sensor
{
	public:
	// Flag definitions that detect the status at various stages
	volatile bool marble_detected;
	volatile bool Trigger_US;
	volatile bool MUX_output_detected;
	volatile bool LS_flag;
	volatile bool Rot_encA;
	volatile bool Rot_encB;
	volatile bool catcher_success;
	//volatile bool Sensor_Timeout=0;
	//volatile bool marble_size_non_conforming=0;
	volatile bool reset_sensor;

	long Catcher_position_mm;
	long pos_prediction_marble_mm; 
	//These two variables are passed as Input and setpoint for PID, therefore are defined as long
	
	int Catcher_distance_increment_mm;
	uint8_t success_counter;
	uint8_t misses_counter;

	//Functions directly called from main
//-----------------------------------------------------------------------------------------------------------------
	
	//Setup routines
	sensor();
	void PIN_setup();
	void LS_setup();
	void LS_stop();
	void Timer1_Timeout_setup(long period_ms);
	void Timer1_Timeout_stop();

	void Trigger_to_US(uint8_t pulse_us); //Routine to trigger active US with value passed in microseconds
	void increment_US(); // Increment Shift-Registers outputs
	void reset_all_sensors(); // Reset sensor configuration
	uint8_t MUX_output_trigger(); //
	bool Calculate_catcher_position();

private:

	//variables internal to sensormodule.
	uint8_t marble_direction_motion; //0 for Left-to-right; 1 for right-to-Left ; 
	uint8_t SR2_serial_input;
	uint8_t SR1_serial_input;
	int current_position_marble_mm;
	//Functions internally called
	//------------------------------------------------------------------------------------------------------------------
	
	void PCint_attach_interrupt(uint8_t pin);
	void PCint_dettach_interrupt(uint8_t pin);
	void Timer2_interrupt_setup(int period_ms);
	void Timer2_interrupt_stop(); //Not used in this implementation. But necessary for further scope.
	void Timer1_reset(); // Reset global timer.
	void EXINT_attach_Rot_enc();//Function to attach external interrup to ports 2 and 3 for the rotation encoder.
	void calculate_distance_mm_US(unsigned int delay_us); //Function to calculate current position of marble and to update the predicted position of marble.
	unsigned int echo_timer_US(); // Determine the length of the echo output.
	void check_pos_prediction(); //Internal consistency check
	void Input_SR1(uint8_t SR1_input); // Serial communication to SR1 for controlling ultrasonic sensors
	void Input_SR2(uint8_t SR2_input); // Serial communication to SR2 for controlling MUX output
};

#endif

/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010

 In this experiment ADC captures the analog sensor values and displays it on the LCD

 Concepts covered:  ADC, LCD interfacing

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 LCD Display interpretation:
 ****************************************************************************
 *BATTERY VOLTAGE	IR PROX.SENSOR 2	IR PROX.SENSOR 3	IR.PROX.SENSOR 4*
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		FRONT SHARP DIS *
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600s
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Make sure that you copy the lcd.c file in your folder

 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"
#include "servo.c"
#include "predef.c"

unsigned int value,counta;
float BATT_Voltage, BATT_V;
int left_speed=160,right_speed=206;
int turn_left=95,turn_right=160;
/*
*
* Function Name: motion_pin_config
* input: no input
* output: no return
* logic: Function to configure ports to enable robot's motion
* Example call: motion_pin_config();
*
*/
void motion_pin_config (void)
{
	DDRD = DDRD | 0x0F;
	PORTD = PORTD & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
*
* Function Name: lcd_port_config
* input: no input
* output: no return
* logic: Function to configure LCD port
* Example call: lcd_port_config(); 
*
*/ 
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/*
*
* Function Name: adc_pin_config
* input: no input
* output: no return
* logic: Function to configure analog devices as input and set the direction as floating 
* Example call: adc_pin_config();
*
*/
void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

/*
*
* Function Name: servo1_pin_config
* input: no input
* output: no return
* logic: Configure PORTB 5 pin for servo motor 1 operation
* Example call: servo1_pin_config();
*
*/
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*
*
* Function Name: servo2_pin_config
* input: no input
* output: no return
* logic: Configure PORTB 6 pin for servo motor 2 operation
* Example call: servo2_pin_config();
*
*/
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

/*
*
* Function Name: port_init
* input: no input
* output: no return
* logic: Function to Initialize PORTS either input or output
* Example call: port_init();
*
*/
void port_init()
{
	motion_pin_config();
	lcd_port_config();
	adc_pin_config();
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation	
}

/*
*
* Function Name: adc_init
* input: no input
* output: no return
* logic: Function to Initialize ADC 
* Example call: adc_init();
*
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
*
* Function Name: print_sensor
* input: row & coloumn number of the 16x2 lcd display and ADC channel number
* output: no return
* logic: This Function prints the Analog Value Of Corresponding Channel No. at required Row and Coloumn Location.
* Example call: print_sensor(1,1,1); // Prints the analog value of the channel no. 1 at first row and first coloumn location
*/
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/*
*
* Function Name: init_devices
* input: no input
* output: no return
* logic: It initializes all the devices either input or output
* Example call: init_devices();
*
*/
void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 timer1_init();
 timer5_init();
 adc_init();
 sei(); //Enables the global interrupts
}

/*
*
*Function Name: myforward()
*input: no input
*output:no return
*logic:it moves forward on basis of reading as like if only centre sensor is high,if it deflect from path it is given left or right turn a bit
to make the bot in centre so as to run line.
*Example call:myforward();
*
*/
void myforward() {
	print_sensor(2,2,1);							//Prints value of White Line Sensor1
	print_sensor(2,6,2);							//Prints Value of White Line Sensor2
	print_sensor(2,10,3);							//Prints Value of White Line Sensor3
	if (conversion_digital(1) == 0 && conversion_digital(2) == 1 && conversion_digital(3) == 0)
	{
		forward();
		velocity(left_speed,right_speed);
	}
	else if (conversion_digital(1) == 1 && conversion_digital(2) == 0 && conversion_digital(3) == 0)
	{
		soft_left();
		velocity(0,turn_right);
	}
	else if (conversion_digital(1) == 0 && conversion_digital(2) == 0 && conversion_digital(3) == 1)
	{
		soft_right();
		velocity(turn_left,0);		
	}
	else if (conversion_digital(1) == 0 && conversion_digital(2) == 1 && conversion_digital(3) == 1)
	{
		soft_right();
		velocity(turn_left,0);
	}
	else if (conversion_digital(1) == 1 && conversion_digital(2) == 1 && conversion_digital(3) == 0)
	{
		soft_left();
		velocity(0,turn_right);
	}
	return;
}

/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
void forward_wls(unsigned char node)
{
	int counta = 0;        // counter for counting the no. nodes robot has checked
	while (counta < node) {
		myforward();
		if ((int)ADC_Conversion(2)>=65)
		{
			counta = counta + 1;
			lcd_cursor(1,1);
			lcd_string("counta ");
			lcd_print(1,9,counta,1);
			forward();
			velocity(left_speed,right_speed);
			_delay_ms(360);
		}
		if (counta == node) {
			break;
		}
	}
	stop();
	_delay_ms(10);
	return;
}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(void)
{
	lcd_cursor(1,1);
	lcd_string("Inside left turn wls");
	while (conversion_digital(1)==0)
	{
		print_sensor(2,2,1);							//Prints value of White Line Sensor1
		print_sensor(2,6,2);							//Prints Value of White Line Sensor2
		print_sensor(2,10,3);							//Prints Value of White Line Sensor3
		left();
		velocity(102,162);
	}
	left();
	velocity(102,162);
	_delay_ms(35);
	stop();
	return;
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*
*/
void right_turn_wls(void)
{
	while (conversion_digital(3) == 0)
	{
		print_sensor(2,2,1);							//Prints value of White Line Sensor1
		print_sensor(2,6,2);							//Prints Value of White Line Sensor2
		print_sensor(2,10,3);							//Prints Value of White Line Sensor3
		right();
		velocity(turn_left,turn_right);
	}
	stop();
	return;
}

unsigned int conversion_digital_h5(int adc_channel)
{
	int adc_ret = (int)ADC_Conversion(adc_channel);     // for getting ADC_Conversion value 
	if (adc_ret <= 50)
	{
		adc_ret = 0;
	}
	else if (adc_ret > 50)
	{
		adc_ret = 1;
	}
	return adc_ret;
}

void myforward_h5()
{
	if (conversion_digital_h5(1)==1 && conversion_digital_h5(2)==0 && conversion_digital_h5(3)==1)
	{
		forward();
		velocity(left_speed,right_speed);
	}
	else if (conversion_digital_h5(1)==1 && conversion_digital_h5(2)==0 && conversion_digital_h5(3)==0)
	{
		soft_right();
		velocity(turn_left,turn_right);
	}
	else if (conversion_digital_h5(1)==1 && conversion_digital_h5(2)==1 && conversion_digital_h5(3)==0)
	{
		soft_right();
		velocity(turn_left,turn_right);
	}
	else if (conversion_digital_h5(1)==0 && conversion_digital_h5(2)==0 && conversion_digital_h5(3)==1)
	{
		soft_left();
		velocity(turn_left,turn_right);
	}
	else if (conversion_digital_h5(1)==0 && conversion_digital_h5(2)==1 && conversion_digital_h5(3)==1)
	{
		soft_left();
		velocity(turn_left,turn_right);
	}
	return;
}

void forward_wls_h5(unsigned int node)
{
	int counta = 0;          // counter for counting the no. nodes robot has checked
	while (counta < node) {
		print_sensor(2,2,1);							//Prints value of White Line Sensor1
		print_sensor(2,6,2);							//Prints Value of White Line Sensor2
		print_sensor(2,10,3);							//Prints Value of White Line Sensor3
		
		myforward_h5();
		if (conversion_digital_h5(1)==0 && conversion_digital_h5(2)==0 && conversion_digital_h5(3)==0)
		{
			counta = counta + 1;
			lcd_cursor(1,1);
			lcd_string("counta ");
			lcd_print(1,9,counta,1);
			forward();
			velocity(left_speed,right_speed);
			_delay_ms(250);
		}
		if (counta == node) {
			break;
		}
	}
	stop();
	_delay_ms(10);
	return;
}

void h5()
{
	forward_wls_h5(1);
	left();
	velocity(left_speed,right_speed);
	_delay_ms(800);	
}

void sharp_right()
{
	right();             //////////////////////////////////////////////////
	velocity(left_speed,right_speed);
	_delay_ms(950);
	stop();
}

void sharp_left()
{
	left();
	velocity(left_speed,right_speed);
	_delay_ms(940);
	stop();
}

//Main Function
int main(void)
{
	init_devices();
	lcd_init();
	lcd_set_4bit();

// 	print_sensor(2,2,1);							//Prints value of White Line Sensor1
// 	print_sensor(2,6,2);							//Prints Value of White Line Sensor2
// 	print_sensor(2,10,3);							//Prints Value of White Line Sensor3
// 	_delay_ms(2000);
	
	servo_1(16);
	_delay_ms(500);
	servo_2(0);
	_delay_ms(500);
	
	forward_wls(1);          /////////// W4-H2
	left_turn_wls();
	forward_wls(1);
	left_turn_wls();
	left();
	velocity(left_speed,right_speed);
	_delay_ms(10);
	stop();
	forward_wls(1);
	forward();
	velocity(left_speed,right_speed);
	_delay_ms(30);
	sharp_right();
	stop();
	pick();
	
	_delay_ms(500);
	left();
	velocity(left_speed,right_speed);
	_delay_ms(90);
	left_turn_wls();
	forward_wls(1);
	
	stop();
		right();             //////////////////////////////////////////////////
		velocity(left_speed,right_speed);
		_delay_ms(1000);
		stop();
	forward();
	velocity(left_speed,right_speed);
	_delay_ms(210);
	stop();
	place();
	_delay_ms(500);
	
	back();      ///// H2-W7
	velocity(left_speed,right_speed);
	_delay_ms(200);
	stop();
	left();
	velocity(left_speed,right_speed);
	_delay_ms(55);
	sharp_left();
	forward_wls(1);
	stop();
	sharp_left();
	stop();
	pick();
	_delay_ms(500);           //// W7-H5
	right();
	velocity(left_speed,right_speed);
	_delay_ms(70);
	sharp_right();
	stop();	
	forward_wls(1);
	forward_wls(1);
	forward_wls(1);
	left_turn_wls();
	stop();
		
		
		
	sharp_left();
	left();
	velocity(240,250);
	_delay_ms(50);
	stop();
	forward();
	velocity(left_speed,right_speed);
	_delay_ms(285);
	stop();
	_delay_ms(100);
	forward_wls_h5(1);
	stop();
	sharp_left();
	back();
	velocity(left_speed,right_speed);
	_delay_ms(220);
	stop();
	place_low();
	_delay_ms(500);
	 
	 sharp_left();
	 left();
	 velocity(left_speed,right_speed);
	 _delay_ms(100);
	 stop();
	 forward();
	 velocity(left_speed,right_speed);
	 _delay_ms(280);
	 stop();
	 _delay_ms(100);
	 while(1){
		 myforward_h5();
		 _delay_ms(10);
		 if (conversion_digital_h5(1) == 0 && conversion_digital_h5(2) == 0 && conversion_digital_h5(3) ==0)
		 {
			 break;
		 }
	 }
	 stop();
 	forward_wls(1);
	 forward();
	 velocity(left_speed,right_speed);
	 _delay_ms(80);
	 while (conversion_digital(3)==0)
	 {
		 print_sensor(2,2,1);							//Prints value of White Line Sensor1
		 print_sensor(2,6,2);							//Prints Value of White Line Sensor2
		 print_sensor(2,10,3);							//Prints Value of White Line Sensor3
		 right();
		 velocity(150,180);
	 }
	 forward_wls(1);
	 forward();
	 velocity(left_speed,right_speed);
	 _delay_ms(150);
	 sharp_right();
	 right();
	 velocity(left_speed,right_speed);
	 _delay_ms(50);
	 stop();
	 pick();
	 
	 sharp_left();
	 left();
	 velocity(left_speed,right_speed);
	 _delay_ms(90);
	 stop();
	 forward_wls(1);
	 
	 sharp_right();
	 right();
	 velocity(left_speed,right_speed);
	 _delay_ms(60);
	 stop();
	 forward_wls(1);
	 forward();
	 velocity(left_speed,right_speed);
	 _delay_ms(250);
	 stop();
	 place();
	 
	 sharp_right();
	 right();
	 velocity(left_speed,right_speed);
	 _delay_ms(80);
	 forward_wls(1);
	 
	 
	 
	 
	 
	 
	 
// 	back();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(100);
// 	stop();
// 	left();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(10);
// 	left_turn_wls();
// 	forward_wls(1);
// 	stop();
// 	
// 	sharp_right();
// 	forward();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(25);
// 	stop();
// 	pick();
// 	_delay_ms(100);
// 	left();
// 	velocity(turn_left,turn_right);
// 	_delay_ms(150);
// 	left_turn_wls();
// 	forward_wls(1);
// 	stop();
// 		right();             //////////////////////////////////////////////////
// 		velocity(left_speed,right_speed);
// 		_delay_ms(1100);
// 	stop();
// 	forward();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(330);
// 	stop();	
// 	place();
// 	
// 	_delay_ms(10);             // H5-W7
// 	back();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(50);
// 	left();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(50);
// 	left_turn_wls();
// 	forward_wls(1);
// 	sharp_left();
// 	stop();
// 	_delay_ms(10);
// 	pick();
// 	_delay_ms(100);
// 	
// 	sharp_right();    /// W7-H5
// 	stop();
// 	forward_wls(1);
// 	forward_wls(1);
// 	forward_wls(1);
// 	
// 	left_turn_wls();      
// 	forward();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(420);
// 	h5();
//  	place_low();
// 	 
// 	sharp_left();
// 	myforward_h5();
// 	_delay_ms(1000);
// 	 
// 	
	
	
// 	forward();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(600);
// 	
// 	h5(); 
// 	
// 	
// 	right();
// 	velocity(left_speed,right_speed);
// 	_delay_ms(840);
// 	
	
	print_sensor(2,2,1);							//Prints value of White Line Sensor1
	print_sensor(2,6,2);							//Prints Value of White Line Sensor2
	print_sensor(2,10,3);							//Prints Value of White Line Sensor3
	
	
 	while(1)
  	{
// 		forward_wls(1);
//  		right();
// 		velocity(150,190);
// 		stop();
//  	
		

// 		BATT_V = ADC_Conversion(0);
// 		BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;	//Prints Battery Voltage Status
// 		lcd_print(1,1,BATT_Voltage,4);

		//print_sensor(1,1,0);							//Prints Battery voltage binary value

//  		print_sensor(2,4,5);							//Prints IR Proximity Sensor 1
//   		print_sensor(2,10,6);							//Prints vlaue of Analog IR Proximity Sensor 2
// 		print_sensor(1,14,7);							//Prints value of Analog IR Proximity Sensor 3
		print_sensor(2,2,1);							//Prints value of White Line Sensor1
		print_sensor(2,6,2);							//Prints Value of White Line Sensor2
 		print_sensor(2,10,3);							//Prints Value of White Line Sensor3
		//print_sensor(2,9,11); 						//Analog Value Of Front Sharp Sensor

//  		sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
// 		value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
// 		lcd_print(2,14,value,3); 						//Prints Value Of Distanc in MM measured by Sharp Sensor.
// 		_delay_ms(1000);
	  }
}
  
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*
*
* Function Name: timer1_init
* input: no input
* output: no return
* Logic: Function to Initialize the timer 1
* Example call: timer1_init();
*
*/ 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
*
* Function Name: servo_1
* input: no input
* output: no return
* Logic: Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
* Example call: servo_1(50); //It servo 1 by 50 degree
*
*/
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}

/*
*
* Function Name: servo_2
* input: no input
* output: no return 
* Logic: Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
* Example call: servo_2(90); //It servo 2 by 90 degree
*
*/
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

/*
*
* Function Name: pick
* input: no input
* output: no return
* Logic: Function to pick the construction materials
* Example call: pick(); 
*
*/
void pick(void)
{
	servo_2(0);
	_delay_ms(500);
	servo_1(135);
	_delay_ms(500);
	servo_2(42);
	_delay_ms(500);
	servo_1(10);
	return;
}

/*
*
* Function Name: place
* input: no input
* output: no return
* Logic: Function to place the construction materials
* Example call: place();
*
*/
void place(void)
{
	servo_2(0);
}

void place_low()
{
	servo_1(80);
	_delay_ms(500);
	
	servo_2(0);
	_delay_ms(500);
	servo_1(21);
}
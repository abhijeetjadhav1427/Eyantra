#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char ADC_Conversion(unsigned char);
int conversion_digital(int);
void timer5_init();
void velocity (unsigned char, unsigned char);
void motion_set (unsigned char);
unsigned int Sharp_GP2D12_estimation(unsigned char);
void forward (); 
void back();
void left();
void right();
void soft_left();
void soft_right();
void stop();
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

int conversion_digital(int adc_channel)
{
	int adc_ret = (int)ADC_Conversion(adc_channel);
	if (adc_ret <= 7)
	{
		adc_ret = 0;
	}
	else if (adc_ret > 7)
	{
		adc_ret = 1;
	}
	return adc_ret;
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;
 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTD; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTD = PortARestore; 			// setting the command to the port
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}


void forward (void) //both wheels forward
{
	motion_set(0x05);
}

void back (void) //both wheels backward
{
	motion_set(0x0A);
}

void right (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x09);
}

void left (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x06);
}

void soft_right (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x01);
}

void soft_left (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x04);
}

void stop (void)
{
	motion_set(0x00);
}
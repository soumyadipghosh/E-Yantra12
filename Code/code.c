
/* The following code is used to implement the line follower theme in the FIREBIRD - V ATmega2560 */ 


#define F_CPU 14745600 //frequency of CPU
#include <avr/io.h> //for I/O Operations
#include <avr/interrupt.h> //for Interrupt Handling
#include <util/delay.h> //for generating delay
#include <math.h> //included to support power function
#include "lcd.h" //for including LCD related functions

unsigned char ADC_Value;
unsigned char First, Second, Third, Fourth, Fifth, Sixth, Seventh;
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning

unsigned int arr[7];// array to store the values of each sensor in the 7x line sensor module
unsigned long int arr_value; //a 7 digit long integer value indicating current status of 7 sensors from left to right 
unsigned long int prev_values[7];//array to store previous 7 sensor values

//sensor calibration thresholds; these may vary depending on the ambient conditions of the arena
int t1 = 100;
int t2 = 100;
int t3 = 100;
int t4 = 200;
int t5 = 200;
int t6 = 200;
int t7 = 200;

//logic reverse variable used when white line changes to black line
int line = 0;//0 is used when following white line, when following black line, 1 is used.

//x1 and x2 variables for governing kind of line following.
//initially bot is following white line, so when sensor value is below threshold, x1(=1) is assigned to the corresponding arr[] variable.
//when black line comes, x1 and x2 are interchanged; now sensor value is above threshold, then x2(=1) is assigned to the corresponding arr[] variable
int x1 = 1;
int x2 = 0;

float pGain = 60; //Proportional Gain for generation of control variable to regulate the velocity of the motors
float control; //the control variable
float s; //the average value of the sensors

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

void spi_pin_config (void)
{
	DDRB = DDRB | 0x07;
	PORTB = PORTB | 0x07;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize Buzzer
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;	//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pullup for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pullup for PORTE 4 pin
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}


//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	spi_pin_config();
	motion_pin_config();
	buzzer_pin_config();
	left_encoder_pin_config(); 
	right_encoder_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256 PWM 8bit fast, TOP=0x00FF
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
//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref = 5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function To Initialize SPI bus
// clock rate: 921600hz
void spi_init(void)
{
	SPCR = 0x53; //setup SPI
	SPSR = 0x00; //setup SPI
	SPDR = 0x00;
}

//Function to send byte to the slave microcontroller and get ADC channel data from the slave microcontroller
unsigned char spi_master_tx_and_rx (unsigned char data)
{
	unsigned char rx_data = 0;

	PORTB = PORTB & 0xFE; // make SS pin low
	SPDR = data;
	while(!(SPSR & (1<<SPIF))); //wait for data transmission to complete

	_delay_ms(1); //time for ADC conversion in the slave microcontroller
	
	SPDR = 0x50; // send dummy byte to read back data from the slave microcontroller
	while(!(SPSR & (1<<SPIF))); //wait for data reception to complete
	rx_data = SPDR;
	PORTB = PORTB | 0x01; // make SS high
	return rx_data;
}


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

//Buzzer functions
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

//This function initializes all the arrays with value 0
void array_initialise()
{
	int i;
	for(i=0; i < 7; i++)
	{
		prev_values[i] = 0;
		arr[i] = 0;
	}
}

//Function to shift prev_values[] cells left by one cell when new sensor reading comes
void shift(long int value)
{
	int i;
	for(i = 0; i < 7 - 1; i++)
	{
		prev_values[i] = prev_values[i+1];
	}
	prev_values[6] = value;
}

//Function used to perform comparisons among cells in prev_values array.
//Here we search for the existence of a particular value of 1 or more sensors in the 7x sensor array."value" is the search parameter,
//“start_index” is the sensor number from which we are considering
//and “end_index” is the sensor number upto which we are considering.
//For e.g, a function call of prev_values_search(111, 3, 5) will check
//if any of the previous sensor states have the state 1 for the 3rd,
//4th and 5th sensors. Care must be taken that the value provided for
//searching must have number of digits in accordance with the
//start_index and end_index,i.e, they must have (end_index – start_index + 1)
//number of digits. For checking values of non-consecutive sensors, the
//function can be called separately for each sensor.

int prev_values_search(long int value, int start_index, int end_index)
{
	int i;//local array loop counter
	int flag = 0;//status
	for(i = 0; i < 7; i++)
	{
		long int temp = prev_values[i];
		long int x = (long int)(pow(10, 7 - end_index));
		long int y = (long int)(pow(10, end_index - start_index + 1));
		//The pow function yields a double value which may vary slightly when approximated i.e, type-casted. Usually it becomes lesser by 1. So
		//we incorporate necessary corrections for both x and y
		if(x == 0 || x == 9 || x == 99 || x == 999 || x == 9999 || x == 99999 || x == 999999 || x == 9999999)
		{
			x += 1;
		}
		if(y == 0 || y == 9 || y == 99 || y == 999 || y == 9999 || y == 99999 || y == 999999 || y == 9999999)
		{
			y += 1;
		}
		temp = temp / x;
		temp = temp % y;
		
		if(value == temp)
		{
			flag = 1;
			return flag;//found match
		}
	}
	return flag;
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	spi_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei(); //Enables the global interrupts
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibble for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void)
{
	motion_set (0x06);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void stop (void)
{
	motion_set (0x00);
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
 	lcd_print(1, 1, ShaftCountLeft, 3);
 	lcd_print(1, 5, ShaftCountRight, 3);
	lcd_print(1, 9, ReqdShaftCountInt, 3);
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop action
}

//Function used for moving robot forward by specified distance
	
void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
 	lcd_print(1, 1, ShaftCountLeft, 3);
 	lcd_print(1, 5, ShaftCountRight, 3);
	lcd_print(1, 9, ReqdShaftCountInt, 3);
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop action
}

//Function to move bot by DistanceInMM
void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

//Function to move bot backward by DistanceInMM
void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

//Function to move the bot left by given Degrees
void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}


//Function to move the bot right by given Degrees
void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

//Function to move the bot left softly by given Degrees
void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

//Function to move the bot right softly by given Degrees
void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

//This function calculates the weighted average value of the 7 sensors and
//also assigns value to the sensor-related variables and arrays
float readSensors()
{
	float avgSensors = 0.0;
	
	First = ADC_Conversion(3); //1st sensor
	Second = ADC_Conversion(2); //2nd sensor
	Third = ADC_Conversion(1); //3rd sensor
	Fourth = spi_master_tx_and_rx(0); //4th sensor
	Fifth = spi_master_tx_and_rx(1); //5th sensor
	Sixth = spi_master_tx_and_rx(2); //6th sensor
	Seventh = spi_master_tx_and_rx(3); //7th sensor
	
	if(First < t1)
	{
		arr[0] = x1;
	}
	else
	{
		arr[0] = x2;
	}
	
	if(Second < t2)
	{
		arr[1] = x1;
	}
	else
	{
		arr[1] = x2;
	}
	
	if(Third < t3)
	{
		arr[2] = x1;
	}
	else
	{
		arr[2] = x2;
	}
	
	if(Fourth < t4)
	{
		arr[3] = x1;
	}
	else
	{
		arr[3] = x2;
	}
	
	if(Fifth < t5)
	{
		arr[4] = x1;
	}
	else
	{
		arr[4] = x2;
	}
	
	if(Sixth < t6)
	{
		arr[5] = x1;
	}
	else
	{
		arr[5] = x2;
	}
	
	if(Seventh < t7)
	{
		arr[6] = x1;
	}
	else
	{
		arr[6] = x2;
	}
	
	arr_value = arr[0] * 1000000 + arr[1] * 100000 + arr[2] * 10000 + arr[3] * 1000 + arr[4] * 100 + arr[5] * 10 + arr[6];
	
	//Displaying the variable arr_value on the LCD. Since the lcd_print() function can print maximum of 5 digits, we break the 
	//number into 2 parts and then print.
	int a1 = arr_value / 100;
	int a2 = arr_value % 100;
	lcd_print(2, 3, a1, 5);
	lcd_print(2, 8, a2, 2);
	
	
	//calculate weighted mean
	float sum = arr[0] + arr[1] + arr[2] + arr[3] + arr[4] + arr[5] + arr[6];
	if(sum == 0) //to avoid division by 0
	{
		return 0.0;
	}
	avgSensors = ((arr[0] * 1 + arr[1] * 2 + arr[2] * 3 + arr[3] * 4 + arr[4] * 5 + arr[5] * 6 + arr[6] * 7) / sum );
	return avgSensors;
}

//Function to generate the control variable
float PID(float cur_value, float req_value) //Here the average sensor value is the cur_value
                                            //and 4.0 is the required value
{
	float pid;
	float error;
	error = req_value - cur_value;
	pid = (pGain * error);
	if(pid < 0) pid *= -1; //To extract only magnitude of control without sign
	return pid;
}


//This function specifies the algorithm for traversing the grid.
//Here we use fixed movements to make the robot traverse the grid.
void gridTraversal()
{
	forward();
	velocity(175, 175);
	forward_mm(260);
	right_degrees(45);
	forward_mm(190);
	left_degrees(45);
	forward_mm(120);
	right_degrees(90);
	forward_mm(155);
	left_degrees(90);
}

//Main Function
int main(void)
{
	init_devices();
	lcd_set_4bit(); 
	lcd_init();
	array_initialise();
	
	int turn = 0; //This variable keeps track of the last turn taken by the robot which is useful in differentiating certain cases.
	              //It is 0 for left turn and 1 for right turn.
	while(1)
	{
		
		s = readSensors(); //Current sensor reading
		
		//Generating control variable
		control = PID(s, 4.0);
		
		lcd_print(1, 13, s*10, 2); //printing the average sensor value. Since fractional part cannot be printed,
		                           //we multiply it by 10 and then print.
		lcd_print(1, 7, line, 1); //displays the logic of line following , i.e , white or black line
		lcd_print(1, 5, turn, 1); //displays the kind of last turn taken by the bot
		
		//Condition for detecting the start of the grid. Details have been already mentioned in implementation analysis. Also we check
		//if the last turn taken by the bot is a right one (according to the arena) so that even if such a sensor state occurs elsewhere,
		//possibility of error is minimized.
		if(line == 0 & (arr_value == 11111 | arr_value == 111111) & prev_values_search(0,1,2) & prev_values_search(0,6,7) & (turn == 1))
		{
			_delay_ms(300);
			gridTraversal();
		}
		
		
		if((arr_value == 1100011 && (prev_values_search(11100,1,7) == 1)) || (arr_value == 1110011 && (prev_values_search(1100,1,7) == 1)) || (arr_value == 1100111 && (prev_values_search(11000,1,7) == 1)))
		{
			buzzer_on(); //buzzer is made ON for 0.5s to understand if the logic of line following has changed 
			_delay_ms(500);
			buzzer_off();
			x1 = 0;
			x2 = 1;
			line = 1;
		}
		
		if(arr_value != 0)//at least 1 sensor on the line
		{ 
			if(s <= 4)//left sensor sees the line, so turn left
			{
				forward();
				velocity(200 - control , 200); //velocity of left motor is decreased to initiate the left turn.
				
			}
			
			if(s > 4)//right sensor sees the line, so turn right
			{
				forward();
				velocity(200, 200 - control); //velocity of right motor is decreased to initiate the right turn.
			}
		}
		else
		{
			if(((prev_values_search(1, 1,1) == 1)) &&  (prev_values_search(11, 6,7) == 0)) //if previous sensor values show that 1st sensor 
			                                                                               //was active and 6th and 7th were inactive. 
			{
				do 
				{
					s = readSensors();
					left();
					velocity(175, 175);
					//average sensor value and line has to be printed seperately in this loop.
					lcd_print(1, 13, s*10, 2);
					lcd_print(1, 7, line, 1);
					turn = 0; //turn made 0 for left turn
				}while(s < 3.5 || s > 4.5);
			}	
			
			else if(((prev_values_search(1, 7,7) == 1)) &&  (prev_values_search(11, 1,2) == 0)) //if previous sensor values show that 7th sensor
			                                                                                    //was active and 1st and 2nd were inactive.
			{
				do
				{
					s = readSensors();
					right();
					velocity(175, 175);
					//average sensor value and line has to be printed seperately in this loop.
					lcd_print(1, 13, s*10, 2);
					lcd_print(1, 7, line, 1);
					turn = 1; //turn made 1 for right turn
				}while(s < 3.5 || s > 4.5);
			}	
			else //if none of the above condition occurs, robot moves back till it tracks the line. This is a kind of recovery logic.
			{
				stop();
				velocity(200,200);
				back();
			}				
		}						
		shift(arr_value); //shift the current sensor value "arr_value" into prev_values[] array so that it is available as reference for 
		                  //next states.
	}
}	
/************************************************************  
 @file SumoRobot.c

 @author Anshul Kirti (kirtia@msoe.edu)

 @brief The following code intializes the robot using the line sensor and the object sensor.
        MSOE library is needed. Arduino Uno is the board.
*/
/************************************************************/

#include <avr/io.h>
#include <inttypes.h>
#include <MSOE/delay.c>
#include <MSOE/ bit.c>
#include <avr/interrupt.h>

/**********************************************************************
                        FUNCTION DECLARATION
*********************************************************************/
void moveForward(void);
void moveBackward(void);
void moveRight(void);
void moveLeft(void);
void lineSensorBehaviour(int s1, int s2, int s3, int s4);
void labTest(int s1, int s2, int count);
void objConfig(void);
/*********************************************************************/

int sensor1ReadValue;
int sensor2ReadValue;
int sensor3ReadValue;
int sensor4ReadValue;
int data;

/******************************************************************
                        MACRO DECLARATION
******************************************************************/
#define LINE_SENSOR_1 (1<<PC0) // Front right line sensor
#define LINE_SENSOR_2 (1<<PC1) // Front left line sensor
#define LINE_SENSOR_3 (1<<PC2) // Back right line sensor
#define LINE_SENSOR_4 (1<<PC3) // Back left line sensor

#define IR_LED_PIN (1<<PB3)
#define IR_TRANSMITTER_PIN (1<<PD6)
#define IR_RECEIVER_PIN (1<<PB0)

#define DIRECTION_PIN_1 (1<<PD3)
#define DIRECTION_PIN_2 (1<<PD2)

#define WHITE 0 // White is 0
#define BLACK 1 // Black is 1

#define TurnLedOn  PORTB |= IR_LED_PIN
#define TurnLedOff PORTB &= ~IR_LED_PIN
/***************************************************************/

/**************************************************************
 @brief The function is used to initialize the initalize the line sensor.
        Mechanism is to measure the amount of light reflected.
***************************************************************/
void lineSensorInit()
{
	//Initialize pin for the line sensors. Here pin A0, A1, A2 and A3 are used.
	DDRC |= (LINE_SENSOR_1) | (LINE_SENSOR_2) | (LINE_SENSOR_3) | (LINE_SENSOR_4);
	PORTC|= (LINE_SENSOR_1) | (LINE_SENSOR_2) | (LINE_SENSOR_3) | (LINE_SENSOR_4);

	// Add delay of 10 us for the line sensor. It is needed in order to charge the capacitors in the sensor
	delay_us(10);

	DDRC =0;
	PORTC = 0;

	delay_ms(5); // So that it doesn't sense the orange line on the Ring.

	//Read the value of the pin attached to the sensor.
	 sensor1ReadValue = (PINC & LINE_SENSOR_1);
	 sensor2ReadValue = (PINC & LINE_SENSOR_2);
	 sensor3ReadValue = (PINC & LINE_SENSOR_3);
	 sensor4ReadValue = (PINC & LINE_SENSOR_4);
}

/**************************************************************
 @brief Initialize the Pins
***************************************************************/
void pinInit()
{
	//Initialize the direction pins.
    DDRD|= (DIRECTION_PIN_1) | (DIRECTION_PIN_2); //Make the pin as output
}

/**************************************************************
 @brief Main function for program execution
***************************************************************/
int main(void)
{
  // Initialize the pin
	pinInit();

  // Initialize the object sensor
  objConfig();

  while(1)
  {
    // Initialize the line senors
    lineSensorInit();

    // If the white is sensed via the front sensors, move backwards.
    if(sensor1ReadValue == WHITE || sensor2ReadValue == WHITE)
    {
  	 moveBackward();
  	 delay_ms(1000);
    }
  else
    {
  	 TurnLedOff;

  	 if (!data) // When object is detected
  	 {
  	   TurnLedOn;
      moveForward();
  	 }
  	 else
  	 {
  	   moveLeft();
  	   TurnLedOff;
  	 }
    }

    if(sensor3ReadValue == WHITE || sensor4ReadValue == WHITE)
    {
     	moveForward();
      delay_ms(1000);
    }
  }
}

void moveForward(void)
{
    PORTD |=DIRECTION_PIN_2; // Give the value 1 to the pin
    PORTD |=DIRECTION_PIN_1; // Give the value 1 to the pin
}
void moveBackward(void)
{
    PORTD &=~DIRECTION_PIN_1;
    PORTD &=~DIRECTION_PIN_2;
}
void moveRight(void)
{
    PORTD |=DIRECTION_PIN_2;
    PORTD &= ~DIRECTION_PIN_1;
}
void moveLeft(void)
{
    PORTD &= ~DIRECTION_PIN_2;
    PORTD |= DIRECTION_PIN_1;
}

/* IR Receiver : Connection from front -
 *               Right : 5 V
 *               Middle: Ground
 *               Left  : PB0 (OC1B) (digital pin 10)
 *
 * IR Transmitter : Long - PD6 (OC0A) (digital pin 6) [Add resistor to this leg to increase or decrease the distance.
 *                                                     To decrease the distance, use small resistor.]
 *                  Small - Ground
 *
 * PB 3 - used for the LED indication.
 */
void objConfig (void)
{
 DDRB |= IR_LED_PIN; // LED
 DDRD |= IR_TRANSMITTER_PIN; // Trnsmitter
 DDRB &= ~IR_RECEIVER_PIN; //Receiver
 TCCR2B |= (1<<WGM22);
 TCCR2B |= (1<<CS20);
 TCCR2B |= (1<<CS21);
 TCCR2B |= (1<<CS22);
 OCR2A = 6;
 OCR2B = 85;
 TIMSK2 |= (1<<OCIE2A);
 TIMSK2 |= (1<<OCIE2B);
 sei();
 TCCR0A |= (1<<WGM01);
 TCCR0A |= (1<<COM0A0);
 OCR0A = 25; // Increase to increase the range. Keep the resistor in mind
}
ISR(TIMER2_COMPA_vect)
{
 TCCR0B |= (1<<CS01);
}

ISR(TIMER2_COMPB_vect)
{
 TCCR0B &= (~(1<<CS01));
 data = PINB & (IR_RECEIVER_PIN);
}

/*		ROBOKITS ARDUINO USB/BLUETOOTH 18 SERVO CONTROLLER FIRMWARE

This code is generated from Robokits Arduino Bluetooth/USB Servo Controller software.
The hardware consist of 2 AVRs out of which one is Master and has Arduino bootloader. The second one is slave and runs all servo routines.
The master controls slave through I2C and slave operates all servos.
Many pins of the main controller are free and can be used for connecting other peripherals and interfaces.

Firmware contains 2 parts (functions in void loop()) - 1. Run in PC controlled Mode and 2. Run User Code
The code is selected through a Jumper on Servo controller board.
You can change the user code as per your need.


This code is generated for Arduino UNO board profile and the same should be chosed while programming with Arduino IDE.
Arduino IDE veersion 1.6.5 + is recommended.

If the code is moved to other servo base platform like a robot, offsets must be set properly so that servos will move correctly as per program.

*/

#include <Wire.h>

#define servo1 (16 >> 1)
#define servo2 (18 >> 1)
#define UART_BAUD_RATE 115200
#define LED 13

#define SERIAL_BUFFER_SIZE 256

void I2C_SERVOSET(unsigned char servo_num, unsigned int servo_pos);
void setServoPosition(unsigned char servo_num, unsigned int servo_pos);
void I2C_SERVOREVERSE(unsigned char servo_num, unsigned char servo_dir);
void I2C_SERVOOFFSET(unsigned char servo_num, int value);
void I2C_SERVOSPEED(unsigned char value);
void I2C_SERVOMIN(unsigned char servo_num, unsigned int servo_pos);
void I2C_SERVOMAX(unsigned char servo_num, unsigned int servo_pos);
char I2C_SERVOEND(void);
void PCControlledCode(void);
void UserCode(void);
void LEDToggle(void);

volatile int cnt, c, servoval;
volatile char state, servobuf[36], bytecnt;

int interval = 100;
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
char runCode = 0; // 1 for PC controlled, 0 for user controlled
char LEDState = 0;
float accel_x, accel_y, accel_z;

#define State_Start 0
#define State_Command 1
#define State_Servoposition 2
#define State_Speed 3
#define State_Servomin 4
#define State_Servomax 5
#define State_Servooffset 6
#define State_Servoreverse 7
#define State_Servonutral 8
#define State_ReadOffsets 9

// The Initialization of different variables in the program
void setup()
{
	cnt = 0;
	int i;
	unsigned int x;
	char buffer[10], tmp, tmp1;
	float range;
	Serial.begin(UART_BAUD_RATE);
	pinMode(13, OUTPUT);
	pinMode(2, OUTPUT);
	pinMode(8, INPUT);
	digitalWrite(8, 1);
	delay(500);
	sei();
	Wire.begin();
	TWSR = 3;  // no prescaler
	TWBR = 18; // Set I2C speed lower to suite I2C Servo controller
	pinMode(2, OUTPUT);
	digitalWrite(2, HIGH);
	delay(500);
	state = State_Start;

	if (digitalRead(8))
	{
		runCode = 1;
	}
	else
	{
		runCode = 0;
	}
}

// Finally the arduino board runs this function to do the task

void loop()
{
	if (runCode == 1)
	{
		UserCode();
	}
	else
	{
		LEDToggle();
	}
}

void I2C_SERVOSET(unsigned char servo_num, unsigned int servo_pos)
{
	if (servo_pos < 500)
		servo_pos = 500;
	else if (servo_pos > 2500)
		servo_pos = 2500;

	if (servo_pos > 501)
		servo_pos = (((servo_pos - 2) * 2) - 1000);
	else
		servo_pos = 0;

	if (servo_num < 19)
		Wire.beginTransmission(servo1);
	else
		Wire.beginTransmission(servo2);
	Wire.write(servo_num - 1);
	Wire.write(servo_pos >> 8);
	Wire.write(servo_pos & 0XFF);
	Wire.endTransmission();
}

void setServoPosition(unsigned char servo_num, unsigned int servo_pos)
{
	const unsigned int move_duration = 40; // Duration to move above and below in milliseconds

	// Ensure servo_pos is within the valid range
	if (servo_pos < 500)
		servo_pos = 500;
	else if (servo_pos > 2500)
		servo_pos = 2500;

	// Determine the adjusted position based on accelerometer data
	int adjusted_pos = servo_pos;
	int move_range = abs(accel_x * 1000) + abs(accel_y * 1000) + abs(accel_z * 100); // Adjust the scale factor as needed

	// Move above the desired position
	for (unsigned int t = 0; t < move_duration; t += 5)
	{
		I2C_SERVOSET(servo_num, adjusted_pos + move_range);
		delay(5); // Wait for 5 milliseconds
	}

	// Settle at the desired position
	I2C_SERVOSET(servo_num, adjusted_pos);
	delay(move_duration);

	// Move below the desired position
	for (unsigned int t = 0; t < move_duration; t += 20)
	{
		I2C_SERVOSET(servo_num, adjusted_pos - move_range);
		delay(20); // Wait for 20 milliseconds
	}

	// Settle back at the desired position
	I2C_SERVOSET(servo_num, adjusted_pos);
}

/*
void setServoPosition(unsigned char servo_num, unsigned int servo_pos)
{
	const unsigned int move_duration = 20;		   // Duration to move above and below in milliseconds
	const unsigned int move_range = rand() % +300; // Range of movement above and below the desired position

	// Ensure servo_pos is within the valid range

	// Determine the adjusted position
	int adjusted_pos = servo_pos;

	// Move above the desired position
	for (unsigned int t = 0; t < move_duration; t += 5)
	{
		I2C_SERVOSET(servo_num, adjusted_pos + move_range);
		// Wait for 20 milliseconds
	}

	// Settle at the desired position
	I2C_SERVOSET(servo_num, adjusted_pos);
	delay(move_duration); // Wait for the move_duration milliseconds

	// Move below the desired position
	for (unsigned int t = 0; t < move_duration; t += 20)
	{
		I2C_SERVOSET(servo_num, adjusted_pos - move_range);
		// Wait for 20 milliseconds
	}

	// Settle back at the desired position
	I2C_SERVOSET(servo_num, adjusted_pos);
}*/

void I2C_SERVOMIN(unsigned char servo_num, unsigned int servo_pos)
{
	if (servo_pos < 500)
		servo_pos = 500;
	else if (servo_pos > 2500)
		servo_pos = 2500;
	servo_pos = ((servo_pos * 2) - 1000);

	if (servo_num < 19)
		Wire.beginTransmission(servo1);
	else
		Wire.beginTransmission(servo2);
	Wire.write((servo_num - 1) + (18 * 4));
	Wire.write(servo_pos >> 8);
	Wire.write(servo_pos & 0XFF);
	Wire.endTransmission();
	delay(20);
}

void I2C_SERVOMAX(unsigned char servo_num, unsigned int servo_pos)
{
	if (servo_pos < 500)
		servo_pos = 500;
	else if (servo_pos > 2500)
		servo_pos = 2500;
	servo_pos = ((servo_pos * 2) - 1000);

	if (servo_num < 19)
		Wire.beginTransmission(servo1);
	else
		Wire.beginTransmission(servo2);
	Wire.write((servo_num - 1) + (18 * 3));
	Wire.write(servo_pos >> 8);
	Wire.write(servo_pos & 0XFF);
	Wire.endTransmission();
	delay(20);
}

void I2C_SERVOSPEED(unsigned char value)
{
	Wire.beginTransmission(servo1);
	Wire.write(18 * 2);
	Wire.write(value);
	Wire.write(0);
	Wire.endTransmission();
	Wire.beginTransmission(servo2);
	Wire.write(18 * 2);
	Wire.write(value);
	Wire.write(0);
	Wire.endTransmission();
	delay(20);
}

void I2C_SERVOOFFSET(unsigned char servo_num, int value)
{
	value = 3000 - value;
	value = value - 1500;

	if (value < -500)
		value = -500;
	else if (value > 500)
		value = 500;

	if (value > 0)
		value = 2000 + (value * 2);
	else if (value <= 0)
		value = -value * 2;

	if (servo_num < 19)
		Wire.beginTransmission(servo1);
	else
		Wire.beginTransmission(servo2);
	Wire.write((servo_num - 1) + (18 * 6));
	Wire.write(value >> 8);
	Wire.write(value & 0XFF);
	Wire.endTransmission();
	delay(20);
}

void I2C_SERVOREVERSE(unsigned char servo_num, unsigned char servo_dir)
{
	if (servo_dir > 0)
		servo_dir = 1;
	if (servo_num < 19)
		Wire.beginTransmission(servo1);
	else
		Wire.beginTransmission(servo2);
	Wire.write((servo_num - 1) + (18 * 7));
	Wire.write(servo_dir);
	Wire.write(0);
	Wire.endTransmission();
	delay(20);
}

char I2C_SERVOEND(void)
{
	int i, n;
	char buffer;
	Wire.beginTransmission(servo1);
	n = Wire.write(181);
	if (n != 1)
		return (-10);

	n = Wire.endTransmission(false);
	if (n != 0)
		return (n);

	delayMicroseconds(350);
	Wire.requestFrom(servo1, 1, true);
	while (Wire.available())
		buffer = Wire.read();

	return (buffer);
}

void ServoSetAll(unsigned int Servo1, unsigned int Servo2, unsigned int Servo3, unsigned int Servo4, unsigned int Servo5, unsigned int Servo6, unsigned int Servo7, unsigned int Servo8, unsigned int Servo9, unsigned int Servo10, unsigned int Servo11, unsigned int Servo12, unsigned int Servo13, unsigned int Servo14, unsigned int Servo15, unsigned int Servo16, unsigned int Servo17, unsigned int Servo18)
{
	if (Servo1 >= 500)
	{
		I2C_SERVOSET(1, Servo1);
	}
	if (Servo2 >= 500)
	{
		I2C_SERVOSET(2, Servo2);
	}
	if (Servo3 >= 500)
	{
		I2C_SERVOSET(3, Servo3);
	}
	if (Servo4 >= 500)
	{
		I2C_SERVOSET(4, Servo4);
	}
	if (Servo5 >= 500)
	{
		I2C_SERVOSET(5, Servo5);
	}
	if (Servo6 > 500)
	{
		I2C_SERVOSET(6, Servo6);
	}
	if (Servo7 >= 500)
	{
		I2C_SERVOSET(7, Servo7);
	}
	if (Servo8 >= 500)
	{
		I2C_SERVOSET(8, Servo8);
	}
	if (Servo9 >= 500)
	{
		I2C_SERVOSET(9, Servo9);
	}
	if (Servo10 >= 500)
	{
		I2C_SERVOSET(10, Servo10);
	}
	if (Servo11 >= 500)
	{
		I2C_SERVOSET(11, Servo11);
	}
	if (Servo12 >= 500)
	{
		I2C_SERVOSET(12, Servo12);
	}
	if (Servo13 >= 500)
	{
		I2C_SERVOSET(13, Servo13);
	}
	if (Servo14 >= 500)
	{
		I2C_SERVOSET(14, Servo14);
	}
	if (Servo15 >= 500)
	{
		I2C_SERVOSET(15, Servo15);
	}
	if (Servo16 >= 500)
	{
		I2C_SERVOSET(16, Servo16);
	}
	if (Servo17 >= 500)
	{
		I2C_SERVOSET(17, Servo17);
	}
	if (Servo18 >= 500)
	{
		I2C_SERVOSET(18, Servo18);
	}
	while (!I2C_SERVOEND())
	{
		delay(1);
	}
	LEDToggle();
}
void ServoSetAllPD(unsigned int Servo1, unsigned int Servo2, unsigned int Servo3, unsigned int Servo4, unsigned int Servo5, unsigned int Servo6, unsigned int Servo7, unsigned int Servo8, unsigned int Servo9, unsigned int Servo10, unsigned int Servo11, unsigned int Servo12, unsigned int Servo13, unsigned int Servo14, unsigned int Servo15, unsigned int Servo16, unsigned int Servo17, unsigned int Servo18)
{
	if (Servo1 >= 500)
	{
		setServoPosition(1, Servo1);
	}
	if (Servo2 >= 500)
	{
		setServoPosition(2, Servo2);
	}
	if (Servo3 >= 500)
	{
		setServoPosition(3, Servo3);
	}
	if (Servo4 >= 500)
	{
		setServoPosition(4, Servo4);
	}
	if (Servo5 >= 500)
	{
		setServoPosition(5, Servo5);
	}
	if (Servo6 > 500)
	{
		setServoPosition(6, Servo6);
	}
	if (Servo7 >= 500)
	{
		setServoPosition(7, Servo7);
	}
	if (Servo8 >= 500)
	{
		setServoPosition(8, Servo8);
	}
	if (Servo9 >= 500)
	{
		setServoPosition(9, Servo9);
	}
	if (Servo10 >= 500)
	{
		setServoPosition(10, Servo10);
	}
	if (Servo11 >= 500)
	{
		setServoPosition(11, Servo11);
	}
	if (Servo12 >= 500)
	{
		setServoPosition(12, Servo12);
	}
	if (Servo13 >= 500)
	{
		setServoPosition(13, Servo13);
	}
	if (Servo14 >= 500)
	{
		setServoPosition(14, Servo14);
	}
	if (Servo15 >= 500)
	{
		setServoPosition(15, Servo15);
	}
	if (Servo16 >= 500)
	{
		setServoPosition(16, Servo16);
	}
	if (Servo17 >= 500)
	{
		setServoPosition(17, Servo17);
	}
	if (Servo18 >= 500)
	{
		setServoPosition(18, Servo18);
	}
	while (!I2C_SERVOEND())
	{
		delay(1);
	}
	LEDToggle();
}

void LEDToggle(void)
{
	if (LEDState == 0)
		LEDState = 1;
	else
		LEDState = 0;
	digitalWrite(LED, LEDState);
}

void UserCode(void)
{
	int i = 0;
	Serial.println("Robo arm ready!");
	char state = Serial.readStringUntil("\n").charAt(0);
	Serial.println(state);
	String n = Serial.readStringUntil("\n");
	String n2 = Serial.readStringUntil("\n");
	String n3 = Serial.readStringUntil("\n");
	accel_x=n.toFloat();
	accel_y=n2.toFloat();
	accel_z=n3.toFloat();
  
	for (int f = 0; f < 12; f++)
	{
		LEDToggle();
		delay(1000);
	}

	if (state == '1')
	{
		//------------------------------Configuration------------------------------

		I2C_SERVOMAX(1, 2500);
		I2C_SERVOMAX(2, 2500);
		I2C_SERVOMAX(3, 2500);
		I2C_SERVOMAX(4, 2500);
		I2C_SERVOMAX(5, 2500);
		I2C_SERVOMAX(6, 2500);
		I2C_SERVOMAX(7, 2500);
		I2C_SERVOMAX(8, 2500);
		I2C_SERVOMAX(9, 2500);
		I2C_SERVOMAX(10, 2500);
		I2C_SERVOMAX(11, 2500);
		I2C_SERVOMAX(12, 2500);
		I2C_SERVOMAX(13, 2500);
		I2C_SERVOMAX(14, 2500);
		I2C_SERVOMAX(15, 2500);
		I2C_SERVOMAX(16, 2500);
		I2C_SERVOMAX(17, 2500);
		I2C_SERVOMAX(18, 2500); // Maximum Values

		I2C_SERVOMIN(1, 500);
		I2C_SERVOMIN(2, 500);
		I2C_SERVOMIN(3, 500);
		I2C_SERVOMIN(4, 500);
		I2C_SERVOMIN(5, 500);
		I2C_SERVOMIN(6, 500);
		I2C_SERVOMIN(7, 500);
		I2C_SERVOMIN(8, 500);
		I2C_SERVOMIN(9, 500);
		I2C_SERVOMIN(10, 500);
		I2C_SERVOMIN(11, 500);
		I2C_SERVOMIN(12, 500);
		I2C_SERVOMIN(13, 500);
		I2C_SERVOMIN(14, 500);
		I2C_SERVOMIN(15, 500);
		I2C_SERVOMIN(16, 500);
		I2C_SERVOMIN(17, 500);
		I2C_SERVOMIN(18, 500); // Minimum Values

		I2C_SERVOOFFSET(1, 1500);
		I2C_SERVOOFFSET(2, 1500);
		I2C_SERVOOFFSET(3, 1442);
		I2C_SERVOOFFSET(4, 1351);
		I2C_SERVOOFFSET(5, 1500);
		I2C_SERVOOFFSET(6, 1500);
		I2C_SERVOOFFSET(7, 1500);
		I2C_SERVOOFFSET(8, 1500);
		I2C_SERVOOFFSET(9, 1500);
		I2C_SERVOOFFSET(10, 1500);
		I2C_SERVOOFFSET(11, 1500);
		I2C_SERVOOFFSET(12, 1500);
		I2C_SERVOOFFSET(13, 1500);
		I2C_SERVOOFFSET(14, 1500);
		I2C_SERVOOFFSET(15, 1500);
		I2C_SERVOOFFSET(16, 1500);
		I2C_SERVOOFFSET(17, 1500);
		I2C_SERVOOFFSET(18, 1500); // Offset Values

		I2C_SERVOREVERSE(1, 0);
		I2C_SERVOREVERSE(2, 0);
		I2C_SERVOREVERSE(3, 0);
		I2C_SERVOREVERSE(4, 0);
		I2C_SERVOREVERSE(5, 0);
		I2C_SERVOREVERSE(6, 0);
		I2C_SERVOREVERSE(7, 0);
		I2C_SERVOREVERSE(8, 0);
		I2C_SERVOREVERSE(9, 0);
		I2C_SERVOREVERSE(10, 0);
		I2C_SERVOREVERSE(11, 0);
		I2C_SERVOREVERSE(12, 0);
		I2C_SERVOREVERSE(13, 0);
		I2C_SERVOREVERSE(14, 0);
		I2C_SERVOREVERSE(15, 0);
		I2C_SERVOREVERSE(16, 0);
		I2C_SERVOREVERSE(17, 0);
		I2C_SERVOREVERSE(18, 0); // Directions (Servo Reverse)

		//------------------------------Code Flow------------------------------

		I2C_SERVOSPEED(85);																										 // Line # 0
		ServoSetAll(2500, 1674, 1817, 1500, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 0
		delay(300);																												 // Default Delay
		ServoSetAll(2500, 1674, 1817, 2045, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 1
		delay(300);																												 // Default Delay
		ServoSetAll(2500, 1674, 1817, 2045, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 2
		delay(300);																												 // Default Delay
		ServoSetAll(2500, 1674, 1400, 2045, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 3
		delay(300);																												 // Default Delay
		ServoSetAll(500, 1674, 1800, 2045, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 4
		delay(300);																												 // Default Delay
		ServoSetAll(500, 1674, 1800, 2045, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 5
		delay(300);																												 // Default Delay
		ServoSetAll(500, 1674, 1400, 2045, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 6
		delay(300);																												 // Default Delay
		ServoSetAll(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 7
		delay(300);																												 // Default Delay
		ServoSetAll(500, 1674, 1400, 2045, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 8
		delay(300);																												 // Default Delay
		ServoSetAll(500, 1674, 1800, 2045, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 9
		delay(300);																												 // Default Delay
		ServoSetAll(500, 1674, 1800, 2045, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 10
		delay(300);																												 // Default Delay
		ServoSetAll(2500, 1674, 1400, 2045, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 11
		delay(300);																												 // Default Delay
		ServoSetAll(2500, 1674, 1817, 2045, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 12
		delay(300);																												 // Default Delay
		ServoSetAll(2500, 1674, 1817, 2045, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 13
		delay(300);																												 // Default Delay
		ServoSetAll(2500, 1674, 1817, 1500, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);	 // Line # 14
		delay(300);																												 // Default Delay
		ServoSetAll(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 15
		delay(300);																												 // Default Delay
		delay(1000);
		// Comment or remove the next 5 lines to run code in loop...

		Serial.println("Task Ended :(");
	}
	else if (state == '2')
	{
		I2C_SERVOMAX(1, 2500);
		I2C_SERVOMAX(2, 2500);
		I2C_SERVOMAX(3, 2500);
		I2C_SERVOMAX(4, 2500);
		I2C_SERVOMAX(5, 2500);
		I2C_SERVOMAX(6, 2500);
		I2C_SERVOMAX(7, 2500);
		I2C_SERVOMAX(8, 2500);
		I2C_SERVOMAX(9, 2500);
		I2C_SERVOMAX(10, 2500);
		I2C_SERVOMAX(11, 2500);
		I2C_SERVOMAX(12, 2500);
		I2C_SERVOMAX(13, 2500);
		I2C_SERVOMAX(14, 2500);
		I2C_SERVOMAX(15, 2500);
		I2C_SERVOMAX(16, 2500);
		I2C_SERVOMAX(17, 2500);
		I2C_SERVOMAX(18, 2500); // Maximum Values

		I2C_SERVOMIN(1, 500);
		I2C_SERVOMIN(2, 500);
		I2C_SERVOMIN(3, 500);
		I2C_SERVOMIN(4, 500);
		I2C_SERVOMIN(5, 500);
		I2C_SERVOMIN(6, 500);
		I2C_SERVOMIN(7, 500);
		I2C_SERVOMIN(8, 500);
		I2C_SERVOMIN(9, 500);
		I2C_SERVOMIN(10, 500);
		I2C_SERVOMIN(11, 500);
		I2C_SERVOMIN(12, 500);
		I2C_SERVOMIN(13, 500);
		I2C_SERVOMIN(14, 500);
		I2C_SERVOMIN(15, 500);
		I2C_SERVOMIN(16, 500);
		I2C_SERVOMIN(17, 500);
		I2C_SERVOMIN(18, 500); // Minimum Values

		I2C_SERVOOFFSET(1, 1500);
		I2C_SERVOOFFSET(2, 1500);
		I2C_SERVOOFFSET(3, 1442);
		I2C_SERVOOFFSET(4, 1351);
		I2C_SERVOOFFSET(5, 1500);
		I2C_SERVOOFFSET(6, 1500);
		I2C_SERVOOFFSET(7, 1500);
		I2C_SERVOOFFSET(8, 1500);
		I2C_SERVOOFFSET(9, 1500);
		I2C_SERVOOFFSET(10, 1500);
		I2C_SERVOOFFSET(11, 1500);
		I2C_SERVOOFFSET(12, 1500);
		I2C_SERVOOFFSET(13, 1500);
		I2C_SERVOOFFSET(14, 1500);
		I2C_SERVOOFFSET(15, 1500);
		I2C_SERVOOFFSET(16, 1500);
		I2C_SERVOOFFSET(17, 1500);
		I2C_SERVOOFFSET(18, 1500); // Offset Values

		I2C_SERVOREVERSE(1, 0);
		I2C_SERVOREVERSE(2, 0);
		I2C_SERVOREVERSE(3, 0);
		I2C_SERVOREVERSE(4, 0);
		I2C_SERVOREVERSE(5, 0);
		I2C_SERVOREVERSE(6, 0);
		I2C_SERVOREVERSE(7, 0);
		I2C_SERVOREVERSE(8, 0);
		I2C_SERVOREVERSE(9, 0);
		I2C_SERVOREVERSE(10, 0);
		I2C_SERVOREVERSE(11, 0);
		I2C_SERVOREVERSE(12, 0);
		I2C_SERVOREVERSE(13, 0);
		I2C_SERVOREVERSE(14, 0);
		I2C_SERVOREVERSE(15, 0);
		I2C_SERVOREVERSE(16, 0);
		I2C_SERVOREVERSE(17, 0);
		I2C_SERVOREVERSE(18, 0); // Directions (Servo Reverse)

		//------------------------------Code Flow------------------------------

		I2C_SERVOSPEED(85);																										   // Line # 0
		ServoSetAllPD(1500, 1400, 1600, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 1
		delay(300);																												   // Default Delay
		ServoSetAllPD(2500, 1400, 1917, 1200, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);  // Line # 2
		delay(300);																												   // Default Delay
		ServoSetAllPD(2500, 1400, 1917, 1550, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);  // Line # 3
		delay(300);																												   // Default Delay
		ServoSetAllPD(2500, 1400, 1917, 1550, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 4
		delay(300);																												   // Default Delay
		ServoSetAllPD(2500, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 5
		delay(300);																												   // Default Delay
		ServoSetAllPD(2000, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 5

		ServoSetAllPD(1500, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 5

		ServoSetAllPD(2000, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 5

		ServoSetAllPD(1000, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);

		ServoSetAllPD(500, 1400, 1905, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 6
		delay(300);																												  // Default Delay
		ServoSetAllPD(500, 1400, 1905, 1550, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 7
		delay(300);
		ServoSetAllPD(500, 1400, 1905, 1550, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);   // Line # 7
		delay(300);																												   // Default Delay
		ServoSetAllPD(500, 1400, 1834, 1200, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);   // Line # 8
		delay(300);																												   // Default Delay
		ServoSetAllPD(1500, 1400, 1600, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 9
		delay(300);																												   // Default Delay
		ServoSetAllPD(500, 1400, 1834, 1200, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);   // Line # 10
		delay(300);																												   // Default Delay
		ServoSetAllPD(500, 1400, 1905, 1550, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);   // Line # 11
		delay(300);																												   // Defaul Delay
		ServoSetAllPD(500, 1400, 1905, 1550, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);  // Line # 12
		delay(300);																												   // Default Delay
		ServoSetAllPD(500, 1400, 1905, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);  // Line # 12
		delay(300);
		ServoSetAllPD(2000, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);

		ServoSetAllPD(1000, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);

		ServoSetAllPD(2500, 1400, 1917, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 13
		delay(300);																												   // Default Delay
		ServoSetAllPD(2500, 1400, 1917, 1550, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 14
		delay(300);																												   // Default Delay
		ServoSetAllPD(2500, 1400, 1917, 1550, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);  // Line # 15
		delay(300);																												   // Default Delay
		ServoSetAllPD(2500, 1400, 1917, 1200, 500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);  // Line # 16
		delay(300);																												   // Default Delay
		ServoSetAllPD(1500, 1400, 1600, 1200, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500); // Line # 17
		delay(300);																												   // Default Delay
		delay(1000);
	}
	delay(3000);
	while (1)
	{
		Serial.println("Task Ended :(");
	}
}

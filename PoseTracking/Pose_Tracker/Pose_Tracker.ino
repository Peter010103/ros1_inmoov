#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
//LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

unsigned char inputBuffer[9]; // int16 is 2 bytes, 10 integers + '\n'
int servo_values[8];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(5);
  // set up the LCD's number of columns and rows:
  //lcd.begin(16, 2);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() 
{
  if (Serial.available())
  {
      Serial.readBytesUntil(255, inputBuffer, 11);
  
      for (int i=0; i<10; i++)
      {
        servo_values[i] = inputBuffer[i];
        //Serial.print(servo_values[i]);
        //Serial.print("\t");
        servo_values[i] = map(servo_values[i], 0, 180, 1000, 2000);
        pwm.writeMicroseconds(i, servo_values[i]);
  
        if (i == 4)
        {
          //Serial.println();
        }
     }
  }
}

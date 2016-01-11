| Prev:  [Connecting the motors to an Arduino](MotorsToArduino.md)| [-Top-](ProjectOverview.md) | Next:  [Connecting the arduino to ROS](ArduinoRos.md) |
|:----------------------------------------------------------------|:----------------------------|:------------------------------------------------------|

# First Arduino Code #

The next step is to get the Arduino to control the motors.  The arduino page has some good documentation for getting started: [Arduino Getting Started](http://arduino.cc/en/Guide/HomePage).

I have been using Eclipse with the Arduino plugin.  It's not necessary, but Eclipse is a nice IDE, and makes coding a bit easier.  Directions for the plugin can be found here: http://www.baeyens.it/eclipse/Install.html.  A few things I had to do to get this to work.

The code I used for testing is below.  You can get this code with the following command (but it may be easier just to paste the code listing into your editor).

```
git clone -b self_test https://code.google.com/p/knex-arduino/
```

This code tests each of the wheels in each of the directions with a 1 second pause between each.  It also drives pin 13 high at the beginning of each loop to blink the LED so you know what should be happening.



Next: [Arduinio and ROS](ArduinoRos.md)

```

#include "knex_arduino.h"

#define P_RFWD A2
#define P_RREV 4
#define P_RENA 6

#define P_LFWD A0
#define P_LREV 7
#define P_LENA 5

	  // FWD REV ENA
	  // L   L   L    coast
	  // L   L   H    coast
	  // H   L   L    fwd
	  // H   L   H    coast (for pwm)
	  // L   H   L    reverse
	  // L   H   H    coast (for pwm)
	  // H   H   L    brake`

void lfwd(int speed=255) {
	digitalWrite( P_LFWD, HIGH );
	digitalWrite( P_LREV, LOW );
	analogWrite( P_LENA, constrain( 255-speed, 0, 255 ) );
}

void lrev(int speed=255) {
	digitalWrite( P_LFWD, LOW );
	digitalWrite( P_LREV, HIGH );
	analogWrite( P_LENA, constrain( 255-speed, 0, 255 ) );
}

void lcoast() {
	digitalWrite( P_LFWD, LOW );
	digitalWrite( P_LREV, LOW );
	digitalWrite( P_LENA, LOW );
}

void lbrake() {
	digitalWrite( P_LFWD, HIGH );
	digitalWrite( P_LREV, HIGH );
	digitalWrite( P_LENA, LOW);
}

void rfwd( int speed=255) {
	digitalWrite( P_RFWD, HIGH );
	digitalWrite( P_RREV, LOW );
	analogWrite( P_RENA, constrain( 255-speed, 0, 255 ) );
}

void rrev(int speed=255) {
	digitalWrite( P_RFWD, LOW );
	digitalWrite( P_RREV, HIGH );
	analogWrite( P_RENA, constrain( 255-speed, 0, 255 ) );
}


void rcoast() {
	digitalWrite( P_RFWD, LOW );
	digitalWrite( P_RREV, LOW );
	digitalWrite( P_RENA, LOW );
}

void rbrake() {
	digitalWrite( P_RFWD, HIGH );
	digitalWrite( P_RREV, HIGH );
	digitalWrite( P_RENA, LOW);
}

void setup()
{
	  pinMode(P_LFWD, OUTPUT);
	  pinMode(P_LREV, OUTPUT);
	  pinMode(P_LENA, OUTPUT);

	  pinMode(P_RFWD, OUTPUT);
	  pinMode(P_RREV, OUTPUT);
	  pinMode(P_RENA, OUTPUT);
}



// The loop function is called in an endless loop
void loop()
{
	digitalWrite( 13, HIGH );
	lfwd();	delay(1000);

	digitalWrite( 13, LOW );
	lbrake();
	delay(1000);

	lrev();
	delay(1000);

	lbrake();
	delay(1000);

	rfwd();
	delay(1000);

	rbrake();
	delay(1000);

	rrev();
	delay(1000);

	rbrake();
	delay(1000);

}

```

Next:  [Connecting the arduino to ROS](ArduinoRos.md)
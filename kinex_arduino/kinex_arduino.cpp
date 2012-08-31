// Do not remove the include below
#include "kinex_arduino.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <Time.h>

#define P_LFWD 3
#define P_LREV 4
#define P_LENA 5

#define P_RFWD 6
#define P_RREV 7
#define P_RENA 8

#define LOOP_DLY 5  // in msec

ros::NodeHandle nh;

std_msgs::String msg_debug;
ros::Publisher debug_pub("arduino_debug", &msg_debug);

char debug_str[80] = "blank";
int tick_no = 0;
int ticks_since_beat = 0;


//The setup function is called once at startup of the sketch

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
	analogWrite( P_LENA, constrain( 255-speed, 0, 255 ) );
}

void rrev(int speed=255) {
	digitalWrite( P_RFWD, LOW );
	digitalWrite( P_RREV, HIGH );
	analogWrite( P_LENA, constrain( 255-speed, 0, 255 ) );
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

//////////////////////////////////////////////////////////////////////
void RMotorCallBack( const std_msgs::Int16& motor_msg) {
//////////////////////////////////////////////////////////////////////
	sprintf(debug_str, "LMotorCallback %d", motor_msg.data);
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

    if (motor_msg.data > 255 || motor_msg.data < -255) {
    	rbrake();
    } else if (motor_msg.data == 0) {
    	rcoast();
    } else if (motor_msg.data < 0) {
    	rrev(motor_msg.data);
    } else {
    	rfwd(motor_msg.data);
    }
}

//////////////////////////////////////////////////////////////////////
void LMotorCallBack( const std_msgs::Int16& motor_msg) {
//////////////////////////////////////////////////////////////////////
	sprintf(debug_str, "LMotorCallback %d", motor_msg.data);
	msg_debug.data = debug_str;
	debug_pub.publish( &msg_debug );

    if (motor_msg.data > 255 || motor_msg.data < -255) {
    	lbrake();
    } else if (motor_msg.data == 0) {
    	lcoast();
    } else if (motor_msg.data < 0) {
    	lrev(motor_msg.data);
    } else {
    	lfwd(motor_msg.data);
    }
}

//////////////////////////////////////////////////////////////////////
// callbacks
//////////////////////////////////////////////////////////////////////

ros::Subscriber<std_msgs::Int16> rmotor_sub("rmotor_cmd", &RMotorCallBack);
ros::Subscriber<std_msgs::Int16> lmotor_sub("lmotor_cmd", &LMotorCallBack);

void setup()
{
	nh.initNode();
	nh.advertise(debug_pub);
	nh.subscribe(rmotor_sub);
	nh.subscribe(lmotor_sub);

	  pinMode(13, OUTPUT);
	  pinMode(P_LFWD, OUTPUT);
	  pinMode(P_LREV, OUTPUT);
	  pinMode(P_LENA, OUTPUT);

	  pinMode(P_RFWD, OUTPUT);
	  pinMode(P_RREV, OUTPUT);
	  pinMode(P_RENA, OUTPUT);

	  //
	  // FWD REV ENA
	  // L   L   L    coast
	  // L   L   H    coast
	  // H   L   L    fwd
	  // H   L   H    coast (for pwm)
	  // L   H   L    reverse
	  // L   H   H    coast (for pwm)
	  // H   H   L    brake`
}



// The loop function is called in an endless loop
void loop()
{
	nh.spinOnce();
	ticks_since_beat++;

	if(ticks_since_beat > (1000 / LOOP_DLY) ) {
    	tick_no++;
    	sprintf(debug_str, "tick %d", tick_no);
    	msg_debug.data = debug_str;
    	debug_pub.publish( &msg_debug );
    	ticks_since_beat = 0;
    	digitalWrite(13, HIGH - digitalRead(13)); // toggle the LED
	}

    delay(LOOP_DLY);
}

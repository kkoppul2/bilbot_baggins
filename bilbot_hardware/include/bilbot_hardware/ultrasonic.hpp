#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "math.h"
#include <pigpiod_if2.h>
#include <stdint.h>
#include <stdlib.h>

#define SRTE_GOOD      0
#define SRTE_BAD_RANGE 1
#define SRTE_TIMEOUT   2

namespace bilbot_hardware {

class ultrasonic
{

private:
	float min_dist_ = 0.02;
	float max_dist_ = 4.5;

	int pi_, trig_, echo_;

	float seconds_;

	//callback ids
	int callback_trig_id_, callback_echo_id_;

	//status variables
	int _in_code, _ready, _new_reading, _got_echo, _got_trig;

	//timing variables
	uint32_t _echo_tick, _trig_tick;

	//data variables
	int status_;
	int round_trip_micros_;
	float range_;
	double timestamp_;
	double tov_;

	void _callback(unsigned gpio, unsigned level, uint32_t tick);

	static void _callbackEx(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

	void read();


public:
	ultrasonic(int pi, int gpioTrig, int gpioEcho);
	~ultrasonic();

	float getRange();
	
};

}

#endif
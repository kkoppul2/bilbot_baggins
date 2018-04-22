#include "bilbot_hardware/ultrasonic.hpp"
#include <pigpiod_if2.h>

namespace bilbot_hardware {

void ultrasonic::_callback(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
	int round_trip_micros;
  	float range_m;
  	int tick_diff;

	if (gpio == trig_) {
    	_got_trig = 1;
    	_trig_tick = tick;
    	_got_echo = 0;
	}
	else if (level == 1) {
		if (_got_trig) {
			tick_diff = tick - self->_trig_tick;

			if (tick_diff > 100) {
				_got_echo = 1;
				_echo_tick = tick;
			}
		}
	}
	else if (level == 0)
	{
		if (_got_echo)
		{
			_got_echo = 0;
			_got_trig = 0;

			round_trip_micros = tick - _echo_tick;
			range_m = round_trip_micros * 0.00017015;

			if ((min_dist_ <= range_m) && (range_m <= max_dist_))
			{
            	tov_ = timestamp;
            	range_ = range_m;
            	round_trip_micros_ = round_trip_micros;
            	status_ = SRTE_GOOD;
			}
			else {
            	status_ = SRTE_BAD_RANGE;
			}

			_ready = 1;
			_new_reading = 1;
		}
	}
}

ultrasonic::ultrasonic(int pi, int gpioTrig, int gpioEcho)
	: pi_(pi), trig_(gpioTrig), echo_(gpioEcho)
{
	set_mode(pi_, trig_, PI_OUTPUT);
	set_mode(pi_, echo_, PI_INPUT);

	callback_trig_id_ = callback_ex(pi_, trig_, FALLING_EDGE, _callback, this);
	callback_echo_id_ = callback_ex(pi_, echo_, EITHER_EDGE, _callback, this);
}

ultrasonic::~ultrasonic() {
	callback_cancel(callback_trig_id_);
	callback_cancel(callback_echo_id_);
}

void ultrasonic::read() {
	_new_reading = 0;

	gpio_trigger(pi_, trig_, 11, 1);

	timestamp_ = time_time();

	for (int i=0; i<25; i++) {
		time_sleep(0.01);
		if (_new_reading) break;
	}

	if (!_new_reading) {
		status_ = SRTE_TIMEOUT;
		_ready = 1;
   }

}

float ultrasonic::getRange() {

	float num_readings = 5;
	float readings[num_readings];
	for (int i = 0; i < num_readings; ++i) {
		read();
		readings[i] = range_;
	}
	float sum = 0;
	for (int i = 0; i < num_readings; ++i) {
	 	sum += readings[i];
	}
	return sum/num_readings;
}

}
#include <pigpiod_if2.h>

#include "bilbot_hardware/rotary_encoder.hpp"

/*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1

*/

namespace bilbot_hardware {

void re_decoder::_pulse(int gpio, int level, uint32_t tick)
{
   if (!(gpio == lastGpio && level == lastLevel)) {
	if (gpio == mygpioA) levA = level; else levB = level;
	lastGpio = gpio;
	lastLevel = level;
	if (levA && gpio == mygpioB) {
		level ? setPosition(1*resolution_) : setPosition(-1*resolution_);
	} else if (!levA && gpio == mygpioB) {
		level ? setPosition(-1*resolution_) : setPosition(1*resolution_);
	} else if (levB && gpio == mygpioA) {
		level ? setPosition(-1*resolution_) : setPosition(1*resolution_);
	} else if (!levB && gpio == mygpioA) {
		level ? setPosition(1*resolution_) : setPosition(-1*resolution_);
	}
   }
}

void re_decoder::_pulseEx(int gpio, int level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   re_decoder *mySelf = (re_decoder *) user;

   mySelf->_pulse(gpio, level, tick); /* Call the instance callback. */
}

re_decoder::re_decoder(int pi, int gpioA, int gpioB)
  : position_(0.0), position_old_(0.0), velocity_(0.0), velocity_old1_(0.0), velocity_old2_(0.0)
{
   mygpioA = gpioA;
   mygpioB = gpioB;

   levA=0;
   levB=0;

   lastGpio = -1;
   lastLevel = -1;

   set_mode(pi, gpioA, PI_INPUT);
   set_mode(pi, gpioB, PI_INPUT);

   /* pull up is needed as encoder common is grounded */

   set_pull_up_down(pi, gpioA, PI_PUD_UP);
   set_pull_up_down(pi, gpioB, PI_PUD_UP);

   /* monitor encoder level changes */

   callback_a_id = callback_ex(pi, gpioA, EITHER_EDGE, _pulseEx, &this);
   callback_b_id = callback_ex(pi, gpioB, EITHER_EDGE, _pulseEx, &this);
}

void re_decoder::re_cancel(void)
{
   callback_cancel(callback_a_id);
   callback_cancel(callback_b_id);
}

float re_decoder::getPosition() {
  return position_;

}

float re_decoder::getVelocity() {
  return velocity_;
}

void re_decoder::filter_velocity() {
  //IIR filter on velocity
    velocity_ = (position_ - position_old_)/0.01;
    velocity_ = (velocity_ + velocity_old1_ + velocity_old2_)/3.0;

    position_old_ = position_;
    velocity_old2_ = velocity_old1_;
    velocity_old1_ = velocity_;
}

void re_decoder::setPosition(float way) {
  position_ += way;
}

}
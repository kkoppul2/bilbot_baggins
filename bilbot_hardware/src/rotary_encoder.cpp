#include <pigpio.h>

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
		level ? (mycallback)(1*resolution) : (mycallback)(-1*resolution);
	} else if (!levA && gpio == mygpioB) {
		level ? (mycallback)(-1*resolution) : (mycallback)(1*resolution);
	} else if (levB && gpio == mygpioA) {
		level ? (mycallback)(-1*resolution) : (mycallback)(1*resolution);
	} else if (!levB && gpio == mygpioA) {
		level ? (mycallback)(1*resolution) : (mycallback)(-1*resolution);
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

re_decoder::re_decoder(int gpioA, int gpioB, re_decoderCB_t callback)
  : position_(0.0), position_old_(0.0), velocity_(0.0), velocity_old1_(0.0), velocity_old2_(0.0)
{
   mygpioA = gpioA;
   mygpioB = gpioB;

   mycallback = callback;

   levA=0;
   levB=0;

   lastGpio = -1;
   lastLevel = -1;

   gpioSetMode(gpioA, PI_INPUT);
   gpioSetMode(gpioB, PI_INPUT);

   /* pull up is needed as encoder common is grounded */

   gpioSetPullUpDown(gpioA, PI_PUD_UP);
   gpioSetPullUpDown(gpioB, PI_PUD_UP);

   /* monitor encoder level changes */

   gpioSetAlertFuncEx(gpioA, _pulseEx, this);
   gpioSetAlertFuncEx(gpioB, _pulseEx, this);
}

void re_decoder::re_cancel(void)
{
   gpioSetAlertFuncEx(mygpioA, 0, this);
   gpioSetAlertFuncEx(mygpioB, 0, this);
}

void re_decoder::positionCallback(float way) {
  position_ += way;
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

}
#ifndef ROTARY_ENCODER_HPP
#define ROTARY_ENCODER_HPP

#include <stdint.h>
#include <pigpiod_if2.h>

namespace bilbot_hardware {

class re_decoder
{
   int mygpioA, mygpioB, levA, levB, lastGpio, lastLevel;

   float resolution_ = 0.0027939424; //Angular resolution per encoder tick

   float position_, position_old_, velocity_, velocity_old1_, velocity_old2_;

   int callback_a_id, callback_b_id;

   void _pulse(unsigned gpio, unsigned level, uint32_t tick);

   /* Need a static callback to link with C. */
   static void _pulseEx(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

   void filter_velocity();

   void setPosition(float way);

   public:

   re_decoder(int pi, int gpioA, int gpioB);
   /*
      This function establishes a rotary encoder on gpioA and gpioB.

      When the encoder is turned the callback function is called.
   */

   void re_cancel(void);
   /*
      This function releases the resources used by the decoder.
   */

   float getPosition();

   float getVelocity();


};

}

#endif

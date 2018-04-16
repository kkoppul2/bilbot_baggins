#ifndef ROTARY_ENCODER_HPP
#define ROTARY_ENCODER_HPP

#include <stdint.h>

namespace bilbot_hardware {

class re_decoder
{
   int mygpioA, mygpioB, levA, levB, lastGpio, lastLevel;

   float resolution_ = 0.1308997; //Angular resolution per encoder tick

   float position_, position_old_, velocity_, velocity_old1_, velocity_old2_;

   void _pulse(int gpio, int level, uint32_t tick);

   /* Need a static callback to link with C. */
   static void _pulseEx(int gpio, int level, uint32_t tick, void *user);

   void filter_velocity();

   void setPosition(float way);

   public:

   re_decoder(int gpioA, int gpioB);
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

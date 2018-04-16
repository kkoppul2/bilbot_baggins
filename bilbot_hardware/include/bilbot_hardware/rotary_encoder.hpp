#ifndef ROTARY_ENCODER_HPP
#define ROTARY_ENCODER_HPP

#include <stdint.h>

namespace bilbot_hardware {

typedef void (*re_decoderCB_t)(float);

class re_decoder
{
   int mygpioA, mygpioB, levA, levB, lastGpio, lastLevel;

   float resolution = 0.1308997; //Angular resolution per encoder tick

   float position_, position_old_, velocity_, velocity_old1_, velocity_old2_;

   re_decoderCB_t mycallback;

   void _pulse(int gpio, int level, uint32_t tick);

   /* Need a static callback to link with C. */
   static void _pulseEx(int gpio, int level, uint32_t tick, void *user);

   void filter_velocity();

   public:

   void positionCallback(float way);

   re_decoder(int gpioA, int gpioB, re_decoderCB_t callback);
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

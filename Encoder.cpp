#include "Encoder.hpp"

#ifdef ARDUINO_AVR_NANO
mtrn3100::Encoder* instances[CORE_NUM_INTERRUPTS];

#endif

#ifdef ARDUINO_ARCH_STM32
  #include "PinNames.h"
#endif

namespace mtrn3100 {

Encoder::Encoder() {}

#ifdef ARDUINO_ARCH_STM32
Encoder::Encoder(TIM_TypeDef* timer) : ovf_count_(0), timer_(timer) {
  // setup
  this->setup();

  this->timer_.resume();
}

void Encoder::setup() {
  // instead of using External Interrupt, we use Encoder Mode provided by the
  // MCU
  this->timer_.setPreloadEnable(true);
  this->timer_.setOverflow(65536 - 1, TICK_FORMAT);
  if (this->timer_.getHandle()->Instance == TIM1) {
    this->timer_.setMode(1, TIMER_INPUT_CAPTURE_RISING, PA_8, FILTER_CKINT_N4);
    this->timer_.setMode(2, TIMER_INPUT_CAPTURE_RISING, PA_9, FILTER_CKINT_N4);
  } else if (this->timer_.getHandle()->Instance == TIM4) {
    this->timer_.setMode(1, TIMER_INPUT_CAPTURE_RISING, PB_6, FILTER_CKINT_N4);
    this->timer_.setMode(2, TIMER_INPUT_CAPTURE_RISING, PB_7, FILTER_CKINT_N4);
  }

  LL_TIM_SetEncoderMode(this->timer_.getHandle()->Instance,
                        LL_TIM_ENCODERMODE_X4_TI12);

  this->timer_.attachInterrupt(std::bind(&Encoder::callback, this));
}

Encoder::~Encoder(void) { this->timer_.pause(); }
#endif
void Encoder::reset(void) {
#ifdef ARDUINO_ARCH_STM32
  this->ovf_count_ = 0;
  LL_TIM_SetCounter(this->timer_.getHandle()->Instance, 0U);
#endif

#ifdef ARDUINO_AVR_NANO
  this->count = 0;
#endif
}

void Encoder::setNegative(bool yes) {
#ifdef ARDUINO_ARCH_STM32
  this->timer_.pause();
  TIM_TypeDef* htim = this->timer_.getHandle()->Instance;

  LL_TIM_IC_SetPolarity(
      htim, LL_TIM_CHANNEL_CH1,
      yes ? LL_TIM_IC_POLARITY_FALLING : LL_TIM_IC_POLARITY_RISING);
  this->timer_.resume();
#endif

#ifdef ARDUINO_AVR_NANO
  this->negative = yes;
#endif
}

#ifdef ARDUINO_AVR_NANO

Encoder::Encoder(uint8_t encA, uint8_t encB)
    : encoderA_pin(encA), encoderB_pin(encB), negative(false) {
  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderB_pin, INPUT_PULLUP);
  // only map Pin A to external interrupt
  auto isr = digitalPinToInterrupt(this->encoderA_pin);
  attach_interrupt(isr, CHANGE, this);
}

Encoder::~Encoder(void) {}
#endif
}  // namespace mtrn3100

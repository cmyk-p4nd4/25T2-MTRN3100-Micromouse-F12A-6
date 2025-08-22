#include "Motor.hpp"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include "PinNames.h"
#include "PinNamesTypes.h"
#include "PortNames.h"
#include "pinmap.h"
#include "stm32f4xx_ll_gpio.h"

namespace mtrn3100 {
Motor::Motor() {}

Motor::~Motor() {}

Motor::Motor(TIM_TypeDef* htim, PinName dir_1_pin, PinName dir_2_pin)
    : dir_1_pin_(dir_1_pin),
      dir_2_pin_(dir_2_pin),
      dir_1_port_(nullptr),
      dir_2_port_(nullptr),
      htim_(htim),
      timer_(htim) {
  // setup
  this->setup();
  this->timer_.resume();
}

__STATIC_INLINE void LL_GPIO_WriteOutputPin(GPIO_TypeDef* GPIOx,
                                            uint16_t PinMask, bool logic_high) {
  WRITE_REG(GPIOx->BSRR, (PinMask << (!logic_high ? 16U : 0U)));
}

void Motor::setPWM(int pwm1,int pwm2) {
  LL_GPIO_WriteOutputPin(this->dir_1_port_, this->dir_1_bit_, pwm1 > 0);
  LL_GPIO_WriteOutputPin(this->dir_2_port_, this->dir_2_bit_, pwm2 < 0);

  pwm1 = std::max(-10000, std::min(10000, pwm1));
  pwm2 = std::max(-10000, std::min(10000, pwm2));

  if (pwm1 > 100 && pwm1 < 1500) {
    pwm1 = 1500;
  } else if (pwm1 < -100 && pwm1 > -1500) {
    pwm1 = -1500;
  } else if (std::abs(pwm1) <= 100) {
    pwm1 = 0;
  }

  if (pwm2 > 100 && pwm2 < 1500) {
    pwm2 = 1500 * 0.95;
  } else if (pwm2 < -100 && pwm2 > -1500) {
    pwm2 = -1500 * 0.95;
  } else if (std::abs(pwm2) <= 100) {
    pwm2 = 0;
  }

  this->timer_.setCaptureCompare(1, std::abs(pwm1));
  this->timer_.setCaptureCompare(2, std::abs(pwm2));
  this->timer_.refresh();
}

void Motor::getPWM(uint32_t& pwm1, uint32_t& pwm2, uint32_t& counter) {
  pwm1 = LL_TIM_OC_GetCompareCH1(this->htim_);
  pwm2 = LL_TIM_OC_GetCompareCH2(this->htim_);
  counter = LL_TIM_GetCounter(this->htim_);
}

void Motor::setup(void) {
  /* Configure Direction GPIO */
  LL_GPIO_InitTypeDef gpio_init = {0};
  this->dir_1_port_ = get_GPIO_Port(STM_PORT(dir_1_pin_));
  this->dir_2_port_ = get_GPIO_Port(STM_PORT(dir_2_pin_));

  assert((this->dir_1_port_ != __null) && (this->dir_2_port_ != __null));
  // If they belong to the same port then only 1 init is needed
  bool same_port = dir_1_port_ == dir_2_port_;

  /* Pin Configuration */
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Pull = LL_GPIO_PULL_DOWN;

  // set Pin B first
  gpio_init.Pin = STM_LL_GPIO_PIN(dir_2_pin_);
  if (!same_port) {
    LL_GPIO_Init(this->dir_2_port_, &gpio_init);
  } else {
    gpio_init.Pin |= STM_LL_GPIO_PIN(dir_1_pin_);
  }

  // set Pin A
  LL_GPIO_Init(this->dir_1_port_, &gpio_init);

  this->dir_1_bit_ = STM_LL_GPIO_PIN(dir_1_pin_);
  this->dir_2_bit_ = STM_LL_GPIO_PIN(dir_2_pin_);

  /* Configure Timer */
  this->timer_.setPreloadEnable(true);

  if (this->timer_.getHandle()->Instance == TIM9) {
    this->timer_.setPrescaleFactor(4 - 1);  // 100 MHz / (4 * 10000) = 2500 Hz
    this->timer_.setOverflow(10000 - 1, TICK_FORMAT);
    this->timer_.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA_2_ALT2);
    this->timer_.setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA_3_ALT2);
  } else if (this->timer_.getHandle()->Instance == TIM5) {
    this->timer_.setPrescaleFactor(4 - 1);  // 100 MHz / (4 * 10000) = 2500 Hz
    this->timer_.setOverflow(10000 - 1, TICK_FORMAT);
    this->timer_.setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PA_2_ALT2);
    this->timer_.setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PA_3_ALT2);
  }
}

}  // namespace mtrn3100

#pragma once
#ifdef ARDUINO_AVR_NANO
  #include <Arduino.h>
#endif

#ifdef ARDUINO_ARCH_STM32
  #include <HardwareTimer.h>
  #include "PinNames.h"
#endif

namespace mtrn3100 {

class Motor {
public:
  Motor();
#ifdef ARDUINO_AVR_NANO
  Motor(uint8_t pwm_pin, uint8_t dir_pin)
      : pwm_pin_(pwm_pin), dir_pin_(dir_pin) {
    // set both pins to output

    pinMode(this->pwm_pin_, OUTPUT);
    pinMode(this->dir_pin_, OUTPUT);
  }
#endif

#ifdef ARDUINO_ARCH_STM32
  Motor(TIM_TypeDef* htim, PinName dir_1_pin, PinName dir_2_pin);
#endif

  virtual ~Motor();

#ifdef ARDUINO_AVR_NANO
  void setPWM(int16_t pwm) {
    bool is_reverse = pwm < 0;
    uint8_t value =
        static_cast<uint8_t>(constrain(abs(pwm) & 0xFF, 0, pwm_max_));

    if (fabs(pwm) < 30) {
      analogWrite(this->pwm_pin_, 0);
    } else {
      digitalWrite(this->dir_pin_, is_reverse ? HIGH : LOW);
      analogWrite(this->pwm_pin_, value);
    }
  }
#endif

#ifdef ARDUINO_ARCH_STM32
  void setPWM(int pwm1,int pwm2);

  void getPWM(uint32_t& pwm1, uint32_t& pwm2, uint32_t& counter);
#endif

  constexpr uint32_t getPulseResolution(void);

public:
  static constexpr int FORWARD = 1;
  static constexpr int BACKWARD = -1;

protected:
#ifdef ARDUINO_ARCH_STM32
  void setup(void);

  PinName dir_1_pin_;
  PinName dir_2_pin_;
  uint32_t pwm_max_ = 10000;
#endif

#ifdef ARDUINO_AVR_NANO
  uint8_t pwm_pin_;
  uint8_t dir_pin_;
  uint32_t pwm_max_ = 255;
#endif

private:
#ifdef ARDUINO_ARCH_STM32
  GPIO_TypeDef* dir_1_port_;
  uint32_t dir_1_bit_;
  GPIO_TypeDef* dir_2_port_;
  uint32_t dir_2_bit_;
  TIM_TypeDef* htim_;
  HardwareTimer timer_;
#endif
};

}  // namespace mtrn3100

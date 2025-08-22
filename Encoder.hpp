#pragma once

#ifdef ARDUINO_AVR_NANO

  #include <Arduino.h>
  #include <wiring_private.h>
  #include <math.h>

  #if !defined(CORE_NUM_INTERRUPT)
    #define CORE_NUM_INTERRUPT 2
  #endif
#endif

#ifdef ARDUINO_ARCH_STM32
  #include <HardwareTimer.h>
  #include <cstdint>
  #include <cmath>
#endif


namespace mtrn3100 {

#ifdef ARDUINO_AVR_NANO
  #define PIN_TO_BASEREG(pin) (portInputRegister(digitalPinToPort(pin)))
  #define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
  #define PIN_READ_FAST(pin) \
    ((*(PIN_TO_BASEREG(pin))) & (PIN_TO_BITMASK(pin)) ? 1 : 0)
#endif
class Encoder {
public:
  static constexpr int HI_Z = 0;
  static constexpr int FORWARD = 1;
  static constexpr int BACKWARD = -1;

public:
  Encoder();

#ifdef ARDUINO_AVR_NANO
  Encoder(uint8_t encA, uint8_t encB);
#endif

#ifdef ARDUINO_ARCH_STM32
  Encoder(TIM_TypeDef* timer);
#endif

  virtual ~Encoder();

  /**
   * the state transition diagram for the encoder:
   *
  ┌────────┐    BACKWARD    ┌────────┐
  │   AB   │<────────────── │   AB   │
  │   00   │                │   10   │
  │        │ ──────────────>│        │
  └─┬──────┘     FORWARD    └────────┘
    │    ^                    │    ^
    │    │                    │    │
    │    │                    │    │
    │    │                    │    │
    │    │                    │    │
    │    │                    │    │
    v    │                    v    │
  ┌──────┴─┐                ┌──────┴─┐
  │   AB   │<────────────── │   AB   │
  │   01   │                │   11   │
  │        │ ──────────────>│        │
  └────────┘                └────────┘

   *
   */

#ifdef ARDUINO_AVR_NANO
  // Encoder function used to update the encoder
  void readEncoder(void) {
    noInterrupts();

    // NOTE: NO BLOCKING FUNCTION HERE SUCH AS `PRINT`

    uint8_t p1 = PIN_READ_FAST(this->encoderA_pin);
    uint8_t p2 = PIN_READ_FAST(this->encoderB_pin);
    state |= p1 ? 1 << 1 : 0;
    state |= p2 ? 1 << 0 : 0;

    switch (state) {
      case 0x2:
      case 0x1:  // __|‾‾
        if (negative) {
          this->count++;
        } else {
          this->count--;
        }
        break;
      case 0x0:
      case 0x3:  // ‾‾|__
        if (negative) {
          this->count--;
        } else {
          this->count++;
        }
        break;
    }

    interrupts();
  }

  constexpr int32_t getRawCount() const { return this->count; }
#endif

#ifdef ARDUINO_ARCH_STM32
  constexpr int64_t getRawCount() {
    int64_t count = 0;
    TIM_TypeDef* const htim = this->timer_.getHandle()->Instance;
    // fetch TIMx->CNT register
    int64_t cnt = static_cast<int64_t>(LL_TIM_GetCounter(htim));
    int64_t arr = static_cast<int64_t>(LL_TIM_GetAutoReload(htim));

    count = this->ovf_count_ * arr + cnt;
    return count;
  }
#endif

  /**
   * @brief Set this counter
   */
  void setNegative(bool yes);

  /**
   * @brief Reset Encoder Status
   *
   */
  void reset(void);

  /**
   * @brief This function returns numbers of revolution
   * since reset
   */
  constexpr float getRotation() {
    // TODO: Convert encoder count to radians
    float count_f = static_cast<float>(this->getRawCount()) * this->resolution * 2 * M_PI;
    return count_f;
  }

  constexpr inline float deg2rad(const float& deg) {
    constexpr float r1 = M_PI / 180.0f;
    return r1 * deg;
  }

  constexpr inline float rad2deg(const float& rad) {
    constexpr float d1 = 180.0f * M_1_PI;
    return d1 * rad;
  }

protected:
#ifdef ARDUINO_AVR_NANO
  const uint8_t encoderA_pin;
  const uint8_t encoderB_pin;
  volatile int32_t count = 0;
  bool negative = false;
#endif

#ifdef ARDUINO_ARCH_STM32
  volatile int32_t ovf_count_;
#endif

#ifdef ARDUINO_AVR_NANO
  const uint16_t counts_per_revolution = 1400U;
#endif
#ifdef ARDUINO_ARCH_STM32
  const uint16_t counts_per_revolution = 2800U;
#endif
  const float resolution = 1.0f / static_cast<float>(counts_per_revolution);

private:
#ifdef ARDUINO_AVR_NANO
  static void attach_interrupt(uint8_t isr, int mode, Encoder* obj) {
    instances[isr] = obj;
    switch (isr) {
      case 0:
        attachInterrupt(isr, Encoder::isr0, mode);
        break;
      case 1:
        attachInterrupt(isr, Encoder::isr1, mode);
    }
  }

  static void isr0(void) { instances[0]->readEncoder(); }
  static void isr1(void) { instances[1]->readEncoder(); }
#endif

#ifdef ARDUINO_ARCH_STM32
  static void callback(Encoder* encoder) {
    TIM_TypeDef* htim = encoder->timer_.getHandle()->Instance;

    // Here, we track the rollover count instead of the actual count
    if (LL_TIM_GetDirection(htim) == LL_TIM_COUNTERDIRECTION_UP) {
      // overflow
      encoder->ovf_count_++;
    } else if (LL_TIM_GetCounterMode(htim) == LL_TIM_COUNTERDIRECTION_DOWN) {
      // underflow
      encoder->ovf_count_--;
    }
  }

  void setup(void);

  TIM_TypeDef* instance_;
  HardwareTimer timer_;
#endif
};

#ifdef ARDUINO_AVR_NANO
extern Encoder* instances[];
  #undef CORE_NUM_INTERRUPT
  #undef PIN_TO_BASEREG
  #undef PIN_TO_BITMASK
  #undef PIN_READ_FAST

#endif

}  // namespace mtrn3100

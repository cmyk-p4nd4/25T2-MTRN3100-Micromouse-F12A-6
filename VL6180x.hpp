/**
 * @Note
 * This code is adapated and modified from the following codebase:
 * https://github.com/pololu/vl6180x-arduino/blob/master/VL6180X.h
 *
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <cstdint>

namespace mtrn3100 {

class VL6180X {
public:
  // Error Codes
  enum class ErrorCode : uint8_t {
    OK = 0,  // No error

    // System Reset required error
    SYSERR_1 = 0b0001,  // VCSEL Continuity Test failed
    SYSERR_2 = 0b0010,  // VCSEL Watchdog Test
    SYSERR_3 = 0b0011,  // VCSEL Watchdog
    SYSERR_4 = 0b0100,  // PLL1 Lock
    SYSERR_5 = 0b0101,  // PLL2 Lock

    // Runtime Error
    ECEFAIL = 0b0110,      // Early Convergence Estimate Check fail
    NOCONVERGE = 0b0111,   // Maximum Convergence Reached
    RANGEIGNORE = 0b1000,  // No Target Ignore; Ignore threshold check failed
    SNR = 0b1011,          // SNR; Ambient conditions too high
    RAW_UVF = 0b1100,      // Raw Range underflow; Target too close
    RAW_OVF = 0b1101,      // Raw Range overflow; Target too far
    RANGE_UVF = 0b1110,    // Range underflow; Target too close
    RANGE_OVF = 0b1111,    // Range overflow; Target too far
  };
  // register Address
  enum regAddr : uint16_t {
    IDENTIFICATION__MODEL_ID = 0x000,
    IDENTIFICATION__MODEL_REV_MAJOR = 0x001,
    IDENTIFICATION__MODEL_REV_MINOR = 0x002,
    IDENTIFICATION__MODULE_REV_MAJOR = 0x003,
    IDENTIFICATION__MODULE_REV_MINOR = 0x004,
    IDENTIFICATION__DATE_HI = 0x006,
    IDENTIFICATION__DATE_LO = 0x007,
    IDENTIFICATION__TIME = 0x008,  // 16-bit

    SYSTEM__MODE_GPIO0 = 0x010,
    SYSTEM__MODE_GPIO1 = 0x011,
    SYSTEM__HISTORY_CTRL = 0x012,
    SYSTEM__INTERRUPT_CONFIG_GPIO = 0x014,
    SYSTEM__INTERRUPT_CLEAR = 0x015,
    SYSTEM__FRESH_OUT_OF_RESET = 0x016,
    SYSTEM__GROUPED_PARAMETER_HOLD = 0x017,

    SYSRANGE__START = 0x018,
    SYSRANGE__THRESH_HIGH = 0x019,
    SYSRANGE__THRESH_LOW = 0x01A,
    SYSRANGE__INTERMEASUREMENT_PERIOD = 0x01B,
    SYSRANGE__MAX_CONVERGENCE_TIME = 0x01C,
    SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E,  // 16-bit
    SYSRANGE__CROSSTALK_VALID_HEIGHT = 0x021,
    SYSRANGE__EARLY_CONVERGENCE_ESTIMATE = 0x022,  // 16-bit
    SYSRANGE__PART_TO_PART_RANGE_OFFSET = 0x024,
    SYSRANGE__RANGE_IGNORE_VALID_HEIGHT = 0x025,
    SYSRANGE__RANGE_IGNORE_THRESHOLD = 0x026,  // 16-bit
    SYSRANGE__MAX_AMBIENT_LEVEL_MULT = 0x02C,
    SYSRANGE__RANGE_CHECK_ENABLES = 0x02D,
    SYSRANGE__VHV_RECALIBRATE = 0x02E,
    SYSRANGE__VHV_REPEAT_RATE = 0x031,

    SYSALS__START = 0x038,
    SYSALS__THRESH_HIGH = 0x03A,
    SYSALS__THRESH_LOW = 0x03C,
    SYSALS__INTERMEASUREMENT_PERIOD = 0x03E,
    SYSALS__ANALOGUE_GAIN = 0x03F,
    SYSALS__INTEGRATION_PERIOD = 0x040,

    RESULT__RANGE_STATUS = 0x04D,
    RESULT__ALS_STATUS = 0x04E,
    RESULT__INTERRUPT_STATUS_GPIO = 0x04F,
    RESULT__ALS_VAL = 0x050,           // 16-bit
    RESULT__HISTORY_BUFFER_0 = 0x052,  // 16-bit
    RESULT__HISTORY_BUFFER_1 = 0x054,  // 16-bit
    RESULT__HISTORY_BUFFER_2 = 0x056,  // 16-bit
    RESULT__HISTORY_BUFFER_3 = 0x058,  // 16-bit
    RESULT__HISTORY_BUFFER_4 = 0x05A,  // 16-bit
    RESULT__HISTORY_BUFFER_5 = 0x05C,  // 16-bit
    RESULT__HISTORY_BUFFER_6 = 0x05E,  // 16-bit
    RESULT__HISTORY_BUFFER_7 = 0x060,  // 16-bit
    RESULT__RANGE_VAL = 0x062,
    RESULT__RANGE_RAW = 0x064,
    RESULT__RANGE_RETURN_RATE = 0x066,             // 16-bit
    RESULT__RANGE_REFERENCE_RATE = 0x068,          // 16-bit
    RESULT__RANGE_RETURN_SIGNAL_COUNT = 0x06C,     // 32-bit
    RESULT__RANGE_REFERENCE_SIGNAL_COUNT = 0x070,  // 32-bit
    RESULT__RANGE_RETURN_AMB_COUNT = 0x074,        // 32-bit
    RESULT__RANGE_REFERENCE_AMB_COUNT = 0x078,     // 32-bit
    RESULT__RANGE_RETURN_CONV_TIME = 0x07C,        // 32-bit
    RESULT__RANGE_REFERENCE_CONV_TIME = 0x080,     // 32-bit

    RANGE_SCALER = 0x096,  // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h

    READOUT__AVERAGING_SAMPLE_PERIOD = 0x10A,
    FIRMWARE__BOOTUP = 0x119,
    FIRMWARE__RESULT_SCALER = 0x120,
    I2C_SLAVE__DEVICE_ADDRESS = 0x212,
    INTERLEAVED_MODE__ENABLE = 0x2A3,
  };

  uint8_t i2c_status;  // status of last I2C transmission

#ifdef ARDUINO_AVR_NANO
  typedef uint8_t Pin;
  #define NC UINT8_MAX
#endif
#ifdef ARDUINO_ARCH_STM32
  using Pin = PinName;
#endif

  VL6180X();
  VL6180X(TwoWire *bus, Pin GPIO0 = NC, Pin GPIO1 = NC);

  VL6180X(VL6180X &other) = delete;
  VL6180X(VL6180X &&other) = delete;

  void setBus(TwoWire *bus) { this->bus_ = bus; }
  TwoWire *getBus() { return bus_; }

  void setGPIO0(Pin GPIO_0) { this->gpio_0_ = GPIO_0; }
  void setGPIO1(Pin GPIO_1) { this->gpio_1_ = GPIO_1; }

  bool setAddress(const uint8_t &address);
  uint8_t getAddress() { return address_; }

  /**
   * @brief Initialize sensor with settings from ST application note AN4545,
   * section "SR03 settings" - "Mandatory : private registers"
   * @note This function will pull up the Chip-Enable pin automatically
   */
  void init(void);

  // Configure some settings for the sensor's default behavior from AN4545 -
  // "Recommended : Public registers" and "Optional: Public registers"
  //
  // Note that this function does not set up GPIO1 as an interrupt output as
  // suggested, though you can do so by calling:
  // writeReg(SYSTEM__MODE_GPIO1, 0x10);
  void configureDefault(void);

  uint8_t writeReg(uint16_t reg, uint8_t value, bool restart = false);
  uint8_t writeReg16Bit(uint16_t reg, uint16_t value, bool restart = false);
  uint8_t writeReg32Bit(uint16_t reg, uint32_t value, bool restart = false);

  uint8_t readReg(uint16_t reg);
  uint16_t readReg16Bit(uint16_t reg);
  uint32_t readReg32Bit(uint16_t reg);

  void setScaling(uint8_t scaling);
  inline constexpr uint8_t getScaling() const { return scaling_; }

  uint16_t getRangeSingle(unsigned timeout);
  uint16_t getAmbientSingle(unsigned timeout);

  void startRangeContinuous(uint16_t period = 100);
  void startAmbientContinuous(uint16_t period = 500);
  void startInterleavedContinuous(uint16_t period = 500);
  void stopContinuous();

  bool isRangeSampleReady(void);
  bool isAmbientSampleReady(void);

  uint16_t getRange(void);
  uint16_t getAmbient(void);

  inline void setTimeout(uint16_t timeout) { io_timeout_ = timeout; }
  inline uint16_t getTimeout() { return io_timeout_; }
  bool isTimeout(void);

  ErrorCode getRangeStatus(void) { return this->error_status; }

protected:
  Pin gpio_0_;
  Pin gpio_1_;
  uint8_t address_;
  uint8_t scaling_;
  int8_t ptp_offset_;
  uint16_t io_timeout_;
  bool timeout_;
  ErrorCode error_status = ErrorCode::OK;

private:
  TwoWire *bus_;
};

}  // namespace mtrn3100

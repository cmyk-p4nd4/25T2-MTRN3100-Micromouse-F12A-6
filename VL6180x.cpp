/**
 * @Note
 * This code is adapated and modified from the following codebase:
 * https://github.com/pololu/vl6180x-arduino/blob/master/VL6180X.cpp
 *
 */

#include "VL6180x.hpp"

// Default address for VL6180x is 0x29 (7-bit)
#define ADDRESS_DEFAULT (0b0101001)

// RANGE_SCALER values for 1x, 2x, 3x scaling - see STSW-IMG003
// core/src/vl6180x_api.c (ScalerLookUP[])
static uint16_t const ScalerValues[] = {0, 253, 127, 84};

static void configure_gpio(mtrn3100::VL6180X::Pin p1,
                           mtrn3100::VL6180X::Pin p2) {
#ifdef ARDUINO_AVR_NANO
  if (p1 != NC) {
    pinMode(p1, OUTPUT);
  }
  if (p2 != NC) {
    pinMode(p2, OUTPUT);
  }
#endif

#ifdef ARDUINO_ARCH_STM32
  if (p1 != NC) {
    GPIO_TypeDef* gpio = set_GPIO_Port_Clock(STM_PORT(p1));
    uint32_t ll_pin = STM_LL_GPIO_PIN(p1);
    LL_GPIO_SetPinMode(gpio, ll_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(gpio, ll_pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(gpio, ll_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(gpio, ll_pin, LL_GPIO_PULL_NO);
  }
  if (p2 != NC) {
    GPIO_TypeDef* gpio = set_GPIO_Port_Clock(STM_PORT(p2));
    uint32_t ll_pin = STM_LL_GPIO_PIN(p2);
    LL_GPIO_SetPinMode(gpio, ll_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(gpio, ll_pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(gpio, ll_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(gpio, ll_pin, LL_GPIO_PULL_NO);
  }

#endif
}

#ifdef ARDUINO_ARCH_STM32
inline static void writePinFast(PinName pin, int level) {
  auto port = get_GPIO_Port(STM_PORT(pin));
  auto pinmsk = STM_LL_GPIO_PIN(pin);
  LL_GPIO_WriteReg(port, BSRR, pinmsk << (level == 0 ? 16u : 0u));
}
#endif

namespace mtrn3100 {

VL6180X::VL6180X() : VL6180X(&Wire) {}

VL6180X::VL6180X(TwoWire* bus, Pin GPIO0, Pin GPIO1)
    : gpio_0_(GPIO0),
      gpio_1_(GPIO1),
      address_(ADDRESS_DEFAULT),
      scaling_(0),
      ptp_offset_(0),
      io_timeout_(0),  // no timeout
      timeout_(false),
      bus_(bus) {}

bool VL6180X::setAddress(const uint8_t& address) {
  uint8_t status = writeReg(regAddr::I2C_SLAVE__DEVICE_ADDRESS, address & 0x7F);
  if (status == 0) {
    this->address_ = address;
    uint8_t addr = readReg(regAddr::I2C_SLAVE__DEVICE_ADDRESS);
    if (address == addr) {
      this->address_ = addr;
      return true;
    }
  } else {
    return false;
  }
  return false;
}

void VL6180X::init(void) {
  configure_gpio(gpio_0_, gpio_1_);

  if (gpio_0_ != NC) {
#ifdef ARDUINO_AVR_NANO
    digitalWrite(gpio_0_, LOW);
    delayMicroseconds(10);
    digitalWrite(gpio_0_, HIGH);
#endif

#ifdef ARDUINO_ARCH_STM32
    writePinFast(gpio_0_, LOW);
    delayMicroseconds(10);
    writePinFast(gpio_0_, HIGH);
#endif
    delayMicroseconds(1050);
  }

  if (readReg(regAddr::SYSTEM__FRESH_OUT_OF_RESET) == 1) {
    scaling_ = 1;

    writeReg(0x207, 0x01, true);
    writeReg(0x208, 0x01, true);
    writeReg(0x096, 0x00, true);
    writeReg(0x097, 0xFD, true);  // RANGE_SCALER = 253
    writeReg(0x0E3, 0x01, true);
    writeReg(0x0E4, 0x03, true);
    writeReg(0x0E5, 0x02, true);
    writeReg(0x0E6, 0x01, true);
    writeReg(0x0E7, 0x03, true);
    writeReg(0x0F5, 0x02, true);
    writeReg(0x0D9, 0x05, true);
    writeReg(0x0DB, 0xCE, true);
    writeReg(0x0DC, 0x03, true);
    writeReg(0x0DD, 0xF8, true);
    writeReg(0x09F, 0x00, true);
    writeReg(0x0A3, 0x3C, true);
    writeReg(0x0B7, 0x00, true);
    writeReg(0x0BB, 0x3C, true);
    writeReg(0x0B2, 0x09, true);
    writeReg(0x0CA, 0x09, true);
    writeReg(0x198, 0x01, true);
    writeReg(0x1B0, 0x17, true);
    writeReg(0x1AD, 0x00, true);
    writeReg(0x0FF, 0x05, true);
    writeReg(0x100, 0x05, true);
    writeReg(0x199, 0x05, true);
    writeReg(0x1A6, 0x1B, true);
    writeReg(0x1AC, 0x3E, true);
    writeReg(0x1A7, 0x1F, true);
    writeReg(0x030, 0x00, true);

    writeReg(regAddr::SYSTEM__FRESH_OUT_OF_RESET, 0x00);

    // Store part-to-part range offset so it can be adjusted if scaling is
    // changed
    ptp_offset_ = readReg(regAddr::SYSRANGE__PART_TO_PART_RANGE_OFFSET);
  } else {
    // Sensor has already been initialized, so try to get scaling settings by
    // reading registers.

    uint16_t s = readReg16Bit(regAddr::RANGE_SCALER);

    if (s == ScalerValues[3]) {
      scaling_ = 3;
    } else if (s == ScalerValues[2]) {
      scaling_ = 2;
    } else {
      scaling_ = 1;
    }

    // Store part-to-part range offset so it can be adjusted if scaling is
    // changed
    ptp_offset_ = readReg(regAddr::SYSRANGE__PART_TO_PART_RANGE_OFFSET);

    // Adjust the part-to-part range offset value read earlier to account for
    // existing scaling. If the sensor was already in 2x or 3x scaling mode,
    // precision will be lost calculating the original (1x) offset, but this can
    // be resolved by resetting the sensor and Arduino again.
    ptp_offset_ *= scaling_;
  }
}

void VL6180X::configureDefault() {
  // "Recommended : Public registers"

  // readout__averaging_sample_period = 48
  writeReg(regAddr::READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);

  // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01
  // according to table "Actual gain values" in datasheet)
  writeReg(regAddr::SYSALS__ANALOGUE_GAIN, 0x46);

  // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature
  // recalibration after every 255 range measurements)
  writeReg(regAddr::SYSRANGE__VHV_REPEAT_RATE, 0xFF);

  // sysals__integration_period = 99 (100 ms)
  writeReg16Bit(regAddr::SYSALS__INTEGRATION_PERIOD, 0x0063);

  // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
  writeReg(regAddr::SYSRANGE__VHV_RECALIBRATE, 0x01);

  // "Optional: Public registers"

  // sysrange__intermeasurement_period = 9 (100 ms)
  writeReg(regAddr::SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);

  // sysals__intermeasurement_period = 49 (500 ms)
  writeReg(regAddr::SYSALS__INTERMEASUREMENT_PERIOD, 0x31);

  // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4
  // (range new sample ready interrupt)
  writeReg(regAddr::SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);

  // Reset other settings to power-on defaults

  // sysrange__max_convergence_time = 49 (49 ms)
  writeReg(VL6180X::regAddr::SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);

  // disable interleaved mode
  writeReg(regAddr::INTERLEAVED_MODE__ENABLE, 0);

  // reset range scaling factor to 1x
  setScaling(1);
}

// Writes an 8-bit register
uint8_t VL6180X::writeReg(uint16_t reg, uint8_t value, bool restart) {
  bus_->beginTransmission(address_);
  bus_->write((uint8_t)(reg >> 8));  // reg high byte
  bus_->write((uint8_t)(reg >> 0));  // reg low byte
  bus_->write((uint8_t)value);
  i2c_status = bus_->endTransmission(!restart);
  return i2c_status;
}

// Writes a 16-bit register
uint8_t VL6180X::writeReg16Bit(uint16_t reg, uint16_t value, bool restart) {
  bus_->beginTransmission(address_);
  bus_->write((uint8_t)(reg >> 8));    // reg high byte
  bus_->write((uint8_t)(reg >> 0));    // reg low byte
  bus_->write((uint8_t)(value >> 8));  // value high byte
  bus_->write((uint8_t)(value >> 0));  // value low byte
  i2c_status = bus_->endTransmission(!restart);
  return i2c_status;
}

// Writes a 32-bit register
uint8_t VL6180X::writeReg32Bit(uint16_t reg, uint32_t value, bool restart) {
  bus_->beginTransmission(address_);
  bus_->write((uint8_t)(reg >> 8));     // reg high byte
  bus_->write((uint8_t)(reg >> 0));     // reg low byte
  bus_->write((uint8_t)(value >> 24));  // value highest byte
  bus_->write((uint8_t)(value >> 16));
  bus_->write((uint8_t)(value >> 8));
  bus_->write((uint8_t)(value >> 0));  // value lowest byte
  i2c_status = bus_->endTransmission(!restart);
  return i2c_status;
}

// Reads an 8-bit register
uint8_t VL6180X::readReg(uint16_t reg) {
  uint8_t value = UINT8_C(0);

  bus_->beginTransmission(address_);
  bus_->write((uint8_t)(reg >> 8));  // reg high byte
  bus_->write((uint8_t)(reg >> 0));  // reg low byte
  i2c_status = bus_->endTransmission();

  bus_->requestFrom(address_, UINT8_C(1), true);
  value = bus_->read();

  return value;
}

// Reads a 16-bit register
uint16_t VL6180X::readReg16Bit(uint16_t reg) {
  uint16_t value = UINT16_C(0);

  bus_->beginTransmission(address_);
  bus_->write((uint8_t)(reg >> 8));  // reg high byte
  bus_->write((uint8_t)(reg >> 0));  // reg low byte
  i2c_status = bus_->endTransmission();

  bus_->requestFrom(address_, UINT8_C(2), true);
  value = (uint16_t)bus_->read() << 8;  // value high byte
  value |= bus_->read();                // value low byte

  return value;
}

// Reads a 32-bit register
uint32_t VL6180X::readReg32Bit(uint16_t reg) {
  uint32_t value = 0u;

  bus_->beginTransmission(address_);
  bus_->write((uint8_t)(reg >> 8));  // reg high byte
  bus_->write((uint8_t)(reg >> 0));  // reg low byte
  i2c_status = bus_->endTransmission();

  bus_->requestFrom(address_, UINT8_C(4), true);
  value = (uint32_t)bus_->read() << 24;  // value highest byte
  value |= (uint32_t)bus_->read() << 16;
  value |= (uint16_t)bus_->read() << 8;
  value |= bus_->read();  // value lowest byte

  return value;
}

// Set range scaling factor. The sensor uses 1x scaling by default, giving range
// measurements in units of mm. Increasing the scaling to 2x or 3x makes it give
// raw values in units of 2 mm or 3 mm instead. In other words, a bigger scaling
// factor increases the sensor's potential maximum range but reduces its
// resolution.

// Implemented using ST's VL6180X API as a reference (STSW-IMG003); see
// VL6180x_UpscaleSetScaling() in vl6180x_api.c.
void VL6180X::setScaling(uint8_t new_scaling) {
  // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT
  uint8_t const DefaultCrosstalkValidHeight = 20;

  // do nothing if scaling value is invalid
  if (new_scaling < 1 || new_scaling > 3) {
    return;
  }

  scaling_ = new_scaling;
  writeReg16Bit(regAddr::RANGE_SCALER, ScalerValues[scaling_]);

  // apply scaling on part-to-part offset
  writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET,
           ptp_offset_ / scaling_);

  // apply scaling on CrossTalkValidHeight
  writeReg(VL6180X::SYSRANGE__CROSSTALK_VALID_HEIGHT,
           DefaultCrosstalkValidHeight / scaling_);

  // This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

  // enable early convergence estimate only at 1x scaling
  uint8_t rce = readReg(VL6180X::SYSRANGE__RANGE_CHECK_ENABLES);
  writeReg(VL6180X::SYSRANGE__RANGE_CHECK_ENABLES,
           (rce & 0xFE) | (scaling_ == 1));
}

// Performs a single-shot ranging measurement
uint16_t VL6180X::getRangeSingle(unsigned timeout) {
  unsigned now = millis();
  writeReg(regAddr::SYSRANGE__START, 0x01);
  // blocks until a sample is available
  while (!this->isRangeSampleReady()) {
    if (millis() - now >= timeout) {
      return 0;
    }
  }

  uint8_t value = readReg(regAddr::RESULT__RANGE_VAL);
  this->writeReg(regAddr::SYSTEM__INTERRUPT_CLEAR, 0x15);

  return (uint16_t)this->scaling_ * (uint16_t)value;
}

// Performs a single-shot ambient light measurement
uint16_t VL6180X::getAmbientSingle(unsigned timeout) {
  writeReg(regAddr::SYSALS__START, 0x01);
  return getAmbient();
}

// Starts continuous ranging measurements with the given period in ms
// (10 ms resolution; defaults to 100 ms if not specified).
//
// The period must be greater than the time it takes to perform a
// measurement. See section "Continuous mode limits" in the datasheet
// for details.
void VL6180X::startRangeContinuous(uint16_t period) {
  int16_t period_reg = (int16_t)(period / 10) - 1;
  period_reg = constrain(period_reg, 0, 254);

  writeReg(regAddr::SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);
  writeReg(regAddr::SYSRANGE__START, 0x03);
}

// Starts continuous ambient light measurements with the given period in ms
// (10 ms resolution; defaults to 500 ms if not specified).
//
// The period must be greater than the time it takes to perform a
// measurement. See section "Continuous mode limits" in the datasheet
// for details.
void VL6180X::startAmbientContinuous(uint16_t period) {
  int16_t period_reg = (int16_t)(period / 10) - 1;
  period_reg = constrain(period_reg, 0, 254);

  writeReg(regAddr::SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
  writeReg(regAddr::SYSALS__START, 0x03);
}

// Starts continuous interleaved measurements with the given period in ms
// (10 ms resolution; defaults to 500 ms if not specified). In this mode, each
// ambient light measurement is immediately followed by a range measurement.
//
// The datasheet recommends using this mode instead of running "range and ALS
// continuous modes simultaneously (i.e. asynchronously)".
//
// The period must be greater than the time it takes to perform both
// measurements. See section "Continuous mode limits" in the datasheet
// for details.
void VL6180X::startInterleavedContinuous(uint16_t period) {
  int16_t period_reg = (int16_t)(period / 10) - 1;
  period_reg = constrain(period_reg, 0, 254);

  writeReg(regAddr::INTERLEAVED_MODE__ENABLE, 1);
  writeReg(regAddr::SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
  writeReg(regAddr::SYSALS__START, 0x03);
}

// Stops continuous mode. This will actually start a single measurement of range
// and/or ambient light if continuous mode is not active, so it's a good idea to
// wait a few hundred ms after calling this function to let that complete
// before starting continuous mode again or taking a reading.
void VL6180X::stopContinuous() {
  writeReg(regAddr::SYSRANGE__START, 0x01);
  writeReg(regAddr::SYSALS__START, 0x01);
  writeReg(regAddr::INTERLEAVED_MODE__ENABLE, 0);
}

/** Returns a range reading
 *  `isRangeSampleReady()` should be called before calling this function
 */
uint16_t VL6180X::getRange(void) {
  uint8_t range = readReg(regAddr::RESULT__RANGE_VAL);
  this->error_status =
      static_cast<ErrorCode>(readReg(regAddr::RESULT__RANGE_STATUS) & 0xF);
  writeReg(regAddr::SYSTEM__INTERRUPT_CLEAR, 0x01);

  return (uint16_t)this->scaling_ * (uint16_t)range;
}
/** Returns an ALS reading
 *  `isAmbientSampleReady()` should be called before calling this function
 */
uint16_t VL6180X::getAmbient() {
  uint16_t ambient = readReg16Bit(regAddr::RESULT__ALS_VAL);
  writeReg(regAddr::SYSTEM__INTERRUPT_CLEAR, 0x02);

  return ambient;
}

// Returns true if a range sample is available, false otherwise
bool VL6180X::isRangeSampleReady(void) {
  // if bits 2:0 (mask 0x07) are set to 0b100 (0x04)
  uint16_t reg = readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
  return static_cast<bool>((reg & 0x07) == 0x04);
}

// Returns true if an ALS sample is available, false otherwise
bool VL6180X::isAmbientSampleReady(void) {
  // if bits 5:3 (mask 0x38) are set to 0b100 (0x20)
  uint16_t reg = readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
  return static_cast<bool>((reg & 0x38) == 0x20);
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL6180X::isTimeout() {
  bool tmp = timeout_;
  timeout_ = false;
  return tmp;
}

}  // namespace mtrn3100

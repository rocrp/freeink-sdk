#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <esp_rom_sys.h>

// Shared Wire owns transaction serialization (endTransmission(false) + requestFrom).
// Only the input/setup/shutdown task writes the expander's output shadow.
namespace freeink::metalio {
constexpr uint8_t EXPANDER = 0x20;
constexpr uint8_t CHARGER = 0x6B;
constexpr uint16_t MAIN_POWER = 1u << 6;
constexpr uint16_t SCREEN_POWER = 1u << 5;
constexpr uint16_t TOUCH_RESET = 1u << 9;
constexpr uint16_t POWER_PULSE = 1u << 11;
// FSUSB42 USB mux select: high routes native USB to the MCU/flash side (stock default), low to the camera.
constexpr uint16_t USB_MUX_SELECT = 1u << 0;
constexpr uint16_t OUTPUTS =
    MAIN_POWER | SCREEN_POWER | TOUCH_RESET | POWER_PULSE | USB_MUX_SELECT | (1u << 4) | (1u << 1);
constexpr uint16_t BOOT_OUTPUT = MAIN_POWER | POWER_PULSE | USB_MUX_SELECT;
inline uint16_t output = BOOT_OUTPUT;
inline bool ready = false;
inline bool bootPowerPending = true;

inline bool powerButtonPressed(bool pressed) {
  if (!bootPowerPending) return pressed;
  if (!pressed) bootPowerPending = false;
  return false;  // Consume the initial held gesture and its release on every boot.
}

inline bool read(uint8_t addr, uint8_t reg, uint8_t* bytes, uint8_t count) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, count, static_cast<uint8_t>(true)) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) bytes[i] = Wire.read();
  return true;
}

inline bool write16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(EXPANDER);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value));
  Wire.write(static_cast<uint8_t>(value >> 8));
  return Wire.endTransmission() == 0;
}

inline bool setOutput(uint16_t value) {
  if (!write16(2, value)) return false;
  output = value;
  return true;
}

inline bool begin() {
  if (ready) return true;
  pinMode(44, OUTPUT);
  digitalWrite(44, LOW);      // Vibration motor idles off; the application pulses it.
  pinMode(46, INPUT_PULLUP);  // SD DAT3/CD: input-only, never part of the 1-bit data bus.
  pinMode(2, INPUT_PULLUP);
  if (!Wire.begin(41, 42, 400000)) return false;
  Wire.setTimeOut(10);
  // Set idle output levels BEFORE enabling the drivers: no low shutdown pulse.
  if (!setOutput(BOOT_OUTPUT) || !write16(6, static_cast<uint16_t>(~OUTPUTS)) || !setOutput(output | SCREEN_POWER))
    return false;
  delay(10);
  if (!setOutput(output | TOUCH_RESET)) return false;
  delay(120);
  ready = true;
  return true;
}

inline uint8_t buttons() {
  static uint32_t nextRead = 0;
  static uint8_t state = 0;
  const uint32_t now = millis();
  if (!ready || static_cast<int32_t>(now - nextRead) < 0) return state;
  uint8_t data[2];
  if (!read(EXPANDER, 0, data, sizeof(data))) {
    nextRead = now + 2000;
    state = 0;  // A failed read must never retain a held key.
    esp_rom_printf("[metalio] TCA9555 input read failed\r\n");
    return state;
  }
  nextRead = now + 20;
  state = ((data[0] & 0x80) ? 0 : (1u << 5)) | ((data[1] & 0x01) ? 0 : (1u << 4));
  return state;
}

inline bool externalPowerConnected(bool& connected) {
  // Read-only CX25601N status. Never run the reference charger's voltage/current init.
  static uint32_t nextRead = 0;
  static bool valid = false;
  static bool cached = false;
  const uint32_t now = millis();
  if (!ready) return false;
  if (static_cast<int32_t>(now - nextRead) >= 0) {
    uint8_t status;
    valid = read(CHARGER, 0x1E, &status, 1);
    if (valid) {
      const uint8_t source = status & 7;
      valid = source != 6;                  // Reserved encoding: leave fallback to the consumer.
      cached = source >= 1 && source <= 5;  // 0 = absent, 7 = OTG output.
    }
    nextRead = now + (valid ? 1000 : 2000);
  }
  if (valid) connected = cached;
  return valid;
}

// Caller has saved state, parked the display and waited for its BUSY completion.
inline bool shutdown() {
  if (!ready) return false;
  delay(280);
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (!setOutput(output | POWER_PULSE)) return false;
    delay(100);
    if (!setOutput(output & ~POWER_PULSE)) return false;
    delay(100);
  }
  // USB may keep the MCU alive. Restore the idle level before fallback deep sleep.
  return setOutput(output | POWER_PULSE);
}
}  // namespace freeink::metalio

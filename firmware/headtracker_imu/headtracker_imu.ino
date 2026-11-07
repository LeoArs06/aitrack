// Hybrid head tracker, protocol v1, firmware 1.0. AVR core 1.8.8 / Leonardo.
#include <Wire.h>
#include <avr/io.h>

static const uint32_t WIRE_TIMEOUT_US = 2500;
static uint8_t address = 0x68, who = 0, consecutiveErrors = 0;
static uint32_t seq = 0, i2cErrors = 0, reinits = 0, txDrops = 0;
static uint32_t deadline = 0, lastReady = 0, lastStatus = 0;
static uint16_t backoff = 100;
static bool streaming = true, infoPending = false, statusSent = false;
static bool attempted = false, isReinit = false;
static bool identified = false;
static uint8_t stickyFlags = 0, failureCode = 1, pendingStatus = 0;
static uint8_t sampleErrors = 0;
enum Phase { RETRY, RESET_WAIT, RUNNING };
static Phase phase = RETRY;

static void put16(uint8_t *p, uint16_t v) {
  p[0] = v; p[1] = v >> 8;
}
static void put32(uint8_t *p, uint32_t v) {
  for (uint8_t i = 0; i < 4; ++i) { p[i] = v; v >>= 8; }
}
static uint16_t crc16(const uint8_t *p, uint8_t n) {
  uint16_t crc = 0xffff;
  while (n--) {
    crc ^= uint16_t(*p++) << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}
static bool sendFrame(uint8_t type, const uint8_t *payload, uint8_t len) {
  // Serial's bool conversion in AVR 1.8.8 delays 10 ms: use DTR directly.
  if (!Serial.dtr() || Serial.availableForWrite() < len + 6) return false;
  uint8_t frame[34];  // Largest outgoing payload is INFO (28 bytes).
  frame[0] = 0xa5; frame[1] = 0x5a; frame[2] = type; frame[3] = len;
  memcpy(frame + 4, payload, len);
  put16(frame + 4 + len, crc16(frame + 2, len + 2));
  return Serial.write(frame, len + 6) == size_t(len + 6);
}
static void sendInfo() {
  uint8_t p[28] = {0};
  put32(p, 0x31495448UL);
  p[4] = 1; p[5] = 1; p[6] = 0; p[7] = identified ? who : 0;
  p[8] = 2; p[9] = 1; p[10] = 3; p[11] = 4;
  put16(p + 12, 200); put16(p + 14, 400);
  put32(p + 16, i2cErrors); put32(p + 20, reinits); put32(p + 24, txDrops);
  if (sendFrame(2, p, sizeof p)) infoPending = false;
}
static void serviceStatus(uint32_t now) {
  if (!pendingStatus || (statusSent && uint32_t(now - lastStatus) < 500)) return;
  uint8_t p[8] = {0}; p[0] = pendingStatus; p[1] = who;
  put32(p + 4, i2cErrors);
  if (sendFrame(3, p, sizeof p)) {
    lastStatus = now; statusSent = true; pendingStatus = 0;
  }
}
static bool result(bool ok) {
  // Wire's timeout reset also calls twi_init(), which enables 5 V pull-ups.
  // Clear them immediately upon return from every bus operation as well.
  PORTD &= ~(_BV(PD1) | _BV(PD0));
  if (Wire.getWireTimeoutFlag()) ok = false;
  Wire.clearWireTimeoutFlag();
  if (!ok) { ++i2cErrors; if (consecutiveErrors < 255) ++consecutiveErrors; }
  else consecutiveErrors = 0;
  return ok;
}
static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
  return result(Wire.endTransmission() == 0);
}
static bool readRegs(uint8_t reg, uint8_t *out, uint8_t n) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) return result(false);
  uint8_t received = Wire.requestFrom(address, n, uint8_t(true));
  if (received != n || Wire.getWireTimeoutFlag()) return result(false);
  for (uint8_t i = 0; i < n; ++i) out[i] = Wire.read();
  return result(true);
}
static void releasePin(uint8_t pin) { digitalWrite(pin, LOW); pinMode(pin, INPUT); }
static void pullLow(uint8_t pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
static void releaseClock() {
  releasePin(SCL);
  uint32_t start = micros();
  while (!digitalRead(SCL) && uint32_t(micros() - start) < WIRE_TIMEOUT_US) {}
  delayMicroseconds(5);
}
static void recoverBus() {
  Wire.end();  // Release TWI before using the pins as open-drain GPIO.
  releasePin(SDA); releasePin(SCL);
  for (uint8_t i = 0; i < 9 && !digitalRead(SDA); ++i) {
    pullLow(SCL); delayMicroseconds(5); releaseClock();
  }
  // STOP: SDA rises while SCL is released; never drive either line HIGH.
  pullLow(SCL); pullLow(SDA); delayMicroseconds(5);
  releaseClock(); releasePin(SDA); delayMicroseconds(5);
  Wire.begin();
  PORTD &= ~(_BV(PD1) | _BV(PD0)); // IMMEDIATELY disable SDA/SCL internal 5 V
  // pull-ups: the module supplies external pull-ups to its 3.3 V rail.
  Wire.setClock(400000UL);
  // Verified in the installed Arduino AVR 1.8.8 Wire.h; bounds all I2C waits.
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
  Wire.clearWireTimeoutFlag();
}
static void fail(uint8_t code) {
  failureCode = code; pendingStatus = code; phase = RETRY;
  deadline = millis() + backoff;
  backoff = backoff >= 500 ? 1000 : backoff * 2;
}
static bool verifyReg(uint8_t reg, uint8_t expected) {
  uint8_t value;
  if (!readRegs(reg, &value, 1)) return false;
  if (value == expected) return true;
  // A configuration mismatch is also an I2C/configuration error.
  ++i2cErrors; return false;
}
static bool family6500() { return who != 0x68; }
static void startInit() {
  isReinit = attempted; attempted = true;
  recoverBus();
  bool found = false;
  for (uint8_t candidate = 0x68; candidate <= 0x69; ++candidate) {
    uint8_t id = 0; address = candidate;
    if (readRegs(0x75, &id, 1) && id != 0) { who = id; found = true; break; }
  }
  // All nonzero WHO_AM_I values are accepted; INFO reports the actual value.
  identified = found;
  if (!found) { fail(1); return; }
  if (!writeReg(0x6b, 0x80)) { fail(2); return; }
  phase = RESET_WAIT; deadline = millis() + 100;
}
static void finishInit() {
  const uint8_t regs[] = {0x6b, 0x19, 0x1a, 0x1b, 0x1c, 0x38};
  const uint8_t values[] = {0x01, 4, 3, 0x10, 0x08, 0x01};
  for (uint8_t i = 0; i < sizeof regs; ++i) {
    if (!writeReg(regs[i], values[i]) || !verifyReg(regs[i], values[i])) {
      fail(2); return;
    }
  }
  if (family6500() && (!writeReg(0x1d, 3) || !verifyReg(0x1d, 3))) {
    fail(2); return;
  }
  if (isReinit) { ++reinits; stickyFlags |= 4; pendingStatus = 3; }
  phase = RUNNING; backoff = 100; consecutiveErrors = 0; sampleErrors = 0;
  lastReady = millis(); infoPending = true;
}
static bool saturated(const uint8_t *p) {
  return (p[0] == 0x7f && p[1] == 0xff) || (p[0] == 0x80 && p[1] == 0);
}
static void acquire() {
  uint8_t status;
  if (!readRegs(0x3a, &status, 1)) {
    if (consecutiveErrors >= 3) fail(2);
    return;
  }
  if (!(status & 1)) return;
  uint32_t timestamp = micros(); // Immediately after observing DATA_RDY.
  uint8_t raw[14];
  if (!readRegs(0x3b, raw, sizeof raw)) {
    // Successful status polls do not hide a streak of failed acquisitions.
    if (++sampleErrors >= 3) fail(2);
    return;
  }
  sampleErrors = 0;
  lastReady = millis();
  uint8_t p[24] = {0}; put32(p, seq++); put32(p + 4, timestamp);
  for (uint8_t i = 0; i < 14; i += 2) { p[8+i] = raw[i+1]; p[9+i] = raw[i]; }
  p[22] = stickyFlags;
  for (uint8_t i = 0; i < 6; i += 2) if (saturated(raw + i)) p[22] |= 1;
  for (uint8_t i = 8; i < 14; i += 2) if (saturated(raw + i)) p[22] |= 2;
  if (streaming) {
    if (sendFrame(1, p, sizeof p)) stickyFlags = 0;
    else { ++txDrops; stickyFlags |= 8; }
  }
}
void setup() {
  // Leave RX/TX LEDs unused: prevent the USB core's normal traffic blinking.
  // Both LEDs are active-low on Leonardo / Pro Micro; inputs cannot sink them.
  DDRD &= ~_BV(PD5); DDRB &= ~_BV(PB0);
  Serial.begin(115200);
}
void loop() {
  static uint8_t command = 0, commandLength = 0;
  // Bounded command work, even when the host floods the input endpoint.
  for (uint8_t n = 0; n < 16 && Serial.available(); ++n) {
    uint8_t c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      command = c;
      if (commandLength < 2) ++commandLength;
      continue;
    }
    // Only a one-character line is a command; ignore unknown/long lines.
    if (commandLength == 1) switch (command) {
      case '?': infoPending = true; break;
      case 'R': phase = RETRY; deadline = millis(); break;
      case '0': streaming = false; break;
      case '1': streaming = true; break;
      default: break;
    }
    commandLength = 0;
  }
  uint32_t now = millis();
  if (phase == RETRY && int32_t(now - deadline) >= 0) startInit();
  else if (phase == RESET_WAIT && int32_t(now - deadline) >= 0) finishInit();
  else if (phase == RUNNING) {
    acquire();
    if (phase == RUNNING && uint32_t(millis() - lastReady) > 50) fail(4);
  }
  if (phase == RETRY && !pendingStatus) pendingStatus = failureCode;
  if (infoPending) sendInfo();
  serviceStatus(millis());
}

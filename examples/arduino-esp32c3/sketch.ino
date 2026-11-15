// QMI8658A IMU demo for the Wokwi web editor (Arduino, ESP32-C3).
//
// Talks to the QMI8658A Wokwi chip over raw I2C and prints acceleration, angular rate and
// temperature in the same format as the Rust example in ../esp32c3. Wiring: SDA = GPIO4,
// SCL = GPIO5. The sketch polls STATUS0 for new data and needs no interrupt line.

#include <Wire.h>

constexpr uint8_t QMI8658A_ADDRESS = 0x6A;
constexpr uint8_t QMI8658A_WHO_AM_I = 0x05;

constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_REVISION_ID = 0x01;
constexpr uint8_t REG_CTRL1 = 0x02;
constexpr uint8_t REG_CTRL2 = 0x03;
constexpr uint8_t REG_CTRL3 = 0x04;
constexpr uint8_t REG_CTRL7 = 0x08;
constexpr uint8_t REG_CTRL8 = 0x09;
constexpr uint8_t REG_STATUSINT = 0x2D;
constexpr uint8_t REG_STATUS0 = 0x2E;
constexpr uint8_t REG_TEMP_L = 0x33;
constexpr uint8_t REG_AX_L = 0x35;
constexpr uint8_t REG_DQY_L = 0x4D;
constexpr uint8_t REG_DVX_L = 0x51;
constexpr uint8_t REG_RESET = 0x60;

constexpr uint8_t SOFT_RESET_COMMAND = 0xB0;
constexpr uint8_t RESET_DONE = 0x80;
constexpr uint32_t TURN_ON_TIME_MS = 15;

// CTRL1: ADDR_AI (auto-increment), BE (big-endian), INT2_EN.
constexpr uint8_t CTRL1_ADDR_AI = 0x40;
constexpr uint8_t CTRL1_BE = 0x20;
constexpr uint8_t CTRL1_INT2_EN = 0x10;

// CTRL2/CTRL3: self-test bit, and full scale + ODR: ±8 g / ±512 dps at 56 Hz.
constexpr uint8_t CTRL23_ST = 0x80;
constexpr uint8_t CTRL2_ACCEL_8G_56HZ = 0x27;
constexpr uint8_t CTRL3_GYRO_512DPS_56HZ = 0x57;

// CTRL7: aEN, gEN.
constexpr uint8_t CTRL7_A_EN = 0x01;
constexpr uint8_t CTRL7_G_EN = 0x02;

// CTRL8: CTRL9 handshake via STATUSINT.bit7.
constexpr uint8_t CTRL8_CTRL9_HANDSHAKE_TYPE = 0x80;

// STATUSINT.bit0 (Avail) follows INT2; STATUS0: aDA, gDA.
constexpr uint8_t STATUSINT_AVAIL = 0x01;
constexpr uint8_t STATUS0_A_DA = 0x01;
constexpr uint8_t STATUS0_G_DA = 0x02;

constexpr float ACCEL_LSB_PER_G = 4096.0f;
constexpr float GYRO_LSB_PER_DPS = 64.0f;
constexpr float TEMPERATURE_LSB_PER_CELSIUS = 256.0f;

// Self-test limits (datasheet 15.1, 15.2) and the result scales of dVX..dVZ.
constexpr float ACCEL_SELF_TEST_MIN_MG = 200.0f;
constexpr float GYRO_SELF_TEST_MIN_DPS = 300.0f;
constexpr uint32_t SELF_TEST_TIMEOUT_MS = 3000;

constexpr int PIN_SDA = 4;
constexpr int PIN_SCL = 5;

bool readRegisters(uint8_t reg, uint8_t *buffer, size_t len) {
  Wire.beginTransmission(QMI8658A_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(int(QMI8658A_ADDRESS), int(len)) != int(len)) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

uint8_t readRegister(uint8_t reg) {
  uint8_t value = 0;
  readRegisters(reg, &value, 1);
  return value;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMI8658A_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t le16(const uint8_t *bytes) {
  return int16_t(uint16_t(bytes[1]) << 8 | bytes[0]);
}

bool waitSelfTest(bool level) {
  for (uint32_t i = 0; i <= SELF_TEST_TIMEOUT_MS; i++) {
    if (bool(readRegister(REG_STATUSINT) & STATUSINT_AVAIL) == level) {
      return true;
    }
    delay(1);
  }

  return false;
}

// Self-test (datasheet 15.1, 15.2) with the sensors off; restores CTRL1 and CTRL7.
bool selfTest(uint8_t ctrlReg, int16_t raw[3]) {
  uint8_t ctrl1 = readRegister(REG_CTRL1);
  uint8_t ctrl7 = readRegister(REG_CTRL7);
  writeRegister(REG_CTRL7, 0x00);
  // The device signals completion on INT2 and STATUSINT.bit0.
  writeRegister(REG_CTRL1, ctrl1 | CTRL1_INT2_EN);

  writeRegister(ctrlReg, readRegister(ctrlReg) | CTRL23_ST);
  bool ok = waitSelfTest(true);
  writeRegister(ctrlReg, readRegister(ctrlReg) & ~CTRL23_ST);
  ok = ok && waitSelfTest(false);

  uint8_t bytes[6];
  ok = ok && readRegisters(REG_DVX_L, bytes, sizeof(bytes));
  for (int i = 0; ok && i < 3; i++) {
    raw[i] = le16(&bytes[2 * i]);
  }

  writeRegister(REG_CTRL1, ctrl1);
  writeRegister(REG_CTRL7, ctrl7);
  return ok;
}

bool accelSelfTestPassed() {
  int16_t raw[3];
  if (!selfTest(REG_CTRL2, raw)) {
    return false;
  }

  for (int16_t v : raw) {
    if (fabsf(v * 1000.0f / 2048.0f) <= ACCEL_SELF_TEST_MIN_MG) {
      return false;
    }
  }

  return true;
}

bool gyroSelfTestPassed() {
  int16_t raw[3];
  if (!selfTest(REG_CTRL3, raw)) {
    return false;
  }

  for (int16_t v : raw) {
    if (fabsf(v / 16.0f) <= GYRO_SELF_TEST_MIN_DPS) {
      return false;
    }
  }

  return true;
}

void halt() {
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // The device accepts no writes during its turn-on time after power-up.
  delay(TURN_ON_TIME_MS);

  uint8_t whoAmI = 0;
  if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1) || whoAmI != QMI8658A_WHO_AM_I) {
    Serial.printf("QMI8658A not found (WHO_AM_I 0x%02X)\n", whoAmI);
    halt();
  }

  uint8_t revision = readRegister(REG_REVISION_ID);
  Serial.printf("QMI8658A ok who_am_i=0x%02X revision=0x%02X\n", whoAmI, revision);

  // Soft reset, then wait for dQY_L to read RESET_DONE.
  writeRegister(REG_RESET, SOFT_RESET_COMMAND);
  delay(TURN_ON_TIME_MS);
  bool reset = false;
  for (int i = 0; i < 10 && !(reset = readRegister(REG_DQY_L) == RESET_DONE); i++) {
    delay(1);
  }

  if (!reset) {
    Serial.println("QMI8658A reset timed out");
    halt();
  }

  // Little-endian with address auto-increment; CTRL9 handshake via STATUSINT.bit7.
  writeRegister(REG_CTRL1, (readRegister(REG_CTRL1) | CTRL1_ADDR_AI) & ~CTRL1_BE);
  writeRegister(REG_CTRL8, readRegister(REG_CTRL8) | CTRL8_CTRL9_HANDSHAKE_TYPE);

  // A game reading 30 to 60 times a second needs no more than 56 Hz.
  writeRegister(REG_CTRL2, (readRegister(REG_CTRL2) & CTRL23_ST) | CTRL2_ACCEL_8G_56HZ);
  writeRegister(REG_CTRL3, (readRegister(REG_CTRL3) & CTRL23_ST) | CTRL3_GYRO_512DPS_56HZ);

  // The self-test runs at the configured accelerometer ODR, with the sensors off.
  bool accel = accelSelfTestPassed();
  bool gyro = gyroSelfTestPassed();
  Serial.printf("self-test accel=%s gyro=%s\n", accel ? "true" : "false",
                gyro ? "true" : "false");

  writeRegister(REG_CTRL7, readRegister(REG_CTRL7) | CTRL7_A_EN | CTRL7_G_EN);
  Serial.println("running accel=8g gyro=512dps odr=56Hz");
}

void loop() {
  static int16_t last[7];
  static bool haveLast = false;

  if (readRegister(REG_STATUS0) & (STATUS0_A_DA | STATUS0_G_DA)) {
    // AX..AZ, GX..GZ, then the temperature.
    uint8_t bytes[12], temp[2];
    if (readRegisters(REG_AX_L, bytes, sizeof(bytes)) &&
        readRegisters(REG_TEMP_L, temp, sizeof(temp))) {
      int16_t raw[7];
      for (int i = 0; i < 6; i++) {
        raw[i] = le16(&bytes[2 * i]);
      }
      raw[6] = le16(temp);

      if (!haveLast || memcmp(raw, last, sizeof(raw)) != 0) {
        Serial.printf(
            "accel x=%.2f y=%.2f z=%.2f g gyro x=%.1f y=%.1f z=%.1f dps temp=%.1fC\n",
            raw[0] / ACCEL_LSB_PER_G, raw[1] / ACCEL_LSB_PER_G, raw[2] / ACCEL_LSB_PER_G,
            raw[3] / GYRO_LSB_PER_DPS, raw[4] / GYRO_LSB_PER_DPS, raw[5] / GYRO_LSB_PER_DPS,
            raw[6] / TEMPERATURE_LSB_PER_CELSIUS);
        memcpy(last, raw, sizeof(raw));
        haveLast = true;
      }
    }
  }

  delay(5);
}

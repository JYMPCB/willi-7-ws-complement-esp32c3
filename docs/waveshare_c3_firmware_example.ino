#include <Arduino.h>
#include <Wire.h>

// C3 esclavo I2C
static const uint8_t I2C_ADDR = 0x2A;

// Pines de salida del C3 hacia actuadores reales
static const int PIN_R = 11;
static const int PIN_G = 12;
static const int PIN_B = 13;
static const int PIN_BEEP = 10;

// Protocolo S3 -> C3
static const uint8_t CMD_RGB  = 0xA1;
static const uint8_t CMD_BEEP = 0xB1;
static const uint8_t CMD_STOP = 0xB2;

static const int BEEP_CH = 0;
static const int BEEP_RES_BITS = 8;

static void handle_rgb(uint8_t r_on, uint8_t g_on, uint8_t b_on) {
  // Activo LOW para coincidir con logica actual
  digitalWrite(PIN_R, r_on ? LOW : HIGH);
  digitalWrite(PIN_G, g_on ? LOW : HIGH);
  digitalWrite(PIN_B, b_on ? LOW : HIGH);
}

static void handle_beep(uint16_t freq_hz, uint16_t dur_ms, uint8_t duty) {
  ledcWriteTone(BEEP_CH, freq_hz);
  ledcWrite(BEEP_CH, duty);
  delay(dur_ms);
  ledcWrite(BEEP_CH, 0);
}

void on_i2c_receive(int len) {
  if (len <= 0) return;

  uint8_t cmd = Wire.read();
  len--;

  if (cmd == CMD_RGB && len >= 3) {
    uint8_t r = Wire.read();
    uint8_t g = Wire.read();
    uint8_t b = Wire.read();
    handle_rgb(r, g, b);
    return;
  }

  if (cmd == CMD_BEEP && len >= 5) {
    uint8_t f_l = Wire.read();
    uint8_t f_h = Wire.read();
    uint8_t d_l = Wire.read();
    uint8_t d_h = Wire.read();
    uint8_t duty = Wire.read();

    uint16_t freq = (uint16_t)f_l | ((uint16_t)f_h << 8);
    uint16_t dur  = (uint16_t)d_l | ((uint16_t)d_h << 8);
    handle_beep(freq, dur, duty);
    return;
  }

  if (cmd == CMD_STOP) {
    ledcWrite(BEEP_CH, 0);
    return;
  }
}

void setup() {
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  digitalWrite(PIN_R, HIGH);
  digitalWrite(PIN_G, HIGH);
  digitalWrite(PIN_B, HIGH);

  ledcSetup(BEEP_CH, 2000, BEEP_RES_BITS);
  ledcAttachPin(PIN_BEEP, BEEP_CH);
  ledcWrite(BEEP_CH, 0);

  Wire.begin((int)I2C_ADDR);
  Wire.onReceive(on_i2c_receive);

  Serial.begin(115200);
  Serial.println("C3 I2C actuator ready @0x2A");
}

void loop() {
  delay(2);
}

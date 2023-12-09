#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* clock=*/SCL, /* data=*/SDA,
    /* reset=*/U8X8_PIN_NONE);  // High speed I2C
Adafruit_BME280 bme;

const uint8_t FAN_PWM_CHANNEL = 0;
const uint8_t FAN_PIN = 5;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  };
  if (!u8g2.begin()) {
    Serial.println("Failed to initialize OLED.");
  }
  if (!bme.begin(BME280_ADDRESS_ALTERNATE)) {
    Serial.println("Failed to initialize BME280 temperature sensor");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    while (1) {
      delay(100);
    }
  }
  ledcSetup(/*channel=*/FAN_PWM_CHANNEL, /*freq=*/15000,
            /*resolution_bits=*/8);
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);
  Serial.println("Initialized!");
}

bool spinning = false;
const float OFF_TEMPERATURE = 21.0f;
const float ON_TEMPERATURE = 23.0f;

void loop(void) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  float temp = bme.readTemperature();
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "Temp: %.02f ºC", temp);
  u8g2.drawStr(0, 10, buffer);
  bool fan_on = false;
  if (temp > ON_TEMPERATURE) {
    if (!spinning) {
      // TODO(bwatling): this should use a PID loop
      ledcWrite(FAN_PWM_CHANNEL, 255);
      delay(2000);
    }
    spinning = true;
    ledcWrite(FAN_PWM_CHANNEL, 220);
    Serial.println("Fan on");
    u8g2.drawStr(0, 20, "Fan on");
    fan_on = true;
  } else if (temp < OFF_TEMPERATURE) {
    spinning = false;
    ledcWrite(FAN_PWM_CHANNEL, 0);
    Serial.println("Fan off");
    u8g2.drawStr(0, 20, "Fan off");
  }
  u8g2.sendBuffer();
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");
  delay(1000);
}
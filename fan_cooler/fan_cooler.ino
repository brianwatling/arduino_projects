#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE); // High speed I2C
Adafruit_BME280 bme;

void setup(void)
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    };
    if (!u8g2.begin())
    {
        Serial.println("Failed to initialize OLED.");
    }
    if (!bme.begin(BME280_ADDRESS_ALTERNATE))
    {
        Serial.println("Failed to initialize BME280 temperature sensor");
        Serial.print("SensorID was: 0x");
        Serial.println(bme.sensorID(), 16);
        while (1)
        {
            delay(100);
        }
    }
    Serial.println("Initialized!");
}

int x = 0;

void loop(void)
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    float temp = bme.readTemperature();
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Temp: %.02f ºC", temp);
    u8g2.drawStr(0, 10, buffer);
    u8g2.sendBuffer();
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");
    delay(1000);
}
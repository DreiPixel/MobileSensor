#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// define i2c pins
#define SDA 21
#define SCL 22

#define SEALEVELPRESSURE_HPA (1013.25)

float temp;
float hum;
float pres;
float gas;
float alt;
float hi;
float iaq;

// init display
// FIX connection to Display

// U8G2_SSD1309_128X64_NONAME0_F_HW_I2C desplay(U8G2_R0, SCL, SDA);
Adafruit_BME680 bme; // I2C

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }
  bme.setTemperatureOversampling(BME680_OS_16X);
  bme.setHumidityOversampling(BME680_OS_16X);
  bme.setPressureOversampling(BME680_OS_16X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  // put your setup code here, to run once:
  // desplay.begin();
  // desplay.setFont(u8g2_font_ncenB08_tr);
  // desplay.drawStr(0, 10, "Hello World!");
  // desplay.sendBuffer();
}

void loop()
{
  unsigned long endTime = bme.beginReading();
  if (endTime == 0)
  {
    Serial.println("Failed to begin reading :(");
    return;
  }
  Serial.print("Reading started at ");
  Serial.print(millis());
  Serial.print(" and will finish at ");
  Serial.println(endTime);

  if (!bme.endReading())
  {
    Serial.println("Failed to complete reading :(");
    return;
  }
  Serial.print("Reading completed at ");
  Serial.print(millis());
  Serial.println(", results:");
  Serial.print("Temperature = ");

  temp = bme.temperature;
  hum = bme.humidity;
  pres = bme.pressure;
  gas = bme.gas_resistance;
  alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  hi = -8.78469475556 + 1.61139411 * temp + 2.33854883889 * hum + -0.14611605 * temp * hum + -0.012308094 * (temp * temp) + -0.0164248277778 * (hum * hum) + 0.002211732 * (temp * temp) * hum + 0.00072546 * temp * (hum * hum) + -0.000003582 * (temp * temp) * (hum * hum);

  iaq = log(gas) + 0.04 * hum;

  Serial.print(temp);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pres / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(hum);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(gas / 1000.0);
  Serial.println(F(" KOhms"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(alt);
  Serial.println(F(" m"));

  Serial.print(F("Heat index = "));
  Serial.print(hi);
  Serial.println(F(" *C"));

  // Calculate IAQ
  Serial.print(F("IAQ = "));
  Serial.print(iaq);
  Serial.println();
  delay(5000);
}

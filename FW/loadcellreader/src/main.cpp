#include "pico.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "usb_service.h"
#include "adc_sensor.h"
#include "pins.h"

uint64_t last_time_us;

constexpr uint64_t loop_period_us = 1000 * 1000;

LoadcellADS1115 loadCell = LoadcellADS1115(&Wire, Pins::ADS1115_RDY);
bool led_on = false;

void scan_i2c()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    gpio_put(Pins::ONBOARD_LED, true);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    gpio_put(Pins::ONBOARD_LED, false);

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup()
{
  // put your setup code here, to run once:

  // Set up LED
  gpio_init(Pins::ONBOARD_LED);
  gpio_set_dir(Pins::ONBOARD_LED, GPIO_OUT);

  // Set up USB Device
  Serial.begin(115200);
  usb_service_setup(0x1234, 0x1234);

  // Set up pins
  Wire.setSDA(Pins::ADS1115_SDA);
  Wire.setSCL(Pins::ADS1115_SCL);
  Wire.setClock(400'000);

  // Start ADS1115
  loadCell.begin();
  last_time_us = time_us_64();
}

void loop()
{
  uint64_t curr_time_us = time_us_64();
  if (curr_time_us - last_time_us < loop_period_us)
  {
    return;
  }
  last_time_us = curr_time_us;
  // uint32_t rawCounts;
  // loadCell.getLastRawCounts(rawCounts);
  // Serial.printf("%.5f,", curr_time_us / 1000'000.0);
  // Serial.printf("%lu,", loadCell.getNumOfCounts());
  // Serial.printf("%llu,", loadCell.getLastInterruptDuration_us());
  // Serial.printf("%d,", loadCell.m_ads1115.getValue());
  // Serial.printf("%lu\n", rawCounts);
  scan_i2c();
  Serial.flush();

  // digitalWrite(Pins::ONBOARD_LED, led_on);
  // led_on *= -1;
}

#include "pico.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "usb_service.h"
#include "adc_sensor.h"
#include "pins.h"

uint64_t last_time_us;

constexpr uint64_t loop_period_us = 10 * 1000;

// Bessel 2nd order 50 hz cutoffs, 800hz sampling rate
std::vector<double> b_coeffs = {0.05044522, 0.10089044, 0.05044522}; // Numerator coefficients
std::vector<double> a_coeffs = {1, -1.17643871, 0.37821959}; // Denominator coefficients
IIRFilter iirFilter(b_coeffs, a_coeffs);

// LoadcellADS1115 loadCell = LoadcellADS1115(&Wire, Pins::ADS1115_RDY, iirFilter);

LoadCellADS1220::PinSetup pinSetup = {
  .MOSI = Pins::ADS1220_MOSI,
  .MISO = Pins::ADS1220_MISO,
  .SCK = Pins::ADS1220_SCK,
  .CS = Pins::ADS1220_CS,
  .DRDY = Pins::ADS1220_DRDY,
};
LoadCellADS1220 loadCell = LoadCellADS1220(pinSetup, iirFilter);
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
  // Wire.setSDA(Pins::ADS1115_SDA);
  // Wire.setSCL(Pins::ADS1115_SCL);
  // Wire.setClock(1'000'000);
  // Wire.begin();

  // Start ADS1115
  loadCell.begin();
  last_time_us = time_us_64();


  // Start ADS1220

}

void loop()
{
  uint64_t curr_time_us = time_us_64();
  if (curr_time_us - last_time_us < loop_period_us)
  {
    return;
  }
  last_time_us = curr_time_us;
  uint32_t rawCounts;
  loadCell.getLastRawCounts(rawCounts);
  Serial.printf("%.5f\t", curr_time_us / 1000'000.0);
  Serial.printf("%lu\t", loadCell.getNumOfCounts());
  Serial.printf("%llu\t", loadCell.getLastInterruptDuration_us());
  Serial.printf("%ld\t", rawCounts);
  Serial.printf("%f\n", loadCell.getLastFilteredCounts());
  Serial.flush();

  // digitalWrite(Pins::ONBOARD_LED, led_on);
  // led_on *= -1;
}

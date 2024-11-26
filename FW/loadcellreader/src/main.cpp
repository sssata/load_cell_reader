#include "pico.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "usb_service.h"
#include "adc_sensor.h"
#include "pins.h"
#include "sensor_reader.h"

uint64_t last_time_us;

constexpr uint64_t loop_period_us = 20 * 1000;

// Bessel 2nd order 50 hz cutoffs, 800hz sampling rate
std::vector<double> b_coeffs = {0.05044522, 0.10089044, 0.05044522}; // Numerator coefficients
std::vector<double> a_coeffs = {1, -1.17643871, 0.37821959};		 // Denominator coefficients
IIRFilter iirFilter(b_coeffs, a_coeffs);
IIRFilter iirFilter2(b_coeffs, a_coeffs);

// LoadcellADS1115 loadCell = LoadcellADS1115(&Wire, Pins::ADS1115_RDY, iirFilter);

ADS1220Pipeline::PinSetup pinSetup = {
	.MOSI = Pins::ADS1220_MOSI,
	.MISO = Pins::ADS1220_MISO,
	.SCK = Pins::ADS1220_SCK,
	.CS = Pins::ADS1220_CS,
	.DRDY = Pins::ADS1220_DRDY,
};

ADS1220Pipeline::PinSetup pinSetup2 = {
	.MOSI = Pins::ADS1220_MOSI,
	.MISO = Pins::ADS1220_MISO,
	.SCK = Pins::ADS1220_SCK,
	.CS = Pins::ADS1220_CS2,
	.DRDY = Pins::ADS1220_DRDY2,
};

queue_t *loadCell_queue = NULL;
ADS1220Pipeline *loadCell[2] = {nullptr, nullptr};

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

void start_core1(void){
	Serial.println("start_core1");
	
	ADCManager::getInstance().setup();
	ADCManager::getInstance().begin();

	Serial.println("Done start_core1");

	while (1)
	{
		sleep_ms(1000);
	}
}

void setup()
{
	// put your setup code here, to run once:
	// Set up USB Device
	Serial.begin(115200);
	usb_service_setup(0x1234, 0x1234);

	sleep_ms(3);
	Serial.println("Starting up1");

	// Set up LED
	gpio_init(Pins::ONBOARD_LED);
	gpio_set_dir(Pins::ONBOARD_LED, GPIO_OUT);

	loadCell[0] = new ADS1220Pipeline{0, pinSetup, iirFilter};
	loadCell[1] = new ADS1220Pipeline{1, pinSetup2, iirFilter2};

	for (auto adspipeline : loadCell)
	{
		ADCManager::getInstance().addADCPipeline(adspipeline);
	}

	multicore_launch_core1(start_core1);

	sleep_ms(100); // Wait for core1 to start

	last_time_us = time_us_64();
	Serial.println("Setup complete");
}

void loop()
{
	Serial.println("Starting Loop");
	Serial.flush();

	while (1)
	{
		uint64_t curr_time_us = time_us_64();
		if (curr_time_us - last_time_us < loop_period_us)
		{
			continue;
		}
		last_time_us = curr_time_us;
		for (auto loadCell : loadCell)
		{
			ADS1220Pipeline::SensorReading reading = {0};

			if (loadCell == nullptr)
			{
				Serial.printf("Not initalized yet\n");
				continue;
			}

			loadCell->getReading(reading);
			if (reading.readingNumber == 0)
			{
				Serial.printf("No readings yet\n");
				Serial.flush();
				continue;
			}

			Serial.printf("%llu\t", reading.lastValidReadingTimestamp_us);
			// Serial.printf("%llu\t", reading.readingNumber);
			Serial.printf("%ld\t", reading.rawReading);
			Serial.printf("%f\t", reading.reading);
			Serial.printf("%lu\t", reading.interruptDuration_us);
			// Serial.printf("%d\n", reading.lastValidReadingTimestamp_us);

			// Serial.printf("%llu\n", loadCell->getNumOfCounts());
			// Serial.printf("%f\n", loadCell->getLastFilteredCounts());
		}
		Serial.println();
		Serial.flush();

		// loadCell->getReading(reading);

		// if (reading.reading_number == 0){
		//   Serial.printf("No readings yet\n");
		//   Serial.flush();
		//   continue;
		// }

		// Serial.printf("%llu\t", reading.timestamp);
		// Serial.printf("%llu\t", reading.reading_number);
		// Serial.printf("%ld\t", reading.rawReading);
		// Serial.printf("%f\t", reading.reading);
		// Serial.printf("%lu\n", reading.interruptDuration_us);

		// Serial.printf("%llu\n", loadCell->getNumOfCounts());
		// Serial.printf("%f\n", loadCell->getLastFilteredCounts());
		// Serial.flush();

		// digitalWrite(Pins::ONBOARD_LED, led_on);
		// led_on *= -1;
		// 26743059670     15787555        -109471 -110153.273438  32
		// 26743300181     15787697        -110634 -110667.039062  32
	}
}

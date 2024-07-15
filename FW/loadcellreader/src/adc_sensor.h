#pragma once
#include <optional>
#include <hardware/gpio.h>
#include <hardware/timer.h>

#include "ADS1X15.h"
#include "ADS1220_driver.h"

// Arduino
#include "Wire.h"
#include <SPI.h>

#include "pins.h"
#include "filters.h"



constexpr int ADS1115_I2C_ADDR = 1;

enum class ErrorCode
{
    OK = 0,
    NOT_CONNECTED,
};

class AnalogSensor {
public:
    
    /*
    * @brief Initialize the sensor
    * @return ErrorCode
    */
    virtual ErrorCode begin() = 0;

    /*
    * @brief Get the last raw counts from the sensor
    * @param counts: uint32_t reference to store the counts
    * @return ErrorCode
    */
    virtual ErrorCode getLastRawCounts(uint32_t &counts) = 0;


    /*
    * @brief Get the last filtered counts from the sensor
    * @return double
    */
    virtual double getLastFilteredCounts() = 0;

    /*
    * @brief Get the number of reads from the sensor
    * @return uint64_t
    */
    virtual uint64_t getNumOfCounts() = 0;

    /*
    * @brief Get the duration of the last interrupt in microseconds
    * @return uint64_t
    */
    virtual uint64_t getLastInterruptDuration_us() = 0;

    /*
    * @brief Check if the sensor is connected
    * @return bool
    */
    virtual bool isConnected() = 0;
};

using namespace ADS1220Driver;

class LoadCellADS1220 : public AnalogSensor
{

public:

    struct PinSetup{
        uint8_t MOSI;
        uint8_t MISO;
        uint8_t SCK;
        uint8_t CS;
        uint8_t DRDY;
    };

    LoadCellADS1220(PinSetup pinSetup, Filter& filter) : m_pins(pinSetup), filter(filter)
    {
        if (!SPI.setRX(m_pins.MISO)){
            while(true){
                Serial.printf("Failed to set MISO pin to %d\n", m_pins.MISO);
                sleep_ms(1000);
            }
        }
        if (!SPI.setTX(m_pins.MOSI)){
            while(true){
                Serial.printf("Failed to set MOSI pin to %d\n", m_pins.MOSI);
                sleep_ms(1000);
            }
        }
        if (!SPI.setSCK(m_pins.SCK)){
            while(true){
                Serial.printf("Failed to set SCK pin to %d\n", m_pins.SCK);
                sleep_ms(1000);
            }
        }
        // SPI.setTX(m_pins.MOSI);
        // SPI.setSCK(m_pins.SCK);
        m_ads1220 = ADS1220();
    }

    ErrorCode begin()
    {

        m_ads1220.begin(m_pins.CS,m_pins.DRDY);


        if (!m_ads1220.isConnected())
        {
            while (true){
                Serial.println("ADS1220 not connected");
                Serial.printf("reg0 data: %x\n", m_ads1220.readRegister(CONFIG_REG0_ADDRESS));
                Serial.printf("reg1 data: %x\n", m_ads1220.readRegister(CONFIG_REG1_ADDRESS));
                Serial.printf("reg2 data: %x\n", m_ads1220.readRegister(CONFIG_REG2_ADDRESS));
                Serial.printf("reg3 data: %x\n", m_ads1220.readRegister(CONFIG_REG3_ADDRESS));
                
                Serial.flush();
                sleep_ms(1000);
            }
            return ErrorCode::NOT_CONNECTED;
        }

        // Set gain
        m_ads1220.set_data_rate(DR_600SPS);
        m_ads1220.set_pga_gain(PGA_GAIN_1);
        m_ads1220.select_mux_channels(MUX_AIN2_AIN3);  // Configure for differential measurement between AIN2 and AIN3
        m_ads1220.set_conv_mode_continuous();          // Set continuous conversion mode
        m_ads1220.Start_Conv();  // Start continuous conversion mode

        // Set interrupt handler to catch rdy
        attachInterruptParam(digitalPinToInterrupt(m_pins.DRDY), drdyCallback, FALLING, this);

        return ErrorCode::OK;
    }

    ErrorCode getLastRawCounts(uint32_t &counts)
    {
        counts = m_lastRawCounts;
        return ErrorCode::OK;
    }

    double getLastFilteredCounts()
    {
        return m_lastFilteredCounts;
    }

    uint64_t getNumOfCounts()
    {
        return m_noOfReads;
    }

    uint64_t getLastInterruptDuration_us()
    {
        return m_lastInterruptDuration_us;
    }

    bool isConnected()
    {
        return m_ads1220.isConnected();
    }
    

    ADS1220 m_ads1220;

private:
    static void drdyCallback(void* loadcell){
        uint64_t start_time_us = time_us_64();
        gpio_put(Pins::ONBOARD_LED, true);
        LoadCellADS1220 *self = static_cast<LoadCellADS1220 *>(loadcell);
        // self->m_lastRawCounts = self->m_ads1220.Read_Data_Samples();
        self->m_lastRawCounts = self->m_ads1220.Read_WaitForData();
        self->m_noOfReads++;
        self->m_lastFilteredCounts = self->filter.step(self->m_lastRawCounts);
        gpio_put(Pins::ONBOARD_LED, false);
        self->m_lastInterruptDuration_us = time_us_64() - start_time_us;
    }

    PinSetup m_pins;
    Filter& filter;

    int32_t m_lastRawCounts = 0;
    double m_lastFilteredCounts = 0;
    uint64_t m_noOfReads = 0;
    uint64_t m_lastInterruptDuration_us = 0;
};

class LoadcellADS1115 : public AnalogSensor
{
public:
    LoadcellADS1115(TwoWire *wire, uint32_t ready_pin, Filter& filter) : m_readyPin(ready_pin), filter(filter)
    {
        m_ads1115 = ADS1115(0x48, wire);
    }

    ErrorCode begin()
    {
        m_ads1115.begin();
        if (!m_ads1115.isConnected())
        {
            Serial.println("ADS1115 not connected");
            return ErrorCode::NOT_CONNECTED;
        }

        // Set gain
        m_ads1115.setGain(16);
        m_ads1115.setDataRate(7);

        // Set the MSB of the Hi_thresh register to 1
        m_ads1115.setComparatorThresholdHigh(0x8000);
        // Set the MSB of the Lo_thresh register to 0
        m_ads1115.setComparatorThresholdLow(0x0000);
        // SET ALERT RDY PIN (QueConvert mode)
        m_ads1115.setComparatorQueConvert(0);

        // Continuous mode
        m_ads1115.setMode(0);
        // Trigger first read
        m_ads1115.requestADC_Differential_0_1();

        // Set interrupt handler to catch rdy
        pinMode(m_readyPin, INPUT_PULLUP);
        attachInterruptParam(digitalPinToInterrupt(m_readyPin), adsReadyCallback, RISING, this);

        // trigger first read
        m_ads1115.requestADC_Differential_0_1(); 
        return ErrorCode::OK;
    }

    ErrorCode getLastRawCounts(uint32_t &counts)
    {
        counts = m_lastRawCounts;
        return ErrorCode::OK;
    }

    double getLastFilteredCounts()
    {
        return m_lastFilteredCounts;
    }

    uint64_t getNumOfCounts()
    {
        return m_noOfReads;
    }

    uint64_t getLastInterruptDuration_us()
    {
        return m_lastInterruptDuration_us;
    }

    bool isConnected(){
        return m_ads1115.isConnected();
    }

    ADS1115 m_ads1115;

private:
    // Catch interrupt and set flag
    static void adsReadyCallback(void *loadcell)
    {
        uint64_t start_time_us = time_us_64();
        gpio_put(Pins::ONBOARD_LED, true);
        LoadcellADS1115 *this_p = static_cast<LoadcellADS1115 *>(loadcell);
        this_p->m_isAdcReady = true;
        this_p->m_lastRawCounts = this_p->m_ads1115.getValue();
        this_p->m_noOfReads++;
        gpio_put(Pins::ONBOARD_LED, false);
        // Serial.printf("%llu\t", start_time_us);
        // Serial.printf("%llu\t", this_p->m_lastInterruptDuration_us);
        // Serial.printf("%ld\n", this_p->m_lastRawCounts);
        this_p->m_lastFilteredCounts = this_p->filter.step(this_p->m_lastRawCounts);
        this_p->m_lastInterruptDuration_us = time_us_64() - start_time_us;
        return;
    }

    const uint32_t m_readyPin;
    int32_t m_lastRawCounts = 0;
    double m_lastFilteredCounts = 0;
    uint64_t m_noOfReads = 0;
    uint64_t m_lastInterruptDuration_us = 0;
    Filter& filter;

    bool m_isAdcReady;
};

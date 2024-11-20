
#pragma once
#include <optional>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include "adc_sensor.h"
#include "filters.h"

#include "ADS1X15.h"

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

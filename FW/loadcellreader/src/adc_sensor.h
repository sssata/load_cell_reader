#pragma once
#include <optional>
#include <hardware/gpio.h>
#include <hardware/timer.h>

#include "ADS1X15.h"
#include "ADS1220_driver.h"

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "pico/util/queue.h"

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

using namespace ADS1220Driver;

class ADS1220Pipeline
{

public:

    enum class State
    {
        INITIALIZING = 0,
        WAITING_FOR_DRDY,
        READING_SPI,
        APPLYING_FILTER,
        ERROR,
    };

    struct PinSetup {
        uint8_t MOSI;
        uint8_t MISO;
        uint8_t SCK;
        uint8_t CS;
        uint8_t DRDY;
    };

    struct SensorReading {
        float reading;
        int32_t rawReading;
        uint64_t timestamp_us;
        uint64_t readingNumber;
        uint32_t interruptDuration_us;
        uint64_t lastValidReadingTimestamp_us;
    };

    /**
     * @brief Construct a new ADS1220 pipeline object
     * 
     * @param pinSetup
     * @param filter the filter to apply to the sensor readings
     * @param output_queue the output queue to store the sensor readings
     */
    ADS1220Pipeline(uint8_t ID, PinSetup pinSetup, Filter& filter, queue_t* output_queue) 
        : m_ID{ID}
        , m_pins{pinSetup}
        , filter{filter}
        , m_State{State::INITIALIZING}
        , m_SPI{spi0}
        , m_DataQueue{output_queue}
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

        m_ads1220 = ADS1220();

        // Fill both buffers with initial values
        m_ReadingBuffer[0] = {0};
        m_ReadingBuffer[1] = {0};
    }

    ~ADS1220Pipeline() {
        if (m_DMAChannel >= 0) {
            dma_channel_unclaim(m_DMAChannel);
        }
    }

    // Reset the ADS1220 and set up the configuration registers
    ErrorCode setup(){

        m_ads1220.begin(m_pins.CS,m_pins.DRDY);

        // Set gain
        m_ads1220.resetAllRegisters(false);
        m_ads1220.setRegister(0, MUX_AIN2_AIN3 | PGA_GAIN_1 | 0x00);  // Register 0: AIN2-AIN3, PGA Gain 1, PGA enabled
        m_ads1220.setRegister(1, DR_330SPS | MODE_TURBO | 0b00000100);  // Register 1: 660 SPS Turbo Mode, Continuous conversion mode, Temp Sensor disabled, Current Source off
        m_ads1220.setRegister(2, 0x00);  // Register 2: Vref internal, 50/60Hz rejection off, power switch open, IDAC off
        m_ads1220.setRegister(3, 0x00);  // Register 3: IDAC1 disabled, IDAC2 disabled, DRDY pin only
        m_ads1220.writeAllRegisters();

        return ErrorCode::OK;
    }

    ErrorCode begin()
    {   
        m_ads1220.Start_Conv();  // Start continuous conversion mode
        m_State = State::WAITING_FOR_DRDY;
        return ErrorCode::OK;
    }

    /**
     * @brief Get the Reading object
     * Thread-safe function to get the latest reading from the sensor
     * 
     * @param reading 
     * @return true 
     * @return false 
     */
    bool getReading(SensorReading& reading) const {
        // Atomically get current reading
        uint32_t read_index = __atomic_load_n(&m_BufferCurrentIndex, __ATOMIC_ACQUIRE);
        reading = m_ReadingBuffer[m_BufferCurrentIndex];
        return true;
    }

    uint8_t getId() const { return m_ID; }
    uint getDRDYPin() const { return m_pins.DRDY; }
    uint getCSPin() const { return m_pins.CS; }
    int getDMAChannel() const { return m_DMAChannel; }

    void __isr handleDRDY() {
        m_interruptStartTime_us = time_us_64();
        gpio_put(Pins::ONBOARD_LED, true);
        m_State = State::READING_SPI;
        if (!readDataToRawDataArray()){
            // enterErrorState();
            return;
        }

        if (!validateDataArray()){
            // enterErrorState();
            return;
        }

        rawDataArrayToCounts();

        m_State = State::APPLYING_FILTER;
        m_lastFilteredCounts = filter.step(m_lastRawCounts);
        m_noOfReads++;
        m_lastValidReadingTimestamp_us = m_interruptStartTime_us;

        SensorReading reading {
            .reading = static_cast<float>(m_lastFilteredCounts),
            .rawReading = m_lastRawCounts,
            .timestamp_us = m_interruptStartTime_us,
            .readingNumber = m_noOfReads,
            .interruptDuration_us = static_cast<uint32_t>(time_us_64() - m_interruptStartTime_us),
            .lastValidReadingTimestamp_us = m_lastValidReadingTimestamp_us,
        };
        updateReadingBuffer(reading);
        m_State = State::WAITING_FOR_DRDY;
        gpio_put(Pins::ONBOARD_LED, false);
    }

    uint8_t m_ID;
    ADS1220 m_ads1220;
    const PinSetup m_pins;

private:

    inline bool readDataToRawDataArray(){
        gpio_put(m_pins.CS, 0);  // Select sensor
        sleep_us(1); // Wait for CS to settle

        int bytesRead = spi_read_blocking(m_SPI, 0xFF , m_RawDataArray, 3);
        sleep_us(1);
        gpio_put(m_pins.CS, 1);  // Deselect sensor

        if (bytesRead != 3){
            // Serial.printf("Failed to read data from sensor, read %d bytes\n", bytesRead);
            return false;
        }
        return true;
    }

    inline bool validateDataArray(){
        // If all bits are 0 or 1, the data is invalid
        if (m_RawDataArray[0] == 0xFF && m_RawDataArray[1] == 0xFF && m_RawDataArray[2] == 0xFF){
            // Serial.println("All bits are 1");
            return false;
        }
        if (m_RawDataArray[0] == 0x00 && m_RawDataArray[1] == 0x00 && m_RawDataArray[2] == 0x00){
            // Serial.println("All bits are 0");
            return false;
        }
        return true;
    }

    inline void rawDataArrayToCounts(){
        int32_t result = 0;
        result = m_RawDataArray[0];
        result = (result << 8) | m_RawDataArray[1];
        result = (result << 8) | m_RawDataArray[2];

        // If the most significant bit is set, the value is negative
        if (m_RawDataArray[0] & (1<<7)) {
            result |= 0xFF000000;
        }
        m_lastRawCounts = result;
    }

    /**
     * @brief Make the given reading safetly available to read from other threads with getReading()
     * 
     * @param reading 
     */
    inline void updateReadingBuffer(SensorReading& reading){
        // Write to inactive buffer
        uint32_t write_index = 1 - __atomic_load_n(&m_BufferCurrentIndex, __ATOMIC_ACQUIRE);
        m_ReadingBuffer[write_index] = reading;
        
        // Atomic swap to make new reading active
        __atomic_store_n(&m_BufferCurrentIndex, write_index, __ATOMIC_RELEASE);
    }

    inline void printLastReading(){
        SensorReading reading;
        getReading(reading);
        Serial.printf("%llu\t%ld\n", reading.timestamp_us, reading.rawReading);
        Serial.flush();
    }

    void enterErrorState(){
        SensorReading reading {
            .reading = static_cast<float>(m_lastFilteredCounts),
            .rawReading = m_lastRawCounts,
            .timestamp_us = time_us_64(),
            .readingNumber = m_noOfReads,
            .interruptDuration_us = static_cast<uint32_t>(time_us_64() - m_interruptStartTime_us),
            .lastValidReadingTimestamp_us = m_lastValidReadingTimestamp_us,
        };
        m_State = State::ERROR;
        updateReadingBuffer(reading);
    }

    Filter& filter;
    int32_t m_lastRawCounts = 0;
    double m_lastFilteredCounts = 0;
    uint64_t m_noOfReads = 0;
    uint64_t m_interruptStartTime_us = 0;
    uint64_t m_lastValidReadingTimestamp_us = 0;


    volatile State m_State = State::INITIALIZING;
    volatile uint32_t m_RawReading;
    
    //  Double buffer the sensor readings for safe inter-thread communication
    SensorReading m_ReadingBuffer[2];
    volatile uint32_t  m_BufferCurrentIndex = 0;

    uint8_t m_RawDataArray[3];
    spi_inst_t* m_SPI = spi0;
    dma_channel_config m_DMAConfig;
    int m_DMAChannel = -1;
    queue_t *const m_DataQueue;
};

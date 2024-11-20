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

class ADS1220Pipeline : public AnalogSensor
{

public:

    enum class State
    {
        INITIALIZING = 0,
        WAITING_FOR_DRDY,
        READING_SPI,
        APPLYING_FILTER,
    };

    struct PinSetup{
        uint8_t MOSI;
        uint8_t MISO;
        uint8_t SCK;
        uint8_t CS;
        uint8_t DRDY;
    };

    struct SensorReading {
        float reading;
        int32_t rawReading;
        uint64_t timestamp;
        uint64_t reading_number;
        uint32_t interruptDuration_us;
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
        return ErrorCode::OK;
    }

    ErrorCode begin()
    {
        m_ads1220.Start_Conv();  // Start continuous conversion mode
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

    bool setUpPipeline(){
        // initializeDMA();
        return true;
    }

    bool getReading(SensorReading& reading) {
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
        gpio_put(m_pins.CS, 0);  // Select sensor
        sleep_us(1); // Wait for CS to settle

        // Start DMA transfer
        // dma_channel_start(m_DMAChannel);
        spi_read_blocking(m_SPI, 0xFF , m_RawReadingArray, 3);
        // m_RawReading = m_ads1220.Read_WaitForData();
        // spi_write_blocking(m_SPI, nullptr, 3);  // Dummy write to clock out the data
        gpio_put(m_pins.CS, 1);  // Deselect sensor
        sleep_us(1); // Wait for CS to settle

        handleDMAComplete();
    }

    uint8_t m_ID;
    ADS1220 m_ads1220;
    const PinSetup m_pins;

private:

    /* Pipeline Init Functions */
    
    // void initializeDMA() {
    //     // Claim a free DMA channel
    //     m_DMAChannel = dma_claim_unused_channel(false);
    //     if (m_DMAChannel < 0) {
    //         // Handle error - no free DMA channels
    //         printf("Error: No free DMA channel for sensor %d\n", m_ID);
    //         return;
    //     }

    //     // Configure DMA
    //     m_DMAConfig = dma_channel_get_default_config(m_DMAChannel);
    //     channel_config_set_transfer_data_size(&m_DMAConfig, DMA_SIZE_8);
    //     channel_config_set_read_increment(&m_DMAConfig, false);
    //     channel_config_set_write_increment(&m_DMAConfig, true);
    //     channel_config_set_dreq(&m_DMAConfig, spi_get_dreq(m_SPI, false));
    //     // dma_channel_configure(m_DMAChannel, &m_DMAConfig, false);
    //     dma_channel_configure(
    //         m_DMAChannel,
    //         &m_DMAConfig,
    //         &m_RawReadingFromDMA,   // dst
    //         &spi_get_hw(m_SPI)->dr, // src
    //         3,                      // 24 bits
    //         false                   // don't start immediately
    //     );

    //     // Enable DMA interrupt for this channel
    //     dma_channel_set_irq0_enabled(m_DMAChannel, true);
    // }

    /* Pipeline Callbacks */
    // static void drdyCallback(void* loadcell){
    //     uint64_t start_time_us = time_us_64();
    //     gpio_put(Pins::ONBOARD_LED, true);
    //     ADS1220Pipeline *self = static_cast<ADS1220Pipeline *>(loadcell);
    //     // self->m_lastRawCounts = self->m_ads1220.Read_Data_Samples();
    //     self->m_lastRawCounts = self->m_ads1220.Read_WaitForData();
    //     self->m_noOfReads++;
    //     self->m_lastFilteredCounts = self->filter.step(self->m_lastRawCounts);
    //     gpio_put(Pins::ONBOARD_LED, false);
    //     self->m_lastInterruptDuration_us = time_us_64() - start_time_us;
    // }

    // DRDY pin interrupt handler


    void __isr handleDMAComplete() {
        // gpio_put(m_pins.CS, 1);  // Deselect sensor
        int32_t result = 0;
        result = m_RawReadingArray[0];
        result = (result << 8) | m_RawReadingArray[1];
        result = (result << 8) | m_RawReadingArray[2];

        if (m_RawReadingArray[0] & (1<<7)) {
            result |= 0xFF000000;
        }

        // Process the reading
        m_lastRawCounts = result;
        float filtered_value = filter.step(m_lastRawCounts);
        
        m_noOfReads++;
        m_lastFilteredCounts = filtered_value;

        SensorReading reading {
            .reading = static_cast<float>(m_lastFilteredCounts),
            .rawReading = m_lastRawCounts,
            .timestamp = time_us_64(),
            .reading_number = m_noOfReads,
            .interruptDuration_us = static_cast<uint32_t>(time_us_64() - m_interruptStartTime_us)
        };
        // Write to inactive buffer
        uint32_t write_index = 1 - __atomic_load_n(&m_BufferCurrentIndex, __ATOMIC_ACQUIRE);
        m_ReadingBuffer[write_index] = reading;
        
        // Atomic swap to make new reading active
        __atomic_store_n(&m_BufferCurrentIndex, write_index, __ATOMIC_RELEASE);
        gpio_put(Pins::ONBOARD_LED, false);
    }

    Filter& filter;
    int32_t m_lastRawCounts = 0;
    double m_lastFilteredCounts = 0;
    uint64_t m_noOfReads = 0;
    uint64_t m_lastInterruptDuration_us = 0;
    uint64_t m_interruptStartTime_us = 0;


    volatile State m_State = State::INITIALIZING;
    volatile uint32_t m_RawReading;
    
    //  Double buffer the sensor readings for safe inter-thread communication
    SensorReading m_ReadingBuffer[2];
    volatile uint32_t  m_BufferCurrentIndex = 0;

    uint8_t m_RawReadingArray[3];
    spi_inst_t* m_SPI = spi0;
    dma_channel_config m_DMAConfig;
    int m_DMAChannel = -1;
    queue_t *const m_DataQueue;
};

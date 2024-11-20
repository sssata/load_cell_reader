// Multi-sensor ADS1220 reading with interrupts and DMA
#pragma once

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "pico/util/queue.h"

#include "adc_sensor.h"

#include <array>
#include <vector>
#include <memory>

class ADCManager {
    // Pipeline:
    // 1. DRDY interrupt triggers DMA transfer
    // 2. DMA transfer reads 3 bytes from the sensor
    // 3. DMA completion triggers digital filtering
    // 4. Digital filtering stores the result in a queue

public:

    // Singleton
    static ADCManager& getInstance() {
        static ADCManager instance;
        return instance;
    }

    ADCManager() = default;
    ~ADCManager() = default;
    ADCManager(const ADCManager&) = delete;
    ADCManager& operator=(const ADCManager&) = delete;

    void addADCPipeline(ADS1220Pipeline* sensor) {
        m_Sensors.push_back(sensor);
    }
    
    /**
     * @brief Set up interrupts and write the configuration to the sensors
     * Must add the sensors with addADCPipeline() before calling setup()
     * 
     */
    void setup() {
        for (auto sensor : m_Sensors) {
            sensor->setup();
            Serial.println("ADS1220 setup complete\n");
        }
        setupInterruptHandlers();
    }

    /**
     * @brief Begin reading from the sensors
     * Must call setup() before calling begin()
     * 
     */
    void begin() {
        Serial.println("ADCManager begin");

        for (auto sensor : m_Sensors) {
            sensor->begin();
            sleep_us(100); // Wait for the sensor to initialize
        }
        Serial.println("ADCManager begin complete");
    }

private:

    void setupInterruptHandlers() {
        // Set up DRDY interrupt handlers
        gpio_set_irq_callback([](uint gpio, uint32_t events) {
            ADCManager::getInstance().handleDRDYInterrupt(gpio);
        });
        
        for (auto sensor : m_Sensors) {
            gpio_set_irq_enabled(sensor->getDRDYPin(), GPIO_IRQ_EDGE_FALL, true);
        }

        irq_set_enabled(IO_IRQ_BANK0, true);

        // attachInterruptParam(m_Sensors[0]->getDRDYPin(), [](void* param) {
        //     ADCManager::getInstance().handleDRDYInterrupt(m_Sensors[0]->getDRDYPin());
        // }, FALLING, nullptr);
    }

    void setupSPI() {
        spi_init(m_SPI, 2000000);
        spi_set_format(m_SPI, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
        spi_set_slave(m_SPI, false);
        gpio_set_function(m_Sensors[0]->m_pins.MISO, GPIO_FUNC_SPI);
        gpio_set_function(m_Sensors[0]->m_pins.MOSI, GPIO_FUNC_SPI);
        gpio_set_function(m_Sensors[0]->m_pins.SCK, GPIO_FUNC_SPI);
    }

    void __isr handleDRDYInterrupt(uint gpio) {
        // Serial.println("DRDY Interrupt");
        // Find which sensor triggered the interrupt
        for (auto sensor : m_Sensors) {
            if (sensor->getDRDYPin() == gpio) {
                sensor->handleDRDY();
                break;
            }
        }
    }

    // DMA completion interrupt handler
    // void dma_handler() {
    //     // Clear the interrupt request
    //     dma_hw->ints0 = 1u << dma_rx_chan;
    //     dma_complete = true;
        
    //     // Trigger the digital filtering interrupt
    //     irq_set_pending(TIMER_IRQ_0);
    // }

    spi_inst_t* m_SPI = spi0;

    std::vector<ADS1220Pipeline*> m_Sensors;

    // std::array<uint, N> dma_channels;

};
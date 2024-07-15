//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino library for the ADS1220 24-bit ADC breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//
//    Based on original code from Texas Instruments
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/Protocentral/Protocentral_ADS1220/
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "ADS1220_driver.h"
#include <SPI.h>
#include <limits.h>

using namespace ADS1220Driver;

//#define BOARD_SENSYTHING ST_1_3


#ifdef _BV
#undef _BV
#endif

#define _BV(bit) (1<<(bit))

SPISettings SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE1); 


ADS1220::ADS1220() 								// Constructors
{

}

void ADS1220::writeRegister(uint8_t address, uint8_t value)
{
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(m_cs_pin,LOW);
    delayMicroseconds(1);
    SPI.transfer(WREG|(address<<2));
    SPI.transfer(value);
    delayMicroseconds(1);
    digitalWrite(m_cs_pin,HIGH);
    SPI.endTransaction();
}

uint8_t ADS1220::readRegister(uint8_t address)
{
    uint8_t data;

    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(m_cs_pin,LOW);
    delayMicroseconds(1);
    SPI.transfer(RREG|(address<<2));
    data = SPI.transfer(SPI_MASTER_DUMMY);
    delayMicroseconds(1);
    digitalWrite(m_cs_pin,HIGH);
    SPI.endTransaction();

    return data;
}

bool ADS1220::isConnected() {
    
    return readRegister(CONFIG_REG0_ADDRESS) == 0x00;
}

void ADS1220::begin(uint8_t cs_pin, uint8_t drdy_pin)
{
    m_drdy_pin=drdy_pin;
    m_cs_pin=cs_pin;

    pinMode(m_cs_pin, OUTPUT);
    pinMode(m_drdy_pin, INPUT_PULLUP);

    SPI.begin();
    ads1220_Reset();
    delayMicroseconds(50);
    
    // The device pulls nDRDY low after it is initialized
    while (!WaitForData(1)){
        // ads1220_Reset();
        // delayMicroseconds(50);
        Serial.println("Failed to wait for data");
        Serial.flush();
        delay(1000);
    }

    m_config_reg0 = 0x00;   //Default settings: AINP=AIN0, AINN=AIN1, Gain 1, PGA enabled
    m_config_reg1 = 0x04;   //Default settings: DR=20 SPS, Mode=Normal, Conv mode=continuous, Temp Sensor disabled, Current Source off
    m_config_reg2 = 0x10;   //Default settings: Vref internal, 50/60Hz rejection, power open, IDAC off
    m_config_reg3 = 0x00;   //Default settings: IDAC1 disabled, IDAC2 disabled, DRDY pin only

    writeRegister( CONFIG_REG0_ADDRESS , m_config_reg0);
    writeRegister( CONFIG_REG1_ADDRESS , m_config_reg1);
    writeRegister( CONFIG_REG2_ADDRESS , m_config_reg2);
    writeRegister( CONFIG_REG3_ADDRESS , m_config_reg3);
}

void ADS1220::PrintRegisterValues(){
    Config_Reg0 = readRegister(CONFIG_REG0_ADDRESS);
    Config_Reg1 = readRegister(CONFIG_REG1_ADDRESS);
    Config_Reg2 = readRegister(CONFIG_REG2_ADDRESS);
    Config_Reg3 = readRegister(CONFIG_REG3_ADDRESS);

    Serial.println("Config_Reg : ");
    Serial.println(Config_Reg0,HEX);
    Serial.println(Config_Reg1,HEX);
    Serial.println(Config_Reg2,HEX);
    Serial.println(Config_Reg3,HEX);
    Serial.println(" ");
}

void ADS1220::SPI_Command(unsigned char data_in)
{
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(m_cs_pin, LOW);
    delayMicroseconds(1);
    auto data = SPI.transfer(data_in);
    Serial.printf("SPI_Command: %x\n", data);
    delayMicroseconds(1);
    digitalWrite(m_cs_pin, HIGH);
    SPI.endTransaction();
}

void ADS1220::ads1220_Reset()
{
    SPI_Command(RESET);
}

void ADS1220::Start_Conv()
{
    SPI_Command(START);
}

// control register 0
void ADS1220::select_mux_channels(int channels_conf)
{
    m_config_reg0 &= ~REG_CONFIG0_MUX_MASK;
    m_config_reg0 |= channels_conf;
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

void ADS1220::set_pga_gain(int pgagain)
{
    m_config_reg0 &= ~REG_CONFIG0_PGA_GAIN_MASK;
    m_config_reg0 |= pgagain ;
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

void ADS1220::PGA_ON(void)
{
    m_config_reg0 &= ~_BV(0);
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

void ADS1220::PGA_OFF(void)
{
    m_config_reg0 |= _BV(0);
    writeRegister(CONFIG_REG0_ADDRESS,m_config_reg0);
}

// control register 1
void ADS1220::set_data_rate(int datarate)
{
    m_config_reg1 &= ~REG_CONFIG1_DR_MASK;
    m_config_reg1 |= datarate;
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void ADS1220::set_OperationMode(int OPmode)
{
    m_config_reg1 &= ~REG_CONFIG1_MODE_MASK;
    m_config_reg1 |= OPmode;
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void ADS1220::set_conv_mode_single_shot(void)
{
    m_config_reg1 &= ~_BV(2);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void ADS1220::set_conv_mode_continuous(void)
{
    m_config_reg1 |= _BV(2);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void ADS1220::TemperatureSensorMode_disable(void)
{
    m_config_reg1 &= ~_BV(1);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void ADS1220::TemperatureSensorMode_enable(void)
{
    m_config_reg1 |= _BV(1);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void ADS1220::CurrentSources_OFF(void)
{
    m_config_reg1 &= ~_BV(0);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

void ADS1220::CurrentSources_ON(void)
{
    m_config_reg1 |= _BV(0);
    writeRegister(CONFIG_REG1_ADDRESS,m_config_reg1);
}

// control register 2
void ADS1220::set_VREF(int vref)
{
    m_config_reg2 &= ~REG_CONFIG2_VREF_MASK;
    m_config_reg2 |= vref;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void ADS1220::set_FIR_Filter(int filter)
{
    m_config_reg2 &= ~REG_CONFIG2_FIR_MASK;
    m_config_reg2 |= filter;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void ADS1220::LowSideSwitch_OPEN(void)
{
    m_config_reg2 &= ~_BV(3);
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void ADS1220::LowSideSwitch_CLOSED(void)
{
    m_config_reg2 |= _BV(3);
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void ADS1220::set_IDAC_Current(int IDACcurrent)
{
    m_config_reg2 &= ~REG_CONFIG2_IDACcurrent_MASK;
    m_config_reg2 |= IDACcurrent;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

// control register 3
void ADS1220::set_IDAC1_Route(int IDAC1routing)
{
    m_config_reg3 &= ~REG_CONFIG3_IDAC1routing_MASK;
    m_config_reg3 |= IDAC1routing;
    writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
}

void ADS1220::set_IDAC2_Route(int IDAC2routing)
{
    m_config_reg3 &= ~REG_CONFIG3_IDAC2routing_MASK;
    m_config_reg3 |= IDAC2routing;
    writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
}

 void ADS1220::DRDYmode_default(void)
 {
     m_config_reg3 &= ~_BV(3);
     writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
 }

 void ADS1220::DRDYmode_DOUT(void)
 {
     m_config_reg3 |= _BV(3);
     writeRegister(CONFIG_REG3_ADDRESS,m_config_reg3);
 }
// end control register

uint8_t * ADS1220::get_config_reg()
{
    static uint8_t config_Buff[4];

    m_config_reg0 = readRegister(CONFIG_REG0_ADDRESS);
    m_config_reg1 = readRegister(CONFIG_REG1_ADDRESS);
    m_config_reg2 = readRegister(CONFIG_REG2_ADDRESS);
    m_config_reg3 = readRegister(CONFIG_REG3_ADDRESS);

    config_Buff[0] = m_config_reg0 ;
    config_Buff[1] = m_config_reg1 ;
    config_Buff[2] = m_config_reg2 ;
    config_Buff[3] = m_config_reg3 ;

    return config_Buff;
}

bool ADS1220::WaitForData(unsigned int timeout_ms){
    while(digitalRead(m_drdy_pin)){
        if(timeout_ms){
            delay(1);
            --timeout_ms;
        } else {
            return false;
        }
    }
    return true;
}

uint8_t * ADS1220::Read_Data(void){
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(m_cs_pin, LOW);                         //Take CS low
    delayMicroseconds(1);
    for (int i = 0; i < 3; i++)
    {
        DataReg[i] = SPI.transfer(0);
    }
    delayMicroseconds(1);
    digitalWrite(m_cs_pin, HIGH);                  //  Clear CS to high
    SPI.endTransaction();

    return DataReg;
}

int32_t ADS1220::DataToInt(){
    int32_t result = 0;
    result = DataReg[0];
    result = (result << 8) | DataReg[1];
    result = (result << 8) | DataReg[2];

    if (DataReg[0] & (1<<7)) {
        result |= 0xFF000000;
    }

    return result;
}

int32_t ADS1220::Read_WaitForData()
{
    if(!WaitForData(60)){
        //return std::numeric_limits<int32_t>::min();
        return 0;
    }
    Read_Data();
    return DataToInt();
}

int32_t ADS1220::Read_Data_Samples()
{
    static byte SPI_Buff[3];
    int32_t mResult32=0;
    long int bit24;

    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(m_cs_pin, LOW);                         //Take CS low
    delayMicroseconds(1);
    for (int i = 0; i < 3; i++)
    {
      SPI_Buff[i] = SPI.transfer(SPI_MASTER_DUMMY);
    }
    delayMicroseconds(1);
    digitalWrite(m_cs_pin, HIGH);                  //  Clear CS to high
    SPI.endTransaction();

    bit24 = SPI_Buff[0];
    bit24 = (bit24 << 8) | SPI_Buff[1];
    bit24 = (bit24 << 8) | SPI_Buff[2];                                 // Converting 3 bytes to a 24 bit int

    bit24= ( bit24 << 8 );
    mResult32 = ( bit24 >> 8 );                      // Converting 24 bit two's complement to 32 bit two's complement

    return mResult32;
}

int32_t ADS1220::Read_SingleShot_WaitForData(void)
{
    Start_Conv();
    return Read_WaitForData();
}

int32_t ADS1220::Read_SingleShot_SingleEnded_WaitForData(uint8_t channel_no)
{
    select_mux_channels(channel_no);
    return Read_SingleShot_WaitForData();
}

#define VREF_MASK ((1 << 6) | (1<<7))
#define VREF_INT (0 << 6)
#define VREF_EXT (1 << 6)

void ADS1220::internal_reference(){
    m_config_reg2 &= ~VREF_MASK;
    m_config_reg2 |= VREF_INT;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

void ADS1220::external_reference(){
    m_config_reg2 &= ~VREF_MASK;
    m_config_reg2 |= VREF_EXT;
    writeRegister(CONFIG_REG2_ADDRESS,m_config_reg2);
}

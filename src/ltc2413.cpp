#include "LTC2413.hpp"

LTC2413::LTC2413(uint8_t cs_pin, float voltage_reference)
    : m_csPin(cs_pin),
    m_EoC(false),
    m_spiSettings(1000000, MSBFIRST, SPI_MODE0),
    m_convFactors(voltage_reference)
{
    pinMode(m_csPin, OUTPUT);
    digitalWrite(m_csPin, HIGH);

    SPI.begin();
}

LTC2413::~LTC2413()
{
    SPI.end();
}

bool LTC2413::isConvReady()
{
    if (!m_EoC) {
        digitalWrite(m_csPin, LOW);
        m_EoC = (digitalRead(MISO)) ? false : true;
        digitalWrite(m_csPin, HIGH);
    }

    return m_EoC;
}

void LTC2413::calibrateConversion(const TwoPointCalibration& cal)
{
    float _refVoltageRange, _outputCodeRange, _offset;

    // Calculating ADC step size based on measured cal voltage and actual output code
    _refVoltageRange = (cal.rpHigh.refVoltage - cal.rpLow.refVoltage);
    _outputCodeRange = (float)(cal.rpHigh.outputCode - cal.rpLow.outputCode);
    m_convFactors.StepSize = _refVoltageRange / _outputCodeRange;

    // Calculating zero offset
    _offset = (cal.rpLow.refVoltage / m_convFactors.StepSize) - (float)(cal.rpLow.outputCode);
    m_convFactors.ZeroOffset = round(_offset);
}

bool LTC2413::clearConversion()
{
    if (m_EoC) 
    {
        // ADC wil start a new conversion if the CS pin is pulled HIGH after 5 or more bits have been shifted out.
        SPI.beginTransaction(m_spiSettings);
        digitalWrite(m_csPin, LOW);
        SPI.transfer(0xFF);
        digitalWrite(m_csPin, HIGH);
        SPI.endTransaction();
        m_EoC = false;

        return true;
    }

    return false;
}

int32_t LTC2413::getConversion()
{
    switch(m_convFactors.Polarity)
    {
        case ConvPolarityType::UNIPOLAR: return (read_device() >> m_convFactors.BitShift) - m_convFactors.MaximumCode;
        case ConvPolarityType::BIPOLAR: return (read_device() >> m_convFactors.BitShift) - (m_convFactors.MaximumCode - m_convFactors.MinimumCode);
    }

    return m_convFactors.MinimumCode;
}

int32_t LTC2413::read_device()
{
    if (m_EoC)
    {
        ConvBufferType _conv;
    
        // Get 32 bits of data from SPI bus (2x 16 bit transfers)
        SPI.beginTransaction(m_spiSettings);
        digitalWrite(m_csPin, LOW);
        _conv.buffer_16bit[1] = SPI.transfer16(0xFFFF);
        _conv.buffer_16bit[0] = SPI.transfer16(0xFFFF);
        digitalWrite(m_csPin, HIGH);
        SPI.endTransaction();
        m_EoC = false;
    
        return _conv.buffer_32bit;
    }

    return 0;
}


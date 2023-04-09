/*
LTC2413 Serial Interface
SPI Mode = 0

SPI Data Format (MSB First)

                      BYTE #1                                            BYTE #2                                      
|| EoC | DMY | SIG | B28 | B27 | B26 | B25 | B24  ||  B23 | B22 | B21 | B20 | B19 | B18 | B17 | B16  || 

                      BYTE #3                                            BYTE #4
|| B15 | B14 | B13 | B12 | B11 | B10 | B09 | B08  ||  B07 | B06 | B05 | B04 | B03 | B02 | B01 | B00  ||

 EoC :: End of Conversion Bit (Active Low)
 DMY  :: Dummy Bit (Always 0)
 SIG  :: Sign Bit (1 = positive, 0 = negative)
 DXX  :: Data Bits
*/
#ifndef LTC2413_hpp
#define LTC2413_hpp

#include <Arduino.h>
#include <SPI.h>

#define LIMIT(val, min, max)   (val < min) ? min : (val > max) ? max : val

struct TwoPointCalibration
{
    struct ReferencePoint 
    {
        float refVoltage;
        int32_t outputCode;
    } rpLow, rpHigh;

    const void setReference(const float& ref_voltage, const int32_t& output_code, ReferencePoint& ref_point)
    {
        ref_point.refVoltage = ref_voltage;
        ref_point.outputCode = output_code;
    }
};

class LTC2413 
{
public:
    enum class ConvPolarityType
    {
        UNIPOLAR = 1,
        BIPOLAR
    };

    // Constructor
    explicit LTC2413(uint8_t pin_CS, float voltage_reference);
    ~LTC2413();
            
    // Initializes device
    void init();

    // Checks and sets the end of conversion flag
    bool isConvReady();
    
    void setClockFreq(uint8_t clock_freq);

    // Sets conversion output polarity
    void setConvPolarity(ConvPolarityType conv_polarity) { m_convFactors.Polarity = conv_polarity; }

    void setConvResolution(uint8_t conv_resolution) { m_convFactors.setResolution(conv_resolution); }

    void calibrateConversion(const TwoPointCalibration& cal);

    // Clear previous conversion from the device, starts a new conversion
    // Return:: True if cleared successfully, False on failure
    bool clearConversion();

    // Reads ADC and outputs the result as a integer code
    int32_t getConversion();

    // Reads ADC and outputs result as a floating point voltage
    inline float toVoltage(int32_t conversion) const { return ((float)(conversion) + m_convFactors.ZeroOffset) * m_convFactors.StepSize; }

private:
    const uint8_t m_csPin;
    bool m_initialized, m_EoC;
    SPISettings m_spiSettings;

    struct ConvFactorsType
    {
        float VRef_P, VRef_N, StepSize;
        int32_t MinimumCode, MaximumCode, ZeroOffset;
        uint8_t BitShift;
        ConvPolarityType Polarity;

        ConvFactorsType(float voltage_reference, uint8_t resolution = 24u, ConvPolarityType polarity = ConvPolarityType::BIPOLAR)
        {
            VRef_P = LIMIT(voltage_reference, 0, 5) / 2.0f;
            VRef_N = VRef_P * (-1.0f);
            ZeroOffset = 0;

            setResolution(resolution);
            
            Polarity = polarity;
        }

        void setResolution(uint8_t resolution)
        {
            resolution = LIMIT(resolution, 8u, 29u);

            // 32 bit coming from ADC - 3 status bits = 29 bits of actual data
            // Subtract number of bits we want from actual number of incoming bits to get the number of bits to shift out
            BitShift = (29u - resolution);

            // Calculate the max/min output code based on the selected resolution and make bi-polar
            MaximumCode = (int32_t)(pow(2, resolution)) / 2;
            MinimumCode = MaximumCode * (-1);

            // Calculate the ADC step size
            StepSize = ((VRef_P - VRef_N) / (float)(MaximumCode - MinimumCode));
        }
    };

    ConvFactorsType m_convFactors; 

private:
    // Gets the previous conversion data from the device and starts a new conversion
    int32_t read_device();

    union ConvBufferType
    {
        uint16_t    buffer_16bit[2];
        int32_t     buffer_32bit;
    };
};

class Calibration 
{
public:
    float   stepSize;
    int32_t zeroOffset;
    uint16_t eepromAddress;
    
    struct ReferencePoint {
        float refVoltage;
        int32_t rawValue;
    } referenceLow, referenceHigh;

    Calibration(const ReferencePoint& ref_low, const ReferencePoint& ref_high)
    {
        float _offset, _refVoltageRange, _rawValueRange;

        _refVoltageRange = ref_high.refVoltage - ref_low.refVoltage;
        _rawValueRange = static_cast<float>(ref_high.rawValue) - static_cast<float>(ref_low.rawValue);
        stepSize = _refVoltageRange / _rawValueRange;

        _offset = (ref_low.refVoltage / stepSize) - static_cast<float>(ref_low.rawValue);
        zeroOffset = round(_offset);
    }

    inline const float ToVoltage(const int32_t& raw_value) const
    {
        return (static_cast<float>(raw_value + zeroOffset) * stepSize);
    }

    inline const int32_t ToCode(const float& voltage) const
    {
        return round(voltage / stepSize);
    }
};

#endif  // LTC2413_hpp
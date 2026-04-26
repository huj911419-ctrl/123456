#include "Ad.h"

float get_adc_voltage(void)
{
    static uint8 adc_inited = 0;
    if (!adc_inited)
    {
        adc_init(ADC0_CH11_A11, ADC_8BIT);
        adc_inited = 1;
    }

    uint16 adc_filtered = adc_mean_filter_convert(ADC0_CH11_A11, 5);
    float real_volt = adc_filtered * 3.3f / 255.0f * 8.658f;
    if (real_volt < 5.0f)
    {
        return 0.0f;
    }
    return real_volt;
}

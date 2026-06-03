#include "Battery.h"

#define BATTERY_ADC_CH      ADC0_CH11_A11
#define BATTERY_LOW_THRESH  11.1f
#define BATTERY_FULL_SCALE  12.6f
#define BATTERY_CHECK_DIV   100u
#define BATTERY_FILTER_OLD  0.85f
#define BATTERY_FILTER_NEW  0.15f

static float s_bat_voltage = 0.0f;
static uint8  s_bat_low = 0u;
static uint16 s_bat_cnt = 0u;
static uint8  s_bat_valid = 0u;

void battery_init(void)
{
    adc_init(BATTERY_ADC_CH, ADC_12BIT);
}

void battery_check(void)
{
    float voltage;
    s_bat_cnt++;
    if (s_bat_cnt < BATTERY_CHECK_DIV)
        return;
    s_bat_cnt = 0u;

    uint16 adc_val = adc_convert(BATTERY_ADC_CH);
    voltage = BATTERY_FULL_SCALE * (float)adc_val / 4095.0f;
    if (s_bat_valid == 0u)
    {
        s_bat_voltage = voltage;
        s_bat_valid = 1u;
    }
    else
    {
        s_bat_voltage = s_bat_voltage * BATTERY_FILTER_OLD + voltage * BATTERY_FILTER_NEW;
    }
    s_bat_low = (s_bat_voltage < BATTERY_LOW_THRESH) ? 1u : 0u;
}

float battery_get_voltage(void)
{
    return s_bat_voltage;
}

uint8 battery_is_low(void)
{
    return s_bat_low;
}

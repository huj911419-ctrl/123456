#include "Battery.h"

#define BATTERY_ADC_CH      ADC0_CH11_A11
#define BAT_ADC_FULL_SCALE  4096u
#define BAT_VOLT_X10_MAX    363u
#define BAT_AVG_SAMPLES     32u
#define BATTERY_LOW_X10     115u
#define BATTERY_LOW_REL_X10 117u
#define BATTERY_CHECK_DIV   1u
#define BAT_FILTER_SHIFT    4u
#define BAT_CAL_NUM         1045u
#define BAT_CAL_DEN         1000u

static float s_bat_voltage = 0.0f;
static uint16 s_bat_adc = 0u;
static uint16 s_bat_voltage_x10 = 0u;
static uint8  s_bat_low = 0u;
static uint16 s_bat_cnt = 0u;
static uint32 s_bat_adc_filt = 0u;
static uint8  s_bat_filt_valid = 0u;

static void battery_sample_once(void)
{
    uint16 raw_adc;
    uint32 mv10;

    raw_adc = adc_mean_filter_convert(BATTERY_ADC_CH, BAT_AVG_SAMPLES);
    if (!s_bat_filt_valid)
    {
        s_bat_adc_filt = ((uint32)raw_adc << BAT_FILTER_SHIFT);
        s_bat_filt_valid = 1u;
    }
    else
    {
        s_bat_adc_filt += (uint32)raw_adc;
        s_bat_adc_filt -= (s_bat_adc_filt >> BAT_FILTER_SHIFT);
    }

    s_bat_adc = (uint16)((s_bat_adc_filt + (1u << (BAT_FILTER_SHIFT - 1u))) >> BAT_FILTER_SHIFT);
    mv10 = ((uint32)s_bat_adc * (uint32)BAT_VOLT_X10_MAX +
            (uint32)(BAT_ADC_FULL_SCALE / 2u)) /
           (uint32)BAT_ADC_FULL_SCALE;
    mv10 = (mv10 * (uint32)BAT_CAL_NUM + (uint32)(BAT_CAL_DEN / 2u)) /
           (uint32)BAT_CAL_DEN;
    s_bat_voltage_x10 = (uint16)mv10;
    s_bat_voltage = (float)s_bat_voltage_x10 * 0.1f;
    if (s_bat_low)
    {
        if (s_bat_voltage_x10 >= BATTERY_LOW_REL_X10)
        {
            s_bat_low = 0u;
        }
    }
    else if (s_bat_voltage_x10 <= BATTERY_LOW_X10)
    {
        s_bat_low = 1u;
    }

    if (s_bat_low)
    {
        gpio_set_level(P33_4, GPIO_HIGH);
        gpio_set_level(P33_5, GPIO_HIGH);
        gpio_set_level(P33_6, GPIO_HIGH);
    }
}

void battery_init(void)
{
    adc_init(BATTERY_ADC_CH, ADC_12BIT);
    battery_sample_once();
}

void battery_check(void)
{
    s_bat_cnt++;
    if (s_bat_cnt < BATTERY_CHECK_DIV)
        return;
    s_bat_cnt = 0u;

    battery_sample_once();
}

float battery_get_voltage(void)
{
    return s_bat_voltage;
}

uint16 battery_get_voltage_x10(void)
{
    return s_bat_voltage_x10;
}

uint16 battery_get_adc(void)
{
    return s_bat_adc;
}

uint8 battery_is_low(void)
{
    return s_bat_low;
}

#ifndef CODE_BATTERY_H_
#define CODE_BATTERY_H_

#include "zf_common_headfile.h"

void battery_init(void);
void battery_check(void);
float battery_get_voltage(void);
uint16 battery_get_voltage_x10(void);
uint16 battery_get_adc(void);
uint8 battery_is_low(void);

#endif

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 0 = debug/tuning mode, 1 = race mode. */
#ifndef RACE_MODE
#define RACE_MODE 0
#endif

/* Runtime quiet mode default: 0 keeps TFT/UART/menu keys on while running. */
#define RUN_QUIET_DEFAULT_ENABLE 0
/* Safety key kept alive in quiet mode: 1=KEY1, 2=KEY2, 3=KEY3, 4=KEY4. */
#define RUN_QUIET_STOP_KEY      1u

/* UART0 tuning stream, active only when RACE_MODE=0 and quiet mode is off.
 * Params/telemetry stay every vision frame; image/edges are throttled to cut
 * serial load while still giving enough pictures for offline diagnosis. */
#define UART0_DEBUG_ENABLE      1u
#define UART0_IMAGE_DIV         8u
#define UART0_EDGES_DIV         8u
#define UART0_PARAMS_DIV        1u
#define UART0_TELEMETRY_DIV     1u

/* 1 = turn on vacuum PWM whenever motor_enable is on. */
#define VACUUM_RUN_ENABLE 1u

/* Otsu threshold update interval. Larger values reduce CPU cost. */
#define TF_OTSU_INTERVAL_DEBUG 2u
#define TF_OTSU_INTERVAL_RACE  4u

/* Optional fastest mode for stable lighting: 1 uses fixed threshold only. */
#define TF_FIXED_THRESHOLD_ENABLE 0
#define TF_FIXED_THRESHOLD_VALUE  70

#endif /* APP_CONFIG_H */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 0 = debug/tuning mode, 1 = race mode. */
#ifndef RACE_MODE
#define RACE_MODE 0
#endif

/* 1 = skip UART0 image packets while the motor is enabled. */
#define UART0_IMAGE_OFF_WHEN_RUNNING 1u

/* 1 = turn on vacuum PWM whenever motor_enable is on. */
#define VACUUM_RUN_ENABLE 1u

/* Otsu threshold update interval. Larger values reduce CPU cost. */
#define TF_OTSU_INTERVAL_DEBUG 4u
#define TF_OTSU_INTERVAL_RACE  8u

/* Optional fastest mode for stable lighting: 1 uses fixed threshold only. */
#define TF_FIXED_THRESHOLD_ENABLE 0
#define TF_FIXED_THRESHOLD_VALUE  75

#endif /* APP_CONFIG_H */

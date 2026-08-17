// config.h

#define DEBUG

// === НАСТРОЙКИ FAILSAFE ===
#define FAILSAFE_TIMEOUT 500
#define FAILSAFE_CH1 PULSE_CENTER
#define FAILSAFE_CH2 PULSE_CENTER
#define FAILSAFE_CH3 PULSE_MIN
#define FAILSAFE_CH4 PULSE_CENTER
#define FAILSAFE_CH5 PULSE_MIN
#define FAILSAFE_CH6 PULSE_MIN
#define FAILSAFE_CH7 PULSE_MIN
#define FAILSAFE_CH8 PULSE_MIN

// === НАСТРОЙКИ СГЛАЖИВАНИЯ (0 = выключено) ===
#define SMOOTH_STEP_CH1 0     // Серво 1 - без сглаживания
#define SMOOTH_STEP_CH2 0     // Серво 2 - без сглаживания
#define SMOOTH_STEP_CH3 50    // Мотор 1 - плавно (50 мкс/шаг)
#define SMOOTH_STEP_CH4 0     // Серво 3 - без сглаживания

/**
 * @file watchdog.h
 * @author Andrew K. Walker
 * @date 13 JUL 2017
 * @brief Watchdog implementation in case our micro goes out to lunch...
 */
#include "stm32f4xx_conf.h"


#define WATCHDOG_RESET_COUNT  0x7F
#define WATCHDOG_WINDOW_COUNT 0x70

void watchdog_init(void);
void watchdog_tickle(void);

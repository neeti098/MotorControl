#ifndef LOGGER_H
#define LOGGER_H

/* Logging: LOGI / LOGW / LOGE / LOGD(TAG, fmt, ...). Backed by printf.
   UART/VCP routing belongs in platforms/stm32/, not here. */
#include <stdio.h>

#define LOGI(tag, fmt, ...) printf("I %s: " fmt "\n", (tag), ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) printf("W %s: " fmt "\n", (tag), ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) printf("E %s: " fmt "\n", (tag), ##__VA_ARGS__)
#define LOGD(tag, fmt, ...) printf("D %s: " fmt "\n", (tag), ##__VA_ARGS__)

#endif /* LOGGER_H */

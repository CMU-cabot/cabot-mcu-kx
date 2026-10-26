#pragma once
using SemaphoreHandle_t = int;
constexpr int portMAX_DELAY = 0;
inline void xSemaphoreTake(int, int) {}
inline void xSemaphoreGive(int) {}
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)

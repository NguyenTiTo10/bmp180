#include "bsp_timer.h"

void bsp_timer_delay (uint32_t time)
{
  vTaskDelay(time / portTICK_PERIOD_MS);
}

void bsp_timer_ets_delay_us (uint32_t time_us)
{
  ets_delay_us(time_us);
}

int64_t bsp_timer_get_time (void)
{
  return esp_timer_get_time();
}


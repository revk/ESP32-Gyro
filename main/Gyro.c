// Gyro app

static const char __attribute__((unused)) TAG[] = "Gyro";

#include "revk.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include <driver/gpio.h>
#include <driver/i2c.h>
#include "led_strip.h"
#include "math.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "halib.h"

#define LEDS	16
led_strip_handle_t strip = NULL;

struct
{
   uint8_t bye:1;
} b = { 0 };

const char *
app_callback (int client, const char *prefix, const char *target, const char *suffix, jo_t j)
{
   return NULL;
}

void
revk_state_extra (jo_t j)
{

}

void
i2c_task (void *p)
{

   while (1)
   {
      sleep (1);
   }
}

void
led_task (void *p)
{
   revk_gpio_output (lede, 1);
   led_strip_config_t strip_config = {
      .strip_gpio_num = led.num,
      .max_leds = LEDS + 1,     // The number of LEDs in the strip,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
      .led_model = LED_MODEL_WS2812,    // LED strip model
      .flags.invert_out = led.invert,   // whether to invert the output signal(useful when your hardware has a level inverter)
   };
   led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,   // different clock source can lead to different power consumption
      .resolution_hz = 10 * 1000 * 1000,        // 10 MHz
      .flags.with_dma = true,
   };
   if (led_strip_new_rmt_device (&strip_config, &rmt_config, &strip))
   {
      ESP_LOGE (TAG, "LED fail %d", led.num);
      vTaskDelete (NULL);
      return;
   }
   ESP_LOGE (TAG, "LED GPIO %d", led.num);
   while (!b.bye)
   {
      revk_led (strip, 0, 255, revk_blinker ());
      led_strip_refresh (strip);
      usleep (100000);
   }
   vTaskDelete (NULL);
}

void
app_main ()
{
   revk_boot (&app_callback);
   revk_start ();
   if (led.set)
      revk_task ("led", &led_task, NULL, 4);
   if (scl.set && sda.set)
      revk_task ("i2c", &i2c_task, NULL, 4);
}

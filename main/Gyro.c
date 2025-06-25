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
app_main ()
{
   revk_boot (&app_callback);
   revk_start ();
   if (scl.set && sda.set)
      revk_task ("i2c", &i2c_task, NULL, 4);
}

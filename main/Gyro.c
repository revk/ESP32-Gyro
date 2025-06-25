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
const uint8_t i2cport = 0;

struct
{
   uint8_t die:1;
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
i2c_write (uint8_t reg, uint8_t val)
{
   i2c_cmd_handle_t t = i2c_cmd_link_create ();
   i2c_master_start (t);
   i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte (t, reg, true);
   i2c_master_write_byte (t, val, true);
   i2c_master_stop (t);
   i2c_master_cmd_begin (i2cport, t, 10 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete (t);
}

uint8_t
i2c_read (uint8_t reg)
{
   uint8_t v = 0;
   i2c_cmd_handle_t t = i2c_cmd_link_create ();
   i2c_master_start (t);
   i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte (t, reg, true);
   i2c_master_start (t);
   i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_READ, true);
   i2c_master_read_byte (t, &v, I2C_MASTER_LAST_NACK);
   i2c_master_stop (t);
   if (i2c_master_cmd_begin (i2cport, t, 10 / portTICK_PERIOD_MS))
      v = 0xFF;
   i2c_cmd_link_delete (t);
   return v;
}

void
i2c_task (void *p)
{
   i2c_driver_install (i2cport, I2C_MODE_MASTER, 0, 0, 0);
   i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda.num,
      .scl_io_num = scl.num,
      .sda_pullup_en = true,
      .scl_pullup_en = true,
      .master.clk_speed = 100000,
   };
   i2c_param_config (i2cport, &config);
   i2c_set_timeout (i2cport, 31);
   if (i2c_read (0x6B) != 0x40)
   {
      ESP_LOGE (TAG, "I2C failed");
      vTaskDelete (NULL);
      return;
   }
   i2c_write (0x1B, 0x18);      // ±2000°/s
   i2c_write (0x1C, 0x18);      // ±16g
   i2c_write (0x6B, 0x20);      // Run
   i2c_write (0x6C, 0x20);      // 5Hz
   while (1)
   {
      uint8_t buf[14];
      i2c_cmd_handle_t t = i2c_cmd_link_create ();
      i2c_master_start (t);
      i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_WRITE, true);
      i2c_master_write_byte (t, 0x3B, true);
      i2c_master_start (t);
      i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_READ, true);
      i2c_master_read (t, buf, sizeof (buf), I2C_MASTER_LAST_NACK);
      i2c_master_stop (t);
      if (!i2c_master_cmd_begin (i2cport, t, 10 / portTICK_PERIOD_MS))
      {
         ESP_LOG_BUFFER_HEX_LEVEL (TAG, buf, sizeof (buf), ESP_LOG_ERROR);
      }
      i2c_cmd_link_delete (t);
      usleep (200000);
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
   led_strip_new_rmt_device (&strip_config, &rmt_config, &strip);
   while (!b.die)
   {
      revk_led (strip, 0, 255, revk_blinker ());
      led_strip_refresh (strip);
      usleep (100000);
   }
   vTaskDelete (NULL);
}

void
btn_task (void *p)
{
   revk_gpio_input (btn);
   while (!b.die)
   {
      // TODO
      usleep (100000);
   }
}

void
chg_task (void *p)
{
   revk_gpio_input (chg);
   revk_gpio_input (vbus);
   while (!b.die)
   {
      // TODO
      usleep (100000);
   }
}

void
app_main ()
{
   ESP_LOGE (TAG, "Started");
   revk_boot (&app_callback);
   revk_start ();
   if (led.set)
      revk_task ("led", &led_task, NULL, 4);
   if (btn.set)
      revk_task ("btn", &btn_task, NULL, 4);
   if (chg.set)
      revk_task ("chg", &chg_task, NULL, 4);
   if (scl.set && sda.set)
      revk_task ("i2c", &i2c_task, NULL, 4);
}

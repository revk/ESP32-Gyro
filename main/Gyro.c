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

SemaphoreHandle_t mutex = NULL;
typedef struct data_s
{
   int16_t ax,
     ay,
     az;
   int16_t gx,
     gy,
     gz;
} data_t;
data_t data = { 0 };

#define	DATAG	(2048)          // ±16g
#define DATARPM	(16*6)          // ±2000°/s

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

esp_err_t
i2c_write (uint8_t reg, uint8_t val)
{
   i2c_cmd_handle_t t = i2c_cmd_link_create ();
   i2c_master_start (t);
   i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte (t, reg, true);
   i2c_master_write_byte (t, val, true);
   i2c_master_stop (t);
   esp_err_t e = i2c_master_cmd_begin (i2cport, t, 100 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete (t);
   if (e)
      ESP_LOGE (TAG, "I2C write fail %02X %02X", reg, val);
   usleep (200000);
   return e;
}

esp_err_t
i2c_read (uint8_t reg, uint8_t len, uint8_t * data)
{
   i2c_cmd_handle_t t = i2c_cmd_link_create ();
   i2c_master_start (t);
   i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte (t, reg, true);
   i2c_master_start (t);
   i2c_master_write_byte (t, (addr << 1) | I2C_MASTER_READ, true);
   i2c_master_read (t, data, len, I2C_MASTER_LAST_NACK);
   i2c_master_stop (t);
   esp_err_t e = i2c_master_cmd_begin (i2cport, t, 100 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete (t);
   if (e)
      ESP_LOGE (TAG, "I2C read fail %02X", reg);
   return e;
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
   uint8_t id = 0;
   if (i2c_read (0x75, 1, &id) || id != 0x68)
   {
      usleep (200000);
      if (i2c_read (0x75, 1, &id) || id != 0x68)
      {
         ESP_LOGE (TAG, "I2C failed %02X", id);
         vTaskDelete (NULL);
         return;
      }
   }
   i2c_write (0x6B, 0x80);      // Reset
   i2c_write (0x6B, 0x08 + 5);  // No temp, clock 1
   i2c_write (0x19, 100 - 1);   // Sample rate divide
   i2c_write (0x1A, 0x01);      // Filter
   i2c_write (0x1B, 0x18);      // ±2000°/s
   i2c_write (0x1C, 0x18);      // ±16g
   i2c_write (0x23, 0x78);      // Acc and gyro FIFO
   i2c_write (0x6A, 0x44);      // FIFO enable / reset

   while (1)
   {
      uint8_t buf[12];
      if (!i2c_read (0x72, 2, buf))
      {
         uint16_t fifo = ((buf[0] << 8) | buf[1]);
         //if (fifo) ESP_LOGE (TAG, "FIFO %d", fifo);
         while (fifo >= 12)
         {
            if (!i2c_read (0x74, sizeof (buf), buf))
            {
               data_t d;
               d.ax = (buf[0] << 8) | buf[1];
               d.ay = (buf[2] << 8) | buf[3];
               d.az = (buf[4] << 8) | buf[5];
               d.gx = (buf[6] << 8) | buf[7];
               d.gy = (buf[8] << 8) | buf[9];
               d.gz = (buf[10] << 8) | buf[11];
               xSemaphoreTake (mutex, portMAX_DELAY);
               data = d;
               xSemaphoreGive (mutex);
               //ESP_LOGE (TAG, "%6d %6d %6d %6d %6d %6d", d.ax, d.ay, d.az, d.gx, d.gy, d.gz);
            }
            fifo -= sizeof (buf);
         }
      }
      usleep (100000);
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

      // TODO show charging/battery level - maybe when charging or button pushed

      // Show level
      data_t d;
      xSemaphoreTake (mutex, portMAX_DELAY);
      d = data;
      xSemaphoreGive (mutex);
      double g = sqrt ((double) d.ax * (double) d.ax + (double) d.ay * (double) d.ay + (double) d.az * (double) d.az) / DATAG;
      if (g == 0)
         for (int l = 0; l < LEDS; l++)
            revk_led (strip, l + 1, 255, 0x4400);
      else
      {
#define	CIRCLE	(LEDS*255)
         double a = atan2 ((double) d.ax / DATAG, (double) d.ay / DATAG) * (CIRCLE / 2) / M_PI;
         double f = (g - (double) d.az / DATAG) / g;
         int a1 = a - (CIRCLE / 4) * f + (CIRCLE / 2) + (CIRCLE / LEDS / 2),
            a2 = a + (CIRCLE / 4) * f + (CIRCLE / 2) + (CIRCLE / LEDS / 2);
         //ESP_LOGE (TAG, "G=%f A=%f F=%f %d-%d", g, a, f, a1, a2);
         uint8_t level[LEDS] = { 0 };
         while (a1 < a2)
         {
            int l = (a1 * LEDS / CIRCLE);
            int n = (l + 1) * CIRCLE / LEDS - a1;
            if (a2 < a1 + n)
               n = a2 - a1;
            if (!n)
               n = 1;
            level[l % LEDS] += n;
            a1 += n;
         }
         //ESP_LOG_BUFFER_HEX_LEVEL (TAG, level, sizeof (level), ESP_LOG_ERROR);
         for (int l = 0; l < LEDS; l++)
            revk_led (strip, l + 1, 255, 0xFF0000 * level[l] * LEDS / CIRCLE + 0x44);
      }
      revk_led (strip, 0, 255, revk_blinker ());
      led_strip_refresh (strip);
      usleep (50000);
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
   revk_gpio_output (adce, 0);
   if (adc.set)
   {
      // TODO
   }
   while (!b.die)
   {
      // TODO
      usleep (100000);
   }
}

void
report_task (void *p)
{
   // TODO
   while (!b.die)
   {
      // TODO
      sleep (1);
   }
}

void
app_main ()
{
   //ESP_LOGE (TAG, "Started");
   revk_boot (&app_callback);
   revk_start ();
   mutex = xSemaphoreCreateMutex ();
   xSemaphoreGive (mutex);
   if (led.set)
      revk_task ("led", &led_task, NULL, 4);
   if (btn.set)
      revk_task ("btn", &btn_task, NULL, 4);
   if (chg.set)
      revk_task ("chg", &chg_task, NULL, 4);
   if (scl.set && sda.set && addr)
      revk_task ("i2c", &i2c_task, NULL, 4);
   if (*report)
      revk_task ("report", &report_task, NULL, 4);
   while (!b.die)
   {
      sleep (1);
   }
   // TODO shutdown / sleep / etc
}

// Gyro app

static const char __attribute__((unused)) TAG[] = "Gyro";

#include "revk.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/rtc_io.h>
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
   uint8_t vbus:1;
   uint8_t charging:1;
   uint8_t batfull:1;
} b = { 0 };

uint8_t showbat = 30;
float voltage = NAN;
#define ADC_SCALE       3
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define	BAT_EMPTY	3500    // mV
#define	BAT_FULL	4100    // mV

const char *
app_callback (int client, const char *prefix, const char *target, const char *suffix, jo_t j)
{
   return NULL;
}

void
revk_state_extra (jo_t j)
{
   data_t d;
   xSemaphoreTake (mutex, portMAX_DELAY);
   d = data;
   xSemaphoreGive (mutex);
   double r = sqrt ((double) d.gx * (double) d.gx + (double) d.gy * (double) d.gy + (double) d.gz * (double) d.gz) / DATARPM;
      double g = sqrt ((double) d.ax * (double) d.ax + (double) d.ay * (double) d.ay + (double) d.az * (double) d.az) / DATAG;
   jo_string (j, "id", hostname);
   jo_litf (j, "rpm", "%.2lf", r);
   jo_litf (j, "G", "%.3lf", g);
   if (!isnan (voltage))
      jo_litf (j, "V", "%.3f", voltage / 1000);
   if (reportdebug)
   {
      jo_object (j, "acc");
      jo_int (j, "x", data.ax);
      jo_int (j, "y", data.ay);
      jo_int (j, "z", data.az);
      jo_close (j);
      jo_object (j, "gyro");
      jo_int (j, "x", data.gx);
      jo_int (j, "y", data.gy);
      jo_int (j, "z", data.gz);
      jo_close (j);
   }
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
   while (i2c_read (0x75, 1, &id) || id != 0x68)
      sleep (1);
   i2c_write (0x6B, 0x80);      // Reset
   i2c_write (0x6B, 0x08 + 5);  // No temp, clock 1
   i2c_write (0x19, 100 - 1);   // Sample rate divide
   i2c_write (0x1A, 0x01);      // Filter
   i2c_write (0x1B, 0x18);      // ±2000°/s
   i2c_write (0x1C, 0x18);      // ±16g
   i2c_write (0x23, 0x78);      // Acc and gyro FIFO
   i2c_write (0x6A, 0x44);      // FIFO enable / reset

   while (!b.die)
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
   xSemaphoreTake (mutex, portMAX_DELAY);
   memset (&data, 0, sizeof (data));
   xSemaphoreGive (mutex);
   vTaskDelete (NULL);
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
      if (showbat)
      {                         // Show battery status
         showbat--;
         if (!isnan (voltage) && !b.batfull)
         {                      // Show battery level
            uint8_t bat = 0;
            if (voltage > BAT_FULL)
               bat = 100;
            else if (voltage > BAT_EMPTY)
               bat = (voltage - BAT_EMPTY) * 100 / (BAT_FULL - BAT_EMPTY);
            //ESP_LOGE (TAG, "Bat=%d", bat);
            bat = ((uint32_t) bat * LEDS + 50) / 100;
            for (int l = 0; l < LEDS; l++)
               revk_led (strip, l + 1, 255, l == bat && b.vbus ? showbat & 1 ? 0x008800 : 0x004400 : l < bat ? 0x004400 : 0x000044);
            continue;
         }
         if (b.vbus && (showbat & 4))
            for (int l = 0; l < LEDS; l++)
               revk_led (strip, l + 1, 255, 0x000044);  // flashing as charger
         else if (b.batfull)
            for (int l = 0; l < LEDS; l++)
               revk_led (strip, l + 1, 255, 0x004400);  // full
         else
            for (int l = 0; l < LEDS; l++)
               revk_led (strip, l + 1, 255, 0x440000);  // no bat or other problem
         continue;
      }
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
         double f = (double) d.az / DATAG / g;
         int da = ((CIRCLE / 2) - asin (f) / M_PI * CIRCLE) / 2;
         int a1 = a - da + CIRCLE + (CIRCLE / 2) + (CIRCLE / LEDS / 2),
            a2 = a + da + CIRCLE + (CIRCLE / 2) + (CIRCLE / LEDS / 2);
         //ESP_LOGE (TAG, "G=%f A=%f F=%f da=%d %d-%d af=%f", g, a, f, da, a1, a2, asin (f));
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
         uint8_t b = (g > 3.99 ? 0xFF : g * 0x40);
         uint8_t r = (g > .99 ? 0xFF : g * 0xFF);
         for (int l = 0; l < LEDS; l++)
            revk_led (strip, l + 1, 255, ((r * level[l] * LEDS / CIRCLE) << 16) + b);
      }
   }
   for (int l = 0; l <= LEDS; l++)
      revk_led (strip, l, 255, 0);
   led_strip_refresh (strip);
   vTaskDelete (NULL);
}

void
btn_task (void *p)
{
   revk_gpio_input (btn);
   uint8_t t = 0;
   while (!b.die)
   {
      uint8_t press = revk_gpio_get (btn);
      if (press)
      {
         showbat = 30;
         t++;
         if (t >= 30)
            b.die = 1;          // Power down
      } else
         t = 0;
      usleep (100000);
   }
   vTaskDelete (NULL);
}

void
chg_task (void *p)
{
   revk_gpio_input (chg);
   revk_gpio_input (vbus);
   revk_gpio_output (adce, 0);
   adc_oneshot_unit_handle_t adc_handle;
   adc_channel_t adc_channel = 0;
   adc_cali_handle_t adc_cali_handle = NULL;
   if (adc.set)
   {
      adc_unit_t adc_unit = 0;
      adc_oneshot_io_to_channel (adc.num, &adc_unit, &adc_channel);
      ESP_LOGE (TAG, "ADC %d unit %d channel %d", adc.num, adc_unit, adc_channel);
      adc_oneshot_unit_init_cfg_t init_config1 = {
         .unit_id = adc_unit,
         .ulp_mode = ADC_ULP_MODE_DISABLE,
      };
      adc_oneshot_new_unit (&init_config1, &adc_handle);
      adc_oneshot_chan_cfg_t config = {
         .bitwidth = ADC_BITWIDTH_DEFAULT,
         .atten = ADC_ATTEN,
      };
      adc_oneshot_config_channel (adc_handle, adc_channel, &config);
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      adc_cali_curve_fitting_config_t cali_config = {
         .unit_id = adc_unit,
         .chan = adc_channel,
         .atten = ADC_ATTEN,
         .bitwidth = ADC_BITWIDTH_DEFAULT,
      };
      adc_cali_create_scheme_curve_fitting (&cali_config, &adc_cali_handle);
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      adc_cali_line_fitting_config_t cali_config = {
         .unit_id = adc_unit,
         .atten = ADC_ATTEN,
         .bitwidth = ADC_BITWIDTH_DEFAULT,
      };
      adc_cali_create_scheme_line_fitting (&cali_config, &adc_cali_handle);
#endif
   }
   uint8_t charge = 0;
   uint8_t tick = 0;
   while (!b.die)
   {
      charge = (charge << 1) | revk_gpio_get (chg);
      uint8_t v = revk_gpio_get (vbus);
      if (v != b.vbus)
      {
         showbat = 30;
         b.vbus = v;
      }
      b.charging = ((b.vbus && charge == 255) ? 1 : 0);
      b.batfull = ((b.vbus && !charge) ? 1 : 0);
      if (b.vbus && charge && charge != 255)
         voltage = NAN;         // No bat
      else if (adc.set && (isnan (voltage) || !tick))
      {
         tick = 10;
         int raw = 0,
            volt = 0;
         revk_gpio_set (adce, 1);
         adc_oneshot_read (adc_handle, adc_channel, &raw);
         revk_gpio_set (adce, 0);
         adc_cali_raw_to_voltage (adc_cali_handle, raw, &volt);
         voltage = volt * ADC_SCALE;
         //ESP_LOGE (TAG, "V=%lf", voltage);
      }
      tick--;
      usleep (100000);
   }
   adc_oneshot_del_unit (adc_handle);
   vTaskDelete (NULL);
}

void
report_task (void *p)
{
#define	IPS	(sizeof(reportip)/sizeof(*reportip))
   int sock[IPS];
   for (int i = 0; i < IPS; i++)
   {
      sock[i] = -1;
      if (*reportip[i])
      {
         const struct addrinfo hints = {
            .ai_family = AF_UNSPEC,
            .ai_socktype = SOCK_DGRAM,
         };
         struct addrinfo *a = 0,
            *t;
         char *mem = strdup (reportip[i]);
         char *ip = mem;
         char *port = NULL;
         if (*ip == '[')
         {
            ip++;
            port = strchr (ip, ']');
            if (port)
            {
               *port++ = 0;
               if (*port == ':')
                  port++;
               else
                  port = NULL;
            }
         } else if ((port = strrchr (ip, ':')))
            *port++ = 0;
         if (!port)
            port = "5555";
         if (!getaddrinfo (ip, port, &hints, &a) && a)
         {
            for (t = a; t; t = t->ai_next)
            {
               sock[i] = socket (t->ai_family, t->ai_socktype, t->ai_protocol);
               if (sock[i] < 0)
                  continue;
               if (!connect (sock[i], t->ai_addr, t->ai_addrlen))
                  break;
               close (sock[i]);
               sock[i] = -1;
            }
            freeaddrinfo (a);
         }
         if (sock[i] < 0)
            ESP_LOGE (TAG, "Failed to resolve %s:%s", ip, port);
         free (mem);
      }
   }
   while (!b.die)
   {
      jo_t j = jo_object_alloc ();
      revk_state_extra (j);
      for (int i = 0; i < IPS; i++)
         if (sock[i] >= 0)
         {
            const char *p = jo_debug (j);
            if (send (sock[i], p, strlen (p), 0) < 0)
               ESP_LOGE (TAG, "Send to %s fail", reportip[i]);
         }
      if (reportmqtt)
         revk_info ("data", &j);
      else
         jo_free (&j);
      sleep (reportrate);
   }
   for (int i = 0; i < IPS; i++)
      if (sock[i] >= 0)
         close (sock[i]);
   vTaskDelete (NULL);
#undef	IPS
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
   if (reportrate)
      revk_task ("report", &report_task, NULL, 4);
   while (!b.die)
      sleep (1);
   revk_pre_shutdown ();
   // Alarm
   if (rtc_gpio_is_valid_gpio (btn.num))
   {                            // Deep sleep
      ESP_LOGE (TAG, "Deep sleep on btn %d %s", btn.num, btn.invert ? "low" : "high");
      rtc_gpio_set_direction_in_sleep (btn.num, RTC_GPIO_MODE_INPUT_ONLY);
      REVK_ERR_CHECK (esp_sleep_enable_ext0_wakeup (btn.num, 1 - btn.invert));
   } else
   {                            // Light sleep
      ESP_LOGE (TAG, "Light sleep on btn %d %s", btn.num, btn.invert ? "low" : "high");
      gpio_wakeup_enable (btn.num, btn.invert ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
      esp_sleep_enable_gpio_wakeup ();
   }
   // Shutdown
   sleep (1);                   // Allow tasks to end
#if     CONFIG_REVK_GPIO_POWER >= 0
   gpio_set_level (CONFIG_REVK_GPIO_POWER, 0);  // Hard power off (unless USB)
#endif
   // Night night
   if (rtc_gpio_is_valid_gpio (btn.num))
      esp_deep_sleep_start ();
   else
      esp_light_sleep_start ();
   esp_restart ();
}

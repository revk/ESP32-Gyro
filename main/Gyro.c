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
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"

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
   double rpm;
   double g;
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
#define ADC_SCALE       (3)     // Resistor ratio
#define ADC_ATTEN       ADC_ATTEN_DB_2_5
#define	BAT_EMPTY	3100    // mV
#define	BAT_FULL	4100    // mV

const char *
app_callback (int client, const char *prefix, const char *target, const char *suffix, jo_t j)
{
   if (client || !prefix || target || strcmp (prefix, topiccommand) || !suffix)
      return NULL;
   if (!strcmp (suffix, "off"))
      b.die = 1;
   return NULL;
}

void
revk_state_extra (jo_t j)
{
   data_t d;
   xSemaphoreTake (mutex, portMAX_DELAY);
   d = data;
   xSemaphoreGive (mutex);
   jo_string (j, "id", hostname);
   jo_litf (j, "rpm", "%.2lf", d.rpm);
   jo_litf (j, "G", "%.3lf", d.g);
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


/// BLE
#define MAX_ADV 31

static void
ble_on_sync (void)
{
}

static void
ble_on_reset (int reason)
{
}

static uint8_t
add_name (uint8_t * data, uint8_t p, uint8_t len, const char *name)
{
   if (!name || !*name || p + len + 2 >= MAX_ADV)
      return p;
   // Name
   int8_t l = strlen (name);
   int8_t space = MAX_ADV - p - len - 2;
   if (l > space)
   {                            // Shortened
      l = space;
      data[p++] = (l + 1);
      data[p++] = (8);
   } else
   {                            // Full
      data[p++] = (l + 1);
      data[p++] = (9);
   }
   while (l--)
      data[p++] = (*name++);
   return p;
}


static void
ble_adv (const char *name, uint8_t * data, uint8_t len)
{
   //ESP_LOG_BUFFER_HEX_LEVEL ("ADV", data, len, ESP_LOG_ERROR);
   ble_gap_adv_set_data (data, len);
   uint8_t rsp[MAX_ADV],
     p = 0;
   p = add_name (rsp, p, 0, name);
   //ESP_LOG_BUFFER_HEX_LEVEL ("RSP", rsp, p, ESP_LOG_ERROR);
   ble_gap_adv_rsp_set_data (rsp, p);
   if (!ble_gap_adv_active ())
   {
      struct ble_gap_adv_params adv_params = { 0 };
      adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
      adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
      int e = ble_gap_adv_start (BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
      ESP_LOGD ("BLE", "Adv started %d", e);
   }
}

void
ble_send (data_t d)
{                               // Set the advertisements
   uint8_t data[MAX_ADV],
     p = 0;
   data[p++] = 2;               // Len
   data[p++] = 1;               // Flags
   data[p++] = 6;               // Discoverable / no BR
   data[p++] = 0;               // Len
   data[p++] = 0xFF;            // Manufacturer specific
   data[p++] = 0x9C;            // A&A
   data[p++] = 0x0E;
   data[p++] = 'S';             // Message
   data[p++] = '1';
   data[p++] = d.ax;
   data[p++] = (d.ax >> 8);
   data[p++] = d.ay;
   data[p++] = (d.ay >> 8);
   data[p++] = d.az;
   data[p++] = (d.az >> 8);
   data[p++] = d.gx;
   data[p++] = (d.gx >> 8);
   data[p++] = d.gy;
   data[p++] = (d.gy >> 8);
   data[p++] = d.gz;
   data[p++] = (d.gz >> 8);
   int16_t v = d.rpm * 100;
   data[p++] = v;
   data[p++] = (v >> 8);
   v = d.g * 100;
   data[p++] = v;
   data[p++] = (v >> 8);
   data[3] = p - 4;             // Len
   p = add_name (data, p, 0, hostname);
   ble_adv (hostname, data, p);
}

void
ble_task (void *p)
{
   nimble_port_run ();
   nimble_port_freertos_deinit ();
}

void
ble_start (void)
{
   REVK_ERR_CHECK (esp_wifi_set_ps (WIFI_PS_MIN_MODEM));        /* default mode, but library may have overridden, needed for BLE at same time as wifi */
   nimble_port_init ();
   ble_hs_cfg.sync_cb = ble_on_sync;
   ble_hs_cfg.reset_cb = ble_on_reset;
   ble_hs_cfg.sm_sc = 1;
   ble_hs_cfg.sm_mitm = 0;
   ble_hs_cfg.sm_bonding = 1;
   ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
   nimble_port_freertos_init (ble_task);
}

void
i2c_task (void *p)
{
#define SPS	100
   // Not floats so as to avoid accumating errors
   unsigned long ra[SPS] = { 0 }, rt = 0;       // RPM moving average
   unsigned long ga[SPS] = { 0 }, gt = 0;       // G moving average
   uint8_t pa = 0;
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
   while (!b.die && (i2c_read (0x75, 1, &id) || id != 0x68))
      sleep (1);
   i2c_write (0x6B, 0x80);      // Reset
   i2c_write (0x6B, 0x08 + 5);  // No temp, clock 1
   i2c_write (0x19, 1000 / SPS - 1);    // Sample rate divide
   if (SPS >= 200)
      i2c_write (0x1A, 0x01);   // Filter 184&188Hz
   else if (SPS >= 100)
      i2c_write (0x1A, 0x02);   // Filter 94&98Hz
   else if (SPS >= 50)
      i2c_write (0x1A, 0x04);   // Filter 44&42Hz
   else if (SPS >= 20)
      i2c_write (0x1A, 0x04);   // Filter 21&20Hz
   else if (SPS >= 10)
      i2c_write (0x1A, 0x05);   // Filter, 10Hz
   else
      i2c_write (0x1A, 0x06);   // Filter, 5Hz
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
               data_t d = { 0 };
               d.ax = (buf[0] << 8) | buf[1];
               d.ay = (buf[2] << 8) | buf[3];
               d.az = (buf[4] << 8) | buf[5];
               d.gx = (buf[6] << 8) | buf[7];
               d.gy = (buf[8] << 8) | buf[9];
               d.gz = (buf[10] << 8) | buf[11];
               // G moving average
               gt -= ga[pa];
               gt += (ga[pa] =
                      1000 * sqrt ((double) d.ax * (double) d.ax + (double) d.ay * (double) d.ay +
                                   (double) d.az * (double) d.az) / DATAG);
               d.g = (double) gt / SPS / 1000;
               // RPM moving average
               rt -= ra[pa];
               rt += (ra[pa] =
                      1000 * sqrt ((double) d.gx * (double) d.gx + (double) d.gy * (double) d.gy +
                                   (double) d.gz * (double) d.gz) / DATARPM);
               d.rpm = (double) rt / SPS / 1000;
               if (++pa == SPS)
                  pa = 0;
               xSemaphoreTake (mutex, portMAX_DELAY);
               data = d;
               xSemaphoreGive (mutex);
               //ESP_LOGE (TAG, "%6d %6d %6d %6d %6d %6d %f %f", d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.rpm, d.g);
            }
            fifo -= sizeof (buf);
         }
      }
      usleep (10000);
   }
   xSemaphoreTake (mutex, portMAX_DELAY);
   memset (&data, 0, sizeof (data));
   xSemaphoreGive (mutex);
   i2c_write (0x6B, 0xC0);      // Reset/sleep
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
      double g = sqrt ((double) d.ax * (double) d.ax + (double) d.ay * (double) d.ay + (double) d.az * (double) d.az) / DATAG;  // Instant, not moving average
      if (g == 0)
         for (int l = 0; l < LEDS; l++)
            revk_led (strip, l + 1, 255, l & 1 ? 0x440000 : 0x4400);
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
   gpio_hold_dis (btn.num);
   revk_gpio_input (btn);
   if (revk_gpio_get (btn))
   {
      ESP_LOGE (TAG, "Wait button");
      while (revk_gpio_get (btn))
         usleep (100000);       // wait release
      ESP_LOGE (TAG, "Button ready");
   }
   uint8_t t = 0;
   while (!b.die)
   {
      uint8_t press = revk_gpio_get (btn);
      if (press)
      {
         //if (!t) ESP_LOGE (TAG, "Press");
         showbat = 30;
         t++;
         if (t >= 30)
            b.die = 1;          // Power down
      } else
      {
         //if (t) ESP_LOGE (TAG, "Release");
         t = 0;
      }
      usleep (100000);
   }
   vTaskDelete (NULL);
}

void
chg_task (void *p)
{
   revk_gpio_input (chg);
   revk_gpio_input (vbus);
   revk_gpio_output (adce, 1);  // Waste of time FFS
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
#else
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      adc_cali_line_fitting_config_t cali_config = {
         .unit_id = adc_unit,
         .atten = ADC_ATTEN,
         .bitwidth = ADC_BITWIDTH_DEFAULT,
      };
      adc_cali_create_scheme_line_fitting (&cali_config, &adc_cali_handle);
#endif
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
      else if (adc.set && isnan (voltage))
         tick = 0;
      if (adc.set && !tick)
      {
         tick = 10;
         int volt = 0;
         adc_oneshot_get_calibrated_result (adc_handle, adc_cali_handle, adc_channel, &volt);
         voltage = volt * ADC_SCALE;
         //ESP_LOGE (TAG, "V=%lf", voltage);
      }
      usleep (100000);
      tick--;
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
      if (!reportrate)
         usleep (100000);
      else
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
   ble_start ();
   if (led.set)
      revk_task ("led", &led_task, NULL, 4);
   if (btn.set)
      revk_task ("btn", &btn_task, NULL, 4);
   if (chg.set)
      revk_task ("chg", &chg_task, NULL, 4);
   if (scl.set && sda.set && addr)
      revk_task ("i2c", &i2c_task, NULL, 10);
   if (reportrate)
      revk_task ("report", &report_task, NULL, 4);
   while (!b.die)
   {
      data_t d;
      xSemaphoreTake (mutex, portMAX_DELAY);
      d = data;
      xSemaphoreGive (mutex);
      ble_send (d);
      sleep (1);
   }
   revk_pre_shutdown ();
   if (btn.set)
   {
      ESP_LOGE (TAG, "Wait release");
      while (revk_gpio_get (btn))
         usleep (100000);       // release
   }
   if (vbus.set && revk_gpio_get (vbus))
   {
      ESP_LOGE (TAG, "Wait USB");
      while (revk_gpio_get (vbus))
      {
         if (revk_gpio_get (btn))
            esp_restart ();
         usleep (10000);
      }
   }
   ESP_LOGE (TAG, "Shutdown");
   // Shutdown
   sleep (1);                   // Allow tasks to end
#if     CONFIG_REVK_GPIO_POWER >= 0
   if (btn.set && btn.pulldown && !btn.invert)
   {
      ESP_LOGE (TAG, "Power off GPIO %d", btn.num);
      gpio_set_level (btn.num, 0);
      gpio_set_direction (btn.num, GPIO_MODE_OUTPUT_OD);
      gpio_hold_en (btn.num);
   }
   ESP_LOGE (TAG, "Power off GPIO %d", CONFIG_REVK_GPIO_POWER);
   gpio_set_direction (CONFIG_REVK_GPIO_POWER, GPIO_MODE_OUTPUT_OD);
   gpio_set_level (CONFIG_REVK_GPIO_POWER, 0);
   gpio_hold_en (CONFIG_REVK_GPIO_POWER);
   sleep (1);
   // Should be off now - if not, undo so we can deep sleep
   gpio_hold_dis (CONFIG_REVK_GPIO_POWER);
   if (btn.set && btn.pulldown && !btn.invert)
   {
      gpio_hold_dis (btn.num);
      revk_gpio_input (btn);
   }
#endif
   ESP_LOGE (TAG, "Sleep");
   // Alarm btn from sleep
   if (btn.set)
   {
      if (rtc_gpio_is_valid_gpio (btn.num))
      {                         // Deep sleep
         ESP_LOGE (TAG, "Deep sleep on btn %d %s", btn.num, btn.invert ? "low" : "high");
         rtc_gpio_set_direction_in_sleep (btn.num, RTC_GPIO_MODE_INPUT_ONLY);
         if (btn.pulldown)
            rtc_gpio_pulldown_en (btn.num);
         REVK_ERR_CHECK (esp_sleep_enable_ext0_wakeup (btn.num, 1 - btn.invert));
      } else
      {                         // Light sleep
         ESP_LOGE (TAG, "Light sleep on btn %d %s", btn.num, btn.invert ? "low" : "high");
         gpio_wakeup_enable (btn.num, btn.invert ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
         esp_sleep_enable_gpio_wakeup ();
      }
   }
   // Night night
   if (btn.set && rtc_gpio_is_valid_gpio (btn.num))
      esp_deep_sleep_start ();
   else
      esp_light_sleep_start ();
   esp_restart ();
}

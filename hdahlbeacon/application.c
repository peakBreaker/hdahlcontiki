/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC13xx/CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC13xx/CC26xx EM
 *   - CC2650 and CC1350 SensorTag
 *   - CC1310, CC1350, CC2650 LaunchPads
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_TRIGGER_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_TRIGGER_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "dev/button-hal.h"
#include "random.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 50)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_TRIGGER_1     BOARD_BUTTON_HAL_INDEX_KEY_LEFT
#define CC26XX_DEMO_TRIGGER_2     BOARD_BUTTON_HAL_INDEX_KEY_RIGHT

#if BOARD_SENSORTAG
#define CC26XX_DEMO_TRIGGER_3     BOARD_BUTTON_HAL_INDEX_REED_RELAY
#endif

/* BLE Intervals: Send a burst of advertisements every BLE_ADV_INTERVAL secs */
#define BLE_ADV_DUTY_CYCLE    (CLOCK_SECOND / 10)
#define BLE_ADV_MESSAGES            10

/* BLE advertisement Macros */
#define HD_BLE_ADV_PAYLOAD_LEN 64
#define ADV_SENSOR_DATA_BASE "{'temp' : %i}"
#define ADV_NAME_BASE "Anders_%i"
#define BLE_ADV_TYPE_DEVINFO      0x01
#define BLE_ADV_TYPE_NAME         0x09
#define BLE_ADV_TYPE_MANUFACTURER 0xFF
#define BLE_ADV_NAME_BUF_LEN        BLE_ADV_MAX_SIZE
#define BLE_ADV_PAYLOAD_BUF_LEN     64
#define BLE_UUID_SIZE               16

/*---------------------------------------------------------------------------*/
static struct etimer et;
static struct etimer adv_et;
static uint8_t payload[HD_BLE_ADV_PAYLOAD_LEN];
static int p = 0;
static int i;

typedef struct acc_vals {
    int x_val;
    int y_val;
    int z_val;
} acc_vals;

/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/
#if BOARD_SENSORTAG
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 20)
#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)

static struct ctimer bmp_timer, opt_timer, hdc_timer, tmp_timer, mpu_timer;
/*---------------------------------------------------------------------------*/
static void init_bmp_reading(void *not_used);
static void init_opt_reading(void *not_used);
static void init_hdc_reading(void *not_used);
static void init_tmp_reading(void *not_used);
static void init_mpu_reading(void *not_used);
/*---------------------------------------------------------------------------*/
static void
print_mpu_reading(int reading)
{
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%d.%02d", reading / 100, reading % 100);
}
/*---------------------------------------------------------------------------*/
static void
get_bmp_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("BAR: Pressure=%d.%02d hPa\n", value / 100, value % 100);
  } else {
    printf("BAR: Pressure Read Error\n");
  }

  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("BAR: Temp=%d.%02d C\n", value / 100, value % 100);
  } else {
    printf("BAR: Temperature Read Error\n");
  }

  SENSORS_DEACTIVATE(bmp_280_sensor);

  ctimer_set(&bmp_timer, next, init_bmp_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_tmp_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_ALL);

  if(value == CC26XX_SENSOR_READING_ERROR) {
    printf("TMP: Ambient Read Error\n");
    return;
  }

  value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_AMBIENT);
  printf("TMP: Ambient=%d.%03d C\n", value / 1000, value % 1000);

  value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_OBJECT);
  printf("TMP: Object=%d.%03d C\n", value / 1000, value % 1000);

  SENSORS_DEACTIVATE(tmp_007_sensor);

  ctimer_set(&tmp_timer, next, init_tmp_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_hdc_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("HDC: Temp=%d.%02d C\n", value / 100, value % 100);
  } else {
    printf("HDC: Temp Read Error\n");
  }

  value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_HUMIDITY);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("HDC: Humidity=%d.%02d %%RH\n", value / 100, value % 100);
  } else {
    printf("HDC: Humidity Read Error\n");
  }

  ctimer_set(&hdc_timer, next, init_hdc_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
  } else {
    printf("OPT: Light Read Error\n");
  }

  /* The OPT will turn itself off, so we don't need to call its DEACTIVATE */
  ctimer_set(&opt_timer, next, init_opt_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static acc_vals
get_mpu_reading()
{
  acc_vals local_accels = {}; // The return object
  int value;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  /* printf("MPU Gyro: X="); */
  /* value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X); */
  /* print_mpu_reading(value); */
  /* printf(" deg/sec\n"); */

  /* printf("MPU Gyro: Y="); */
  /* value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y); */
  /* print_mpu_reading(value); */
  /* printf(" deg/sec\n"); */

  /* printf("MPU Gyro: Z="); */
  /* value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z); */
  /* print_mpu_reading(value); */
  /* printf(" deg/sec\n"); */

  printf("MPU Acc: X=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  local_accels.x_val = value;
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Acc: Y=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  local_accels.y_val = value;
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Acc: Z=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  local_accels.z_val = value;
  print_mpu_reading(value);
  printf(" G\n");

  SENSORS_DEACTIVATE(mpu_9250_sensor);

  ctimer_set(&mpu_timer, next, init_mpu_reading, NULL);

  // Finally deactivate the sensor, and return the struct
  return local_accels;
}
/*---------------------------------------------------------------------------*/
static void
init_bmp_reading(void *not_used)
{
  SENSORS_ACTIVATE(bmp_280_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_opt_reading(void *not_used)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_hdc_reading(void *not_used)
{
  SENSORS_ACTIVATE(hdc_1000_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_tmp_reading(void *not_used)
{
  SENSORS_ACTIVATE(tmp_007_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *not_used)
{ //   MPU_9250_SENSOR_TYPE_ACC
  mpu_9250_sensor.configure(SENSORS_ACTIVE,MPU_9250_SENSOR_TYPE_ALL);
}
#endif
/*---------------------------------------------------------------------------*/
static void
get_sync_sensor_readings(void)
{
  int value;

  printf("-----------------------------------------\n");

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  printf("Bat: Temp=%d C\n", value);

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  printf("Bat: Volt=%d mV\n", (value * 125) >> 5);

#if BOARD_SMARTRF06EB
  SENSORS_ACTIVATE(als_sensor);

  value = als_sensor.value(0);
  printf("ALS: %d raw\n", value);

  SENSORS_DEACTIVATE(als_sensor);
#endif

  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{
  SENSORS_ACTIVATE(batmon_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_sensor_readings(void)
{
#if BOARD_SENSORTAG
  SENSORS_ACTIVATE(hdc_1000_sensor);
  SENSORS_ACTIVATE(tmp_007_sensor);
  SENSORS_ACTIVATE(opt_3001_sensor);
  SENSORS_ACTIVATE(bmp_280_sensor);

  init_mpu_reading(NULL);
#endif
}

static void
init_sensor_cycle(void)
{
  init_tmp_reading(NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
  // For beacon advertisement
  char adv_name[20];
  static int counter;
  static acc_vals acc_sensor_values;
  /* char adv_sensor_data[20]; */

  PROCESS_BEGIN();

  printf("CC26XX demo\n");

  init_sensors();

  /* Init the BLE advertisement daemon */
  /* rf_ble_beacond_config(0, ADV_NAME_BASE); */
  /* rf_ble_beacond_start(); */

  etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);

  get_sync_sensor_readings();
  /* init_sensor_readings(); */
  init_mpu_reading(NULL);

  counter = 0;
  i = 0;
  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {

      /* get_sync_sensor_readings(); */

      printf("Advertising Name :: AndersSensortag\n");
      /* Parse the data to the payload */

      p = 0; // payload indexer
      // device info
      memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);
      payload[p++] = 0x02;          /* 2 bytes */
      payload[p++] = BLE_ADV_TYPE_DEVINFO;
      payload[p++] = 0x1a;          /* LE general discoverable + BR/EDR */

      // Sensordata

      /* printf("hdc :: %i\n", hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP)); */
      /* printf("hdc :: %i\n", hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP)); */
      /* printf("mpu :: %i \n", mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X)); */
      /* printf("tmp :: %i\n", tmp_007_sensor.value(TMP_007_SENSOR_TYPE_ALL)); */
      /* printf("opt :: %i \n", opt_3001_sensor.value(0)); */
      /* printf("bmp :: %i\n", bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP)); */
      /* acc_sensor_values = get_mpu_reading(); */
      
      acc_sensor_values.x_val = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
      acc_sensor_values.y_val = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
      acc_sensor_values.z_val = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);

      printf("x valus :: %x, y value :: %x, z value :: %x \n",
             acc_sensor_values.x_val,
             acc_sensor_values.y_val,
             acc_sensor_values.z_val);

      char s_vals[200];
      sprintf(s_vals, "%i,%i,%i",
              acc_sensor_values.x_val,
              acc_sensor_values.y_val,
              acc_sensor_values.z_val);
      printf("sending values %s", s_vals);

      payload[p++] = 1 + strlen(s_vals);
      payload[p++] = BLE_ADV_TYPE_MANUFACTURER;
      memcpy(&payload[p], s_vals, strlen(s_vals));
      p += strlen(s_vals);
      /* p+=sizeof(int); */
      /* payload[p] = acc_sensor_values.y_val; */
      /* p+=sizeof(int); */
      /* payload[p] = acc_sensor_values.z_val; */
      /* p+=sizeof(int); */

      // Name
      sprintf(adv_name, ADV_NAME_BASE, counter);
      payload[p++] = 1 + strlen(adv_name);
      payload[p++] = BLE_ADV_TYPE_NAME;
      memcpy(&payload[p], adv_name,
            strlen(adv_name));
      p += strlen(adv_name);

      /*
       * Send BLE_ADV_MESSAGES beacon bursts. Each burst on all three
       * channels, with a BLE_ADV_DUTY_CYCLE interval between bursts
       */
      for(i = 0; i < BLE_ADV_MESSAGES; i++) {
          rf_ble_beacon_single(BLE_ADV_CHANNEL_ALL, payload, p);
          etimer_set(&adv_et, BLE_ADV_DUTY_CYCLE);
          /* Wait unless this is the last burst */
          if(i < BLE_ADV_MESSAGES - 1) {
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adv_et));
          }
      }

      // Toggle the leds
      leds_toggle(CC26XX_DEMO_LEDS_PERIODIC);

      // reset the timer
      etimer_reset(&et);
    } /* else if(ev == sensors_event)  { */
    /*   if(data == &bmp_280_sensor) { */
    /*     get_bmp_reading(); */
    /*   } else if(data == &opt_3001_sensor) { */
    /*     get_light_reading(); */
    /*   } else if(data == &hdc_1000_sensor) { */
    /*     get_hdc_reading(); */
    /*   } else if(data == &tmp_007_sensor) { */
    /*     get_tmp_reading(); */
    /*   } else if(data == &mpu_9250_sensor) { */
    /*     get_mpu_reading(); */
    /*   } */
    /* } */
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */

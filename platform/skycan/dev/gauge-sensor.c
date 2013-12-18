/*
 * Copyright (c) 2013, Skycan.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *         Sensor driver for TI BQ27210 battery gauge.
 * \author
 *         Andres Vahter <andres.vahter@gmail.com>
 */

#include "dev/gauge-sensor.h"
#include "dev/i2c.h"

#define BQ27210_I2C_7BIT_ADDR 0x55

static uint8_t sensor_status = 0;

enum {
  BQ27x10CMD_CTRL      = 0x00,
  BQ27x10CMD_MODE      = 0x01,
  BQ27x10CMD_AR_LSB    = 0x02,
  BQ27x10CMD_AR_MSB    = 0x03,
  BQ27x10CMD_ARTTE_LSB = 0x04,
  BQ27x10CMD_ARTTE_MSB = 0x05,
  BQ27x10CMD_TEMP_LSB  = 0x06,
  BQ27x10CMD_TEMP_MSB  = 0x07,
  BQ27x10CMD_VOLT_LSB  = 0x08,
  BQ27x10CMD_VOLT_MSB  = 0x09,
  BQ27x10CMD_FLAGS     = 0x0A,
  BQ27x10CMD_RSOC      = 0x0B,
  BQ27x10CMD_NAC_LSB   = 0x0C,
  BQ27x10CMD_NAC_MSB   = 0x0D,
  BQ27x10CMD_LMD_LSB   = 0x0E,
  BQ27x10CMD_LMD_MSB   = 0x0F,
  BQ27x10CMD_CAC_LSB   = 0x10,
  BQ27x10CMD_CAC_MSB   = 0x11,
  BQ27x10CMD_FCAC_LSB  = 0x12,
  BQ27x10CMD_FCAC_MSB  = 0x13,
  BQ27x10CMD_AI_LSB    = 0x14,
  BQ27x10CMD_AI_MSB    = 0x15,
  BQ27x10CMD_TTE_LSB   = 0x16,
  BQ27x10CMD_TTE_MSB   = 0x17,
  BQ27x10CMD_TTF_LSB   = 0x18,
  BQ27x10CMD_TTF_MSB   = 0x19,
  BQ27x10CMD_SI_LSB    = 0x1A,
  BQ27x10CMD_SI_MSB    = 0x1B,
  BQ27x10CMD_STTE_LSB  = 0x1C,
  BQ27x10CMD_STTE_MSB  = 0x1D,
  BQ27x10CMD_CEDV_LSB  = 0x20,
  BQ27x10CMD_CEDV_MSB  = 0x21,
  BQ27x10CMD_TTECP_LSB = 0x26,
  BQ27x10CMD_TTECP_MSB = 0x27,
  BQ27x10CMD_CYCL_LSB  = 0x28,
  BQ27x10CMD_CYCL_MSB  = 0x29,
  BQ27x10CMD_CYCT_LSB  = 0x2A,
  BQ27x10CMD_CYCT_MSB  = 0x2B,
  BQ27x10CMD_CSOC      = 0x2C,
  BQ27x10CMD_EE_EN     = 0x6E,
  BQ27x10CMD_ILMD      = 0x76,
  BQ27x10CMD_SEDVF     = 0x77,
  BQ27x10CMD_SEDV1     = 0x78,
  BQ27x10CMD_ISLC_EDVT = 0x79,
  BQ27x10CMD_DMFSD     = 0x7A,
  BQ27x10CMD_TAPER     = 0x7B,
  BQ27x10CMD_PKCFG     = 0x7C,
  BQ27x10CMD_GAF_DEDV  = 0x7D,
  BQ27x10CMD_DCOMP     = 0x7E,
  BQ27x10CMD_TCOMP     = 0x7F
};

/*---------------------------------------------------------------------------*/
uint16_t
read_reg(uint8_t reg) {
  uint8_t rxbuf[2];
  uint8_t tx = reg;

  /* transmit the register to read */
  i2c_transmitinit(BQ27210_I2C_7BIT_ADDR);
  while (i2c_busy());
  i2c_transmit_n(1, &tx);
  while (i2c_busy());

  /* receive the data */
  i2c_receiveinit(BQ27210_I2C_7BIT_ADDR);
  while (i2c_busy());
  i2c_receive_n(2, (uint8_t*)&rxbuf);
  while (i2c_busy());

  return (rxbuf[0] << 8) | (rxbuf[1] & 0xFF);
}

/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  uint16_t templh;

  switch(type) {
    case GAUGE_TEMPERATURE:
      templh = read_reg(BQ27x10CMD_TEMP_LSB);
      /*
       * Datasheet page 14: Temperature = 0.25 * (256 * TEMPH + TEMPL) Kelvins
       * Lets convert it to Celsius throwing away fractional part
       */
      return (256 * (templh & 0xFF) + (templh >> 8)) / 4 - 273;
      break;

    default:
      return 0;
  }
}

/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  switch(type) {
    case SENSORS_HW_INIT:
      i2c_enable();
      break;

    case SENSORS_ACTIVE:
      if(value) {
        /* activate sensor */
        sensor_status = 1;
      } else {
        /* deactivate sensor */
        sensor_status = 0;
      }
      break;
  }

  return 0;
}

/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
    case SENSORS_READY:
      return sensor_status;
  }

  return 0;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(gauge_sensor, GAUGE_SENSOR, value, configure, status);

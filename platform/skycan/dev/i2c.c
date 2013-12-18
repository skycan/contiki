/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         I2C communication device driver for MSP430F5xxx.
 * \author
 *         Enric M. Calvo, Zolertia <ecalvo@zolertia.com>
 *         Marcus Lundén, SICS <mlunden@sics.se>
 *         Andres Vahter <andres.vahter@gmail.com>
 */

#include "i2c.h"
#include "isr_compat.h"

unsigned char tx_byte_counter;
unsigned char rx_byte_counter;
uint8_t tx_byte_total = 0;
uint8_t rx_byte_total = 0;
unsigned char* tx_buf_ptr;
unsigned char* rx_buf_ptr;

void
i2c_receiveinit(uint8_t slave_address)
{
  /* Enable SW reset */
  UCB0CTL1 = UCSWRST;
  /* I2C master, synchronous mode */
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;
  /* Use SMCLK, keep SW reset */
  UCB0CTL1 = UCSSEL_2 | UCSWRST;
  UCB0BR0  = I2C_PRESC_100KHZ_LSB;
  UCB0BR1  = I2C_PRESC_100KHZ_MSB;
  UCB0I2CSA = slave_address;

  /* I2C receiver */
  UCB0CTL1 &= ~UCTR;

  /* Clear SW reset, resume operation */
  UCB0CTL1 &= ~UCSWRST;
  UCB0IE = UCNACKIE;
  /* Enable RX interrupt */
  UCB0IE = UCRXIE;
}

void
i2c_transmitinit(uint8_t slave_address)
{
  /* Enable SW reset */
  UCB0CTL1 |= UCSWRST;
  /* I2C master, synchronous mode */
  UCB0CTL0 |= (UCMST | UCMODE_3 | UCSYNC);
  /* Use SMCLK, keep SW reset */
  UCB0CTL1  = UCSSEL_2 + UCSWRST;
  UCB0BR0   = I2C_PRESC_100KHZ_LSB;
  UCB0BR1   = I2C_PRESC_100KHZ_MSB;
  UCB0I2CSA = slave_address;

  /* Clear SW reset, resume operation */
  UCB0CTL1 &= ~UCSWRST;
  UCB0IE = UCNACKIE;
  /* Enable TX ready interrupt */
  UCB0IE = UCTXIE;
}

uint8_t
i2c_receive_n(uint8_t len, uint8_t* rx_buf)
{

  rx_byte_total = len;
  rx_byte_counter = len;
  rx_buf_ptr  = rx_buf;

  /* Slave acks address? */
  while ((UCB0CTL1 & UCTXSTT) || (UCB0STAT & UCNACKIFG)) {}

  /*
   * Special case: stop condition must be sent while receiving the 1st byte for 1-byte only read operations.
   * See page 537 of slau144e.pdf
   */
  if(rx_byte_total == 1) {
    dint();
    /* I2C start condition */
    UCB0CTL1 |= UCTXSTT;

    /* Waiting for Start bit to clear */
    while(UCB0CTL1 & UCTXSTT) {}

    /* I2C stop condition */
    UCB0CTL1 |= UCTXSTP;
    eint();
  } else {
    /* I2C start condition */
    UCB0CTL1 |= UCTXSTT;
  }
  return 0;
}

uint8_t
i2c_busy(void)
{
  return (UCB0STAT & UCBBUSY);
}

void
i2c_enable(void)
{
  I2C_PxSEL |= (I2C_SDA | I2C_SCL);
}

void
i2c_disable(void)
{
  /*
   * GPIO function selected
   * Deactivate internal pull-up/-down resistors
   */
  I2C_PxSEL &= ~(I2C_SDA | I2C_SCL);
  I2C_PxREN &= ~(I2C_SDA | I2C_SCL);
  I2C_PxOUT &= ~(I2C_SDA | I2C_SCL);
}

void
i2c_transmit_n(uint8_t len, uint8_t* tx_buf)
{
  tx_byte_total = len;
  tx_byte_counter = len;
  tx_buf_ptr  = tx_buf;

  /* I2C TX, start condition */
  UCB0CTL1 |= UCTR + UCTXSTT;
}


ISR(USCI_B0, i2c_service_routine)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  switch (UCB0IV) {
    /* Transmit buffer empty */
    case USCI_I2C_UCTXIFG:
      if (tx_byte_counter == 0) {
        /* I2C stop condition */
        UCB0CTL1 |= UCTXSTP;
      } else {
        UCB0TXBUF = tx_buf_ptr[tx_byte_total - tx_byte_counter];
        tx_byte_counter--;
      }
      break;

    /* Data received */
    case USCI_I2C_UCRXIFG:
      rx_buf_ptr[rx_byte_total - rx_byte_counter] = UCB0RXBUF;
      rx_byte_counter--;
      /*
       * Stop condition should be set before receiving last byte.
       * For 1 byte transmissions it is handled in i2c_receive_n.
       */
      if (rx_byte_counter == 1){
        if (rx_byte_total != 1)
          /* I2C stop condition */
          UCB0CTL1 |= UCTXSTP;
      }
      break;
  }

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

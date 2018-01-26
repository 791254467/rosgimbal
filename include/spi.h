/**
 * STM32F4xx SPI driver for OpenPilot REVO
 *
 * Adapted from https://github.com/jihlein/AQ32Plus/blob/master/src/drv/drv_spi.h
 * 			and https://github.com/superjax/airbourne/blob/f4/f4/src/spi.cpp
 * @author len0rd
 * @since 2017-08-04
 */
#pragma once

#include "revo_f4.h"

#include "gpio.h"

class SPI
{

public:

  void init(const spi_hardware_struct_t *conf);

  inline bool is_busy() {return busy_;}
  void set_divisor(uint16_t new_divisor);
  void enable(GPIO& cs);
  void disable(GPIO& cs);
  bool transfer(uint8_t *out_data, uint32_t num_bytes, uint8_t* in_data, GPIO* cs = NULL);
  uint8_t transfer_byte(uint8_t data, GPIO* cs = NULL);
  void transfer_complete_cb();
  void register_complete_cb(void (*cb)(void));

private:
  bool busy_ = false;
  SPI_TypeDef*	dev;
  GPIO mosi_;
  GPIO miso_;
  GPIO sck_;

  GPIO* cs_;
  uint32_t errors_ = 0;

  DMA_InitTypeDef DMA_InitStructure_;
  const spi_hardware_struct_t* c_;

  void (*transfer_cb_)(void) = NULL;
};

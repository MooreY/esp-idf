// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _SPI_LCDX_H_
#define _SPI_LCDX_H_

#include "driver/gpio.h"
#include "driver/spi_master.h"

#define MISO            19
#define MOSI            23
#define SCLK            18

#define LCD_CS          5
#define LCD_DC          25
#define LCD_RST         27
#define LCD_BCKL        22

#define CLK_FREQ        26*1000*1000
#define BCKL_ACT_LVL    1
#define MY_SPI_HOST     (spi_host_device_t)VSPI_HOST


#ifdef __cplusplus
extern "C" {
#endif


void lcd_init(spi_device_handle_t *spi_wr_dev, int dma_chan);

void lcd_spi_pre_transfer_callback(spi_transaction_t *t);

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd);

void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /*_SPI_LCDX_H_*/

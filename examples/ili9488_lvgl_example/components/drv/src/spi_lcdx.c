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

#include "string.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "spi_lcdx.h"


// This struct stores a bunch of command values to be initialized for displays
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;


DRAM_ATTR static const lcd_init_cmd_t ili9488_init_cmds[] = {
    {0xE0, {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15}, // Positive Gamma Control-
    {0xE1, {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15}, // Negative Gamma Control-
    {0xC0, {0x17, 0x15}, 2},                // Power Control 1 - 
    {0xC1, {0x41}, 1},                      // Power Control 2 -
    {0xC5, {0x00, 0x12, 0x80}, 3},          // Power Control 3 -
    {0x36, {0x48}, 1},                      // Memory Access Control - 
    {0x3A, {0x66}, 1},                      // Pixel Format 18bpp -
    {0xB0, {0x80}, 1},                      // Interface Mode Control -
    {0xB1, {0xA0}, 1},                      // Frame Rate -
    {0xB4, {0x02}, 1},                      // Display Inversion Control -
    {0xB6, {0x02, 0x02}, 2},                // Display Function Control -
    {0xE9, {0x00}, 1},                      // Set Image Function -
    {0xF7, {0xA9, 0x51, 0x2C, 0x82}, 4},    // Adjust Control 3 (loose) -
    {0x11, {0}, 0x80},                      // Sleep Out -
    {0x29, {0}, 0x80},                      // Display On -
	{0x36, {0x28}, 1}, 	    				// Rotate display 90 degrees -
    {0, {0}, 0xFF},
};

/* This function is called (in irq context!) just before a transmission starts.
It will set the D/C line to the value indicated in the user field */
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(LCD_DC, dc);
}

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8,                    			// Command is 8 bits
    t.tx_buffer = &cmd,              			// The data is the cmd itself
    t.user = (void*)0,              			// D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;
    memset(&t, 0, sizeof(t));
    t.length = len * 8,              			// Len is in bytes, transaction length is in bits
    t.tx_buffer = data,              			// Data
    t.user = (void*)1,              			// D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void lcd_init(spi_device_handle_t *spi_wr_dev, int dma_chan)
{
    //Initialize non-SPI GPIOs
    gpio_pad_select_gpio(LCD_DC);
    gpio_set_direction(LCD_DC, GPIO_MODE_OUTPUT);

    // Hardware reset the display
    if (LCD_RST < GPIO_NUM_MAX) {
		ESP_LOGI("SPI_LCDX", "Hardware reset sequence for ILI9488.");
		gpio_pad_select_gpio(LCD_RST);
		gpio_set_direction(LCD_RST, GPIO_MODE_OUTPUT);
		gpio_set_level(LCD_RST, 1);
		vTaskDelay(5 / portTICK_RATE_MS);
		gpio_set_level(LCD_RST, 0);
		vTaskDelay(10 / portTICK_RATE_MS);
		gpio_set_level(LCD_RST, 1);
		vTaskDelay(120 / portTICK_RATE_MS);
    }
    else {
		ESP_LOGE("SPI_LCDX", "Choose another pin for LCD_RESET!");
        exit(0);
    }
    
    spi_bus_config_t buscfg = {
        .miso_io_num = MISO,
        .mosi_io_num = MOSI,
        .sclk_io_num = SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
		.max_transfer_sz = 65000					// Should keep it large !
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CLK_FREQ,                 // Clock out frequency
        .mode = 0,                                  // SPI mode 0
        .spics_io_num = LCD_CS,                     // CS pin
        .queue_size = 7,                            // We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback,    // Specify pre-transfer callback to handle D/C line
		.flags = SPI_DEVICE_HALFDUPLEX	            // Halfduplex mode
    };

    esp_err_t ret;
    ret = spi_bus_initialize(MY_SPI_HOST, &buscfg, dma_chan);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(MY_SPI_HOST, &devcfg, spi_wr_dev);
    ESP_ERROR_CHECK(ret);
    
    int cmd = 0;
    const lcd_init_cmd_t* lcd_init_cmds = NULL;
    lcd_init_cmds = ili9488_init_cmds;
    ESP_LOGI("SPI_LCDX", "ILI9488 initialization sequence.");
    assert(lcd_init_cmds != NULL);

    // Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(*spi_wr_dev, lcd_init_cmds[cmd].cmd);
        lcd_data(*spi_wr_dev, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(120 / portTICK_RATE_MS);
        }
        cmd++;
    }

    // Enable backlight
    if (LCD_BCKL < GPIO_NUM_MAX) {
        gpio_pad_select_gpio(LCD_BCKL);
        gpio_set_direction(LCD_BCKL, GPIO_MODE_OUTPUT);
        gpio_set_level(LCD_BCKL, (BCKL_ACT_LVL) & 0x1);
    }
    else {
        ESP_LOGE("SPI_LCDX", "Choose another pin for LCD_BCKL!");
        exit(0);
    }
}


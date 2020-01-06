/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "eeprom_test";

// I2C 
#define I2C_MASTER_TX_BUF_DISABLE 0 		/*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 		/*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  		/*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    		/*!< I2C master read */
#define ACK_CHECK_EN 0x1            		/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           		/*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 		/*!< I2C ack value */
#define NACK_VAL 0x1                		/*!< I2C nack value */

static gpio_num_t i2c_gpio_sda = 21;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

// EEPROM
#define EPR_ADR					0x50
#define EPR_BUF_SIZE			7
char epr_buff[EPR_BUF_SIZE];


////////////////////////////////////// I2C functions ///////////////////////////////////////
static esp_err_t i2c_master_driver_initialize()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = i2c_frequency
    };
    return i2c_param_config(i2c_port, &conf);
}

static esp_err_t i2c_set(uint8_t chip_addr, uint8_t data_addr, char *str, uint8_t len)
{
	esp_err_t ret;

    ret = i2c_master_driver_initialize();
	if(ret != ESP_OK) ESP_LOGE("i2c_write", "Master driver init error");

    ret = i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	if(ret != ESP_OK) ESP_LOGE("i2c_write", "Master driver install error");

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ret = i2c_master_start(cmd);
	if(ret != ESP_OK) ESP_LOGE("i2c_write", "Master start error");

    ret = i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
	if(ret != ESP_OK) ESP_LOGE("i2c_write", "Master write byte (chip address) error");

    ret = i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
	if(ret != ESP_OK) ESP_LOGE("i2c_write", "Master write byte (data address) error");

    for (int i = 0; i < len; i++) {
        ret = i2c_master_write_byte(cmd, str[i], ACK_CHECK_EN);
		if(ret != ESP_OK) ESP_LOGE("i2c_write", "Master write byte (data) error");
    }

    ret = i2c_master_stop(cmd);
	if(ret != ESP_OK) ESP_LOGE(TAG, "Master stop error");

    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
	switch(ret) {
		case ESP_ERR_INVALID_ARG: 	ESP_LOGE("i2c_write", "Parameter error."); break;
		case ESP_FAIL:				ESP_LOGE("i2c_write", "Sending command error, slave doesn’t ACK the transfer."); break;
		case ESP_ERR_INVALID_STATE: ESP_LOGE("i2c_write", "I2C driver not installed or not in master mode."); break;
		case ESP_ERR_TIMEOUT:		ESP_LOGE("i2c_write", "Operation timeout because the bus is busy."); break;
		default:					break;
	}

    i2c_cmd_link_delete(cmd);
    i2c_driver_delete(i2c_port);
	return ret;
}

static esp_err_t i2c_get(uint8_t chip_addr, uint8_t data_addr, uint8_t len)
{
    esp_err_t ret;

    uint8_t *data = malloc(len);
	if (data == NULL) { 
        ESP_LOGE("i2c_read", "Heap memory not allocated.");
        exit(0); 
    } 

    ret = i2c_master_driver_initialize();
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master driver init error");

    ret = i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master driver install error");

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ret = i2c_master_start(cmd);
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master start error");

    ret = i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master write byte (chip address) error");

    i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master write byte (data address) error");

    ret = i2c_master_start(cmd);

    ret = i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master read cmd error");
    if (len > 1) {
        ret = i2c_master_read(cmd, data, len - 1, ACK_VAL);
		if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master reading data error");
    }
    ret = i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master NACK error");

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
	if(ret != ESP_OK) ESP_LOGE("i2c_read", "Master cmd begin error");

    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        for (int i = 0; i < len; i++) {
	    	epr_buff[i] = data[i];		
	    	//printf("0x%02x ", epr_buff[i]);		
			printf("%c", epr_buff[i]);
            if ((i + 1) % EPR_BUF_SIZE == 0) {
                printf("\r\n");
            }
        }
        if (len % EPR_BUF_SIZE) {
            printf("\r\n");
        }
	}

    free(data);
    i2c_driver_delete(i2c_port);
    return ret;
}


//////////////////////////////////////// tasks //////////////////////////////////////
void eprom_task(void *pvParameter)
{
	esp_err_t ret;
	while(1) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		ret = i2c_get(EPR_ADR, 0x00, EPR_BUF_SIZE);
		if(ret != ESP_OK) 
			ESP_LOGE(TAG, "EEPROM read failure.");
	}
}


//////////////////////////////////////// main ///////////////////////////////////////
void app_main()
{
	// Write once, comment this section	and flash again to check 
	char tmp[] = { 'i', 'l', 'i', 'j', 'a', 'm', 'r' }; // define size in EPR_BUF_SIZE
	esp_err_t ret;
	ret = i2c_set(EPR_ADR, 0x00, tmp, sizeof(tmp));
	if(ret != ESP_OK) ESP_LOGE(TAG, "EEPROM write failure.");
	////////

	xTaskCreate(&eprom_task, "eprom_task", 2048, NULL, 1, NULL);
}



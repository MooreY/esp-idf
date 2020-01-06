/* 
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

static const char *TAG = "i2c_rtcc";

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

// RTC
#define RTC_ADR 							0x6F
#define RTC_BUF_SIZE 						8
uint8_t rtc_buff[RTC_BUF_SIZE];
uint8_t wkday_num;
char sec, min, hr, date, year, wkday, month;
char strTime[] = "00:46:00";
char strDate[] = "13.10.2019";
char strDay[] = "Ned.";
char sDay[] = "zzz.";


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
	    	rtc_buff[i] = data[i];		
	    	//printf("0x%02x ", rtc_buff[i]);		
            if ((i + 1) % RTC_BUF_SIZE == 0) {
                printf("\r\n");
            }
        }
        if (len % RTC_BUF_SIZE) {
            printf("\r\n");
        }
	}

    free(data);
    i2c_driver_delete(i2c_port);
    return ret;
}

/////////////////////////////////// RTC functions //////////////////////////////////
// Checks OSCRUN bit and returns 0 if OSC running
static int RTC_Is_Osc_Run() 
{
    if(rtc_buff[3] & 0x20) 
		return 0;
    else                   
		return -1;
}

// Sets ST bit                                              
static void RTC_Enable_Osc() 
{
    char mydata[] = { 0x80 };
    int ret = i2c_set(RTC_ADR, 0x00, mydata, sizeof(mydata));
    vTaskDelay(50 / portTICK_PERIOD_MS); // 32 cycles
    if(ret == 0) {
        ESP_LOGI(TAG, "RTC_Enable_Osc: RTC write success.");
    }
    else {
	ESP_LOGI(TAG, "RTC_Enable_Osc: RTC write failure.");
    }
}

// Clears ST bit
static void RTC_Disable_Osc() 
{
    char mydata[] = { 0x00 };
    int ret = i2c_set(RTC_ADR, 0x00, mydata, sizeof(mydata));
    vTaskDelay(50 / portTICK_PERIOD_MS); // 32 cycles
    if(ret == 0) {
        ESP_LOGI(TAG, "RTC_Disable_Osc: RTC write success.");
    }
    else {
		ESP_LOGI(TAG, "RTC_Disable_Osc: RTC write failure.");
    }
}

/*
// Writes week day to clear PWRFAIL bit
static int Clear_Power_Fail() 
{
    char mydata[] = { wkday | 0x08 };
    i2c_set(RTC_ADR, 0x03, mydata, sizeof(mydata));
    vTaskDelay(50 / portTICK_PERIOD_MS); // 32 cycles
    if(RTC_Is_Osc_Run() != 0) {
		ESP_LOGI(TAG, "RTC disabled, OSC is not running.");
		return 0;
    }
    else {
		return -1;
    }
}
*/

// Gets day of the week from number 1-7
static void GetWeekDay()
{
    switch(wkday_num) {
        case 1: strcpy(sDay, "Ned."); break;	// Sun
		case 2: strcpy(sDay, "Pon."); break;	// Mon
        case 3: strcpy(sDay, "Uto."); break;	// Tue
		case 4: strcpy(sDay, "Sri."); break;	// ...
		case 5: strcpy(sDay, "Cet."); break;
		case 6: strcpy(sDay, "Pet."); break;
		case 7: strcpy(sDay, "Sub."); break;
    }
}

// Formats time variables from RTC buffer values
static void RTC_Format_Time()
{
    char month_one, month_ten;
    sec       = (((rtc_buff[0] & 0x70) >> 4) * 10) + (rtc_buff[0] & 0x0F);
    min       = (((rtc_buff[1] & 0x70) >> 4) * 10) + (rtc_buff[1] & 0x0F);
    hr        = (((rtc_buff[2] & 0x30) >> 4) * 10) + (rtc_buff[2] & 0x0F);
    wkday_num =    rtc_buff[3] & 0x07;
    GetWeekDay();
    date      = (((rtc_buff[4] & 0x30) >> 4) * 10) + (rtc_buff[4] & 0x0F);
    month_ten = (rtc_buff[5] & 0x10) >> 4;
    month_one = rtc_buff[5] & 0x0F;
    month     = month_ten * 10 + month_one;
    year      = (((rtc_buff[6] & 0xF0) >> 4) * 10) + (rtc_buff[6] & 0x0F);
}

// Writes date&time to RTC, returns 0 on success
static int RTC_Write_Time_Date() 
{
    int ret;
    char tmp[7];  // Registers from 0x01 to 0x07
    // Convert from global time&date strings to writable buffer
    tmp[0] = ((strTime[3] - 48) << 4) + (strTime[4] - 48);                     // minutes
    tmp[1] = ((strTime[0] - 48) << 4) + (strTime[1] - 48);                     // hours

    // Determine week day number from strDay (other language should have different code)
    tmp[2] = 0;
    switch(strDay[0]) {
        case 'N': tmp[2] = 1; break;
        case 'U': tmp[2] = 3; break;
        case 'C': tmp[2] = 5; break;
        default:  {
               switch(strDay[1]) {
               case 'o': tmp[2] = 2; break;
               case 'r': tmp[2] = 4; break;
               case 'e': tmp[2] = 6; break;
               default:  tmp[2] = 7;
               }
        }
    }
    tmp[2] = tmp[2] | 0x08;

    tmp[3] = ((strDate[0] - 48) << 4) + (strDate[1] - 48);                     // date
    tmp[4] = ((strDate[3] - 48) << 4) + (strDate[4] - 48);                     // month
    tmp[5] = ((strDate[8] - 48) << 4) + (strDate[9] - 48);                     // year

    tmp[6] = 0x80;  // Control register (Ext. Osc enabled)

    // Write sequence
    RTC_Disable_Osc(); 
    ret = i2c_set(RTC_ADR, 0x01, tmp, sizeof(tmp));
    RTC_Enable_Osc();
    return ret;
}


//////////////////////////////////////// tasks //////////////////////////////////////
void i2c_task(void *pvParameter)
{
    int ret;

	// Uncomment this section to write time defined in strings above
	/*
    ret = RTC_Write_Time_Date();
    if(ret == 0) {
        ESP_LOGI(TAG, "RTC_Write_Time_Date: RTC write success.");
    }
    else {
		ESP_LOGE(TAG, "RTC_Write_Time_Date: RTC write failure.");
    }
    */

    while(1) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		ret = i2c_get(RTC_ADR, 0x00, RTC_BUF_SIZE);
        if(ret != 0) {
            ESP_LOGE(TAG, "RTC Task reading: RTC read failure.");
        }
		else {
			RTC_Format_Time();
			// Write time & date 
			printf("%d.%d.20%d. %d:%d:%d\r\n", date, month, year, hr, min, sec);
			if(RTC_Is_Osc_Run() != 0) { 
				printf("Osc is not running!!!\r\n");
			}
		}
    }
}

//////////////////////////////////////// main ///////////////////////////////////////
void app_main()
{
    xTaskCreate(&i2c_task, "i2c_task", 2048, NULL, 1, NULL);
}

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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "lvgl/lvgl.h"
#include "spi_lcdx.h"


#define LV_BUF_SZ		(480*20)

static lv_obj_t *chart = NULL;
static lv_chart_series_t *series = NULL;
static spi_device_handle_t spi;


void my_disp_flush(lv_disp_t* disp, const lv_area_t* area, lv_color_t* color_p)
{    
    lv_color16_t* buffer_16bit = (lv_color16_t*) color_p;
    uint8_t* mybuf = (uint8_t*) heap_caps_malloc(3 * LV_BUF_SZ * sizeof(uint8_t), MALLOC_CAP_DMA);
    
    if (mybuf == NULL) { 
        ESP_LOGE("MY_DISP_FLUSH", "Memory not allocated.");
        exit(0); 
    } 

    uint16_t LD = 0;
    int j = 0;
    for(int i = 0; i < LV_BUF_SZ; i++) {
        LD = buffer_16bit[i].full; 
		mybuf[j] = (uint8_t)((LD & 0xF800) >> 8);
        j++;
		mybuf[j] = (uint8_t)((LD & 0x07E0) >> 3);
        j++;    
		mybuf[j] = (uint8_t)((LD & 0x001F) << 3);
        j++;
    }

    esp_err_t ret;
    int x;
    static spi_transaction_t trans[6];

    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x & 1) == 0) {
        	// Even transfers are commands
            trans[x].length = 8;
            trans[x].user = (void*)0;
        } 
	    else {
            // Odd transfers are data
            trans[x].length = 8*4;
            trans[x].user = (void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }

    trans[0].tx_data[0] = 0x2A;           
    trans[1].tx_data[0] = area->x1 >> 8;       
    trans[1].tx_data[1] = area->x1 & 0xFF;      
    trans[1].tx_data[2] = area->x2 >> 8;       
    trans[1].tx_data[3] = area->x2 & 0xFF;     	
    trans[2].tx_data[0] = 0x2B;           		
    trans[3].tx_data[0] = area->y1 >> 8;       	
    trans[3].tx_data[1] = area->y1 & 0xFF;     	
    trans[3].tx_data[2] = area->y2 >> 8;    	
    trans[3].tx_data[3] = area->y2 & 0xFF;  	
    trans[4].tx_data[0] = 0x2C;           	
    trans[5].tx_buffer = mybuf;            		// Finally send the line data
    trans[5].length = 3 * LV_BUF_SZ * 8;  	    // Data length, in bits
    trans[5].flags = 0; 						// Undo SPI_TRANS_USE_TXDATA flag

    // Queue all transactions
    for (x=0; x<6; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    spi_transaction_t *rtrans;
    // Wait for all 6 transactions to be done and get back the results
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    heap_caps_free(mybuf);						// Free memory
    mybuf = NULL;
    lv_disp_flush_ready(disp);  				// Indicate you are ready with the flushing
}



void gui_create()
{    
	lv_init();
    
	// Init buffer
    static lv_disp_buf_t disp_buf;
    static lv_color_t buf1[LV_BUF_SZ];	
    lv_disp_buf_init(&disp_buf, buf1, NULL, LV_BUF_SZ);

	// Init display driver
    lv_disp_drv_t disp_drv;	
    lv_disp_drv_init(&disp_drv);
    disp_drv.buffer = &disp_buf;
    disp_drv.flush_cb = my_disp_flush;
    lv_disp_t* disp;
    disp = lv_disp_drv_register(&disp_drv);

    // Create gui
    lv_obj_t *scr = lv_obj_create(NULL, NULL);
    lv_scr_load(scr);

    lv_theme_t *th = lv_theme_alien_init(100, NULL);
    lv_theme_set_current(th);

    lv_obj_t *tabview = lv_tabview_create(lv_scr_act(), NULL);

    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, LV_SYMBOL_LOOP);
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, LV_SYMBOL_HOME);
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, LV_SYMBOL_SETTINGS);
    lv_tabview_set_tab_act(tabview, 1, false);   

    chart = lv_chart_create(tab2, NULL);
    lv_obj_set_size(chart, 300, 150);
    lv_chart_set_point_count(chart, 20);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(chart, (lv_chart_type_t)(LV_CHART_TYPE_POINT | LV_CHART_TYPE_LINE));
    lv_chart_set_series_opa(chart, LV_OPA_70);
    lv_chart_set_series_width(chart, 4);
    lv_chart_set_range(chart, 0, 100);
    series = lv_chart_add_series(chart, LV_COLOR_RED);
}

static void user_task(void *pvParameter)
{
    uint8_t value = 0;
    while (1) {
        value = esp_random() % 100;
        lv_chart_set_next(chart, series, value);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void tick_task(void *pvParameter)
{
    while(1) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
		lv_tick_inc(1); 
    }
}

void handler_task(void *pvParameter)
{
    while(1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
		lv_task_handler();
    }
}

void app_main()
{
    lcd_init(&spi, 1);
    ESP_LOGI("MAIN", "LCD initialized.");

    gui_create();
    ESP_LOGI("MAIN", "GUI created.");

    // These should match values in lv_conf.h
    ESP_LOGI("MAIN", "Color size is %d bits.", sizeof(lv_color_t) * 8);	
    ESP_LOGI("MAIN", "Horizontal resolution is %d pixels.", LV_HOR_RES);	
    ESP_LOGI("MAIN", "Vertical resolution is %d pixels.", LV_VER_RES);	

    xTaskCreate(&tick_task, "tick_task", 1024, NULL, 7, NULL);
    xTaskCreate(&handler_task, "handler_task", 2048, NULL, 5, NULL);
    xTaskCreate(&user_task, "user_task", 1024, NULL, 1, NULL);

    ESP_LOGI("MAIN", "ESP-IDF version: %s.", esp_get_idf_version());
    ESP_LOGI("MAIN", "Free heap memory: %d bytes.", esp_get_free_heap_size());
}





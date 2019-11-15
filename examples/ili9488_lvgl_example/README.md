	# ili9488_lvgl_example
  
    For use with esp-idf v4.1-dev (GNU MAKE) and lvgl version 6.0.
    Tested on DEVKITV1 (ESP32) & ER-TFTM035-6 (ILI9488) boards.
	Touch is not implemented yet.

	Download lvgl from https://littlevgl.com.
	Create directory lvgl-gui in esp/esp-idf/components
	and copy or git clone lvgl directory into it.
	Next to lvgl directory you should have lv_conf.h and component.mk files.

	# Component Makefile

	COMPONENT_SRCDIRS := .

	COMPONENT_ADD_INCLUDEDIRS = .

	COMPONENT_ADD_INCLUDEDIRS += \
		lvgl/src/lv_core \
		lvgl/src/lv_hal \
		lvgl/src/lv_objx \
		lvgl/src/lv_font \
		lvgl/src/lv_misc \
		lvgl/src/lv_themes \
		lvgl/src/lv_draw	

	COMPONENT_SRCDIRS += \
		lvgl/src/lv_core \
		lvgl/src/lv_hal \
		lvgl/src/lv_objx \
		lvgl/src/lv_font \
		lvgl/src/lv_misc \
		lvgl/src/lv_themes \
		lvgl/src/lv_draw


	File lv_conf.h comes with lvgl, set the following lines in it:

	#define LV_HOR_RES_MAX 				(480)
	#define LV_VER_RES_MAX 				(320)
	#define LV_COLOR_DEPTH     			16
	#define LV_DISP_DEF_REFR_PERIOD     200

	Pinout is in drv/include/spi_lcdx.h so change it if you need to.

	Run make menuconfig and make sure that /Components Config/FreeRTOS/Tick Rate/ 
	is set to 1000 Hz.

	Run make flash monitor.
	



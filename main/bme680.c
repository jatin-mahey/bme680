#include <stdint.h>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

void set_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_data uint8_t len);


void app_main()
{
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = 0,
		.scl_io_num = 7,
		.sda_io_num = 6,
		.glitch_ignore_cnt = 7,
	};
	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = 0x76,
		.scl_speed_hz = 100000,
	};
	i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

	// soft reset: reg=0xe0 cmd=0xb6
	printf("Resetting BME680...\n\r");
	uint8_t buf[2] = {0xe0, 0xb6};
	i2c_master_transmit(dev_handle, buf, 2, -1);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// set oversampling for temperature=x2 & pressure=x1: reg=0x74 data=0b01000100
	printf("\nSetting oversampling for temperature to x2 and pressure to x1...\n\r");
	set_reg(dev_handle, 0x74, 0b11111100, 0b01000100);

	// set oversampling for humidity=x16: reg=0x72 data=0b00000101
	printf("\nSetting oversampling for humidity to x16...\n\r");
	set_reg(dev_handle, 0x72, 0b00000111, 0b00000101);

	// set IIR filter for gas conversion: reg=0x75 data=0b00011100
	printf("\nSetting IIR filter value for gas conversion to 127...\n\r");
	set_reg(dev_handle, 0x75, 0b00011100, 0b00011100);

	// enable gas conversion and set index of heater set-point
	printf("\nEnabling gas conversion and setting heater set-point index to 0...\n\r");
	set_reg(dev_handle, 0x71, 0b00011111, 0b00010000);

	// set heater on time
	printf("\nSet heater on time to 100ms...\n\r");
	set_reg(dev_handle, 0x64, 0b11111111, 0b01011001);
	
	// set heating temperature
	printf("\nSet heating temperature to 300C...\n\r");
	set_reg(dev_handle, 0x5a, 0b11111111, 0b01011001);
	
	// define heater on time
	printf("\nDefine heater on time to 100ms...\n\r");
	set_reg(dev_handle, 0x64, 0b11111111, 0b01011001);
}


void set_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_data, uint8_t len)
{	
	uint8_t reg_value = 0;
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &reg_value, 1, -1));
	vTaskDelay(100 / portTICK_PERIOD_MS);
	printf("Old reg=%x with value=%x\n\r", reg_addr, reg_value);
	
	// setting the new reg value
	reg_value &= ~reg_mask; // clear old values
	reg_value |= reg_data; // set new values
	
	uint8_t buf[2] = {reg_addr, reg_value};
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &buf, 2, -1));
	vTaskDelay(100 / portTICK_PERIOD_MS);
	printf("New reg=%x with value=%x\n\r", reg_addr, reg_value);
}


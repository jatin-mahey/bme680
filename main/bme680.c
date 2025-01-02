#include <stdint.h>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

typedef struct readings {
	double temp_comp;
	double pres_comp;
	double humd_comp;
	double gas_comp;
} readings_t;

typedef struct adc_val {
	uint8_t temp_adclx;
	uint8_t temp_adcl;
	uint8_t temp_adcm;
	uint32_t temp_adc;
	uint8_t pres_adclx;
	uint8_t pres_adcl;
	uint8_t pres_adcm;
	uint32_t pres_adc;
	uint8_t humd_adcl;
	uint8_t humd_adcm;
	uint32_t humd_adc;

} adc_val_t;

typedef struct t_coff {
	uint16_t par_t1;
	uint8_t par_t1m;
	uint8_t par_t1l;
	uint16_t par_t2;
	uint8_t par_t2l;
	uint8_t par_t2m;
	uint8_t par_t3;
	uint8_t temp_adclx;
	uint8_t temp_adcl;
	uint8_t temp_adcm;
	uint32_t temp_adc;
} t_coff_t;

typedef struct p_coff{
	uint16_t par_p1;
	uint8_t par_p1l;
	uint8_t par_p1m;
	uint16_t par_p2;
	uint8_t par_p2l;
	uint8_t par_p2m;
	uint8_t par_p3;
	uint16_t par_p4;
	uint8_t par_p4l;
	uint8_t par_p4m;
	uint16_t par_p5;
	uint8_t par_p5l;
	uint8_t par_p5m;
	uint8_t par_p6;
	uint8_t par_p7;
	uint16_t par_p8;
	uint8_t par_p8l;
	uint8_t par_p8m;
	uint16_t par_p9;
	uint8_t par_p9l;
	uint8_t par_p9m;
	uint8_t par_p10;
	uint8_t press_adclx;
	uint8_t press_adcl;
	uint8_t press_adcm;
	uint32_t press_adc;
} p_coff_t;

typedef struct h_coff{
	uint16_t par_h1;
	uint8_t par_h1l;
	uint8_t par_h1m;
	uint16_t par_h2;
	uint8_t par_h2l;
	uint8_t par_h2m;
	uint8_t par_h3;
	uint8_t par_h4;
	uint8_t par_h5;
	uint8_t par_h6;
	uint8_t par_h7;
	uint8_t humd_adcl;
	uint8_t humd_adcm;
	uint16_t humd_adc;
} h_coff_t;

typedef struct g_coff{
	uint8_t par_g1;
	uint16_t par_g2;
	uint8_t par_g2l;
	uint8_t par_g2m;
	uint8_t par_g3;
	uint8_t res_heat_range;
	uint8_t res_heat_value;
} g_coff_t;


void set_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_data, uint8_t len);
void get_calib(i2c_master_dev_handle_t dev_handle, t_coff_t *tcoff, p_coff_t *pcoff, h_coff_t *hcoff, g_coff_t *gcoff);
void calc_res_heat(g_coff_t *gcoff, uint8_t *res_heat);
void calc_temp(t_coff_t *tcoff, adc_val_t *adc_val, readings_t *readings);
void calc_pres(p_coff_t *pcoff, adc_val_t *adc_val, readings_t *readings);
void calc_humd(h_coff_t *hcoff, adc_val_t *adc_val, readings_t *readings);
void calc_gas(h_coff_t *gcoff, adc_val_t *adc_val, readings_t *readings);
void get_adc(i2c_master_dev_handle_t dev_handle, adc_val_t *adc_val);


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

	// set calibration data
	t_coff_t tcoff;
	p_coff_t pcoff;
	h_coff_t hcoff;
	g_coff_t gcoff;
	get_calib(dev_handle, &tcoff, &pcoff, &hcoff, &gcoff);
	uint8_t res_heat;
	calc_res_heat(&gcoff, &res_heat);

	// set oversampling for temperature=x2 & pressure=x1: reg=0x74 data=0b01000100
	printf("\nSetting oversampling for temperature to x2 and pressure to x1...\n\r");
	set_reg(dev_handle, 0x74, 0b11111100, 0b10000100, 1);

	// set oversampling for humidity=x16: reg=0x72 data=0b00000101
	printf("\nSetting oversampling for humidity to x16...\n\r");
	set_reg(dev_handle, 0x72, 0b00000111, 0b00000101, 1);

	// set IIR filter for gas conversion: reg=0x75 data=0b00011100
	printf("\nSetting IIR filter value for gas conversion to 127...\n\r");
	set_reg(dev_handle, 0x75, 0b00011100, 0b00011100, 1);

	// enable gas conversion and set index of heater set-point
	printf("\nEnabling gas conversion and setting heater set-point index to 0...\n\r");
	set_reg(dev_handle, 0x71, 0b00011111, 0b00010000, 1);

	// set heater on time
	printf("\nSetting heater on time to 100ms...\n\r");
	set_reg(dev_handle, 0x64, 0b11111111, 0b01011001, 1);
	
	// set heater resistance
	printf("\nSetting heater resistance value...\n\r");
	set_reg(dev_handle, 0x5a, 0b11111111, &res_heat, 1);

	// get new adc values
	printf("\nGathering...\n\r");
	while (1) {
		// enable forced mode
		uint8_t reg_addr = 0x74;
		uint8_t reg_value = 0;
		ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &reg_value, 1, -1));
		vTaskDelay(100 / portTICK_PERIOD_MS);
		reg_value &= ~0b00000011; // clear old values
		reg_value |= 0b00000001; // set new values
		uint8_t buf[2] = {reg_addr, reg_value};
		ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &buf, 2, -1));
		vTaskDelay(100 / portTICK_PERIOD_MS);
	
		// check for new data
		reg_addr = 0x1d;
		reg_value = 0;
		while(reg_value == 0) {
			ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &reg_value, 1, -1));
			vTaskDelay(100 / portTICK_PERIOD_MS);
			reg_value &= 0b10000000;
		}
		
		// calculate and print new data
		adc_val_t adc_val;
		readings_t readings;
		get_adc(dev_handle, &adc_val);
		vTaskDelay(200 / portTICK_PERIOD_MS);
		calc_temp(&tcoff, &adc_val, &readings);
		calc_pres(&pcoff, &adc_val, &readings);
		calc_humd(&hcoff, &adc_val, &readings);
		printf("Temp=%.2f Press=%.2f Humd=%.2f\n\r",
			readings.temp_comp, readings.pres_comp, readings.humd_comp);
		
		// sleep mode
		reg_addr = 0x74;
		reg_value = 0;
		ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &reg_value, 1, -1));
		vTaskDelay(100 / portTICK_PERIOD_MS);
		reg_value &= ~0b00000011; // clear old values
		reg_value |= 0b00000000; // set new values
		buf[0] = reg_addr;
		buf[1] = reg_value;
		ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &buf, 2, -1));
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
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


void get_calib(i2c_master_dev_handle_t dev_handle, t_coff_t *tcoff, p_coff_t *pcoff, h_coff_t *hcoff, g_coff_t *gcoff)
{
	printf("\nGetting calibration cofficients set 1...\n\r");
	uint8_t reg_addr = 0x8a;
	uint8_t coff_1[23] = {0};
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &coff_1, sizeof(coff_1), -1));
	for (uint8_t i=0; i<23; i++) {
		printf("[%d] reg=0x%x value=%d\n\r", i, reg_addr+i, coff_1[i]);
	}

	printf("\nGetting calibration cofficients set 2...\n\r");
	reg_addr = 0xe1;
	uint8_t coff_2[14] = {0};
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &coff_2, sizeof(coff_2), -1));
	for (uint8_t i=0; i<14; i++) {
		printf("[%d] reg=0x%x value=%d\n\r", i, reg_addr+i, coff_2[i]);
	}

	printf("\nGetting calibration cofficients set 3...\n\r");
	reg_addr = 0x00;
	uint8_t coff_3[5] = {0};
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &coff_3, sizeof(coff_3), -1));
	for (uint8_t i=0; i<5; i++) {
		printf("[%d] reg=0x%x value=%d\n\r", i, reg_addr+i, coff_3[i]);
	}

	printf("\nGetting ADC Values...\n\r");
	reg_addr = 0x1f;
	uint8_t adc_val[7] = {0};
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &adc_val, sizeof(adc_val), -1));
	for (uint8_t i=0; i<7; i++) {
		printf("[%d] reg=0x%x value=%d\n\r", i, reg_addr+i, adc_val[i]);
	}
	
	tcoff->par_t1l = coff_2[8];
	tcoff->par_t1m = coff_2[9];
	tcoff->par_t1 = (coff_2[9] << 8) + coff_2[8];
	tcoff->par_t2l = coff_1[0];
	tcoff->par_t2m = coff_1[1];
	tcoff->par_t2 = (coff_1[1] << 8) + coff_1[0];
	tcoff->par_t3  = coff_1[2];

	pcoff->par_p1l = coff_1[4];
	pcoff->par_p1m = coff_1[5];
	pcoff->par_p1  = (coff_1[5] << 8) + coff_1[4];
	pcoff->par_p2l = coff_1[6];
	pcoff->par_p2m = coff_1[7];
	pcoff->par_p2  = (coff_1[7] << 8) + coff_1[6];
	pcoff->par_p3  = coff_1[8];
	pcoff->par_p4l = coff_1[10];
	pcoff->par_p4m = coff_1[11];
	pcoff->par_p4  = (coff_1[11] << 8) + coff_1[10];
	pcoff->par_p5l = coff_1[12];
	pcoff->par_p5m = coff_1[13];
	pcoff->par_p5  = (coff_1[13] << 8) + coff_1[12];
	pcoff->par_p6  = coff_1[15];
	pcoff->par_p7  = coff_1[14];
	pcoff->par_p8l = coff_1[18];
	pcoff->par_p8m = coff_1[19];
	pcoff->par_p8  = (coff_1[19] << 8) + coff_1[18];
	pcoff->par_p9l = coff_1[20];
	pcoff->par_p9m = coff_1[21];
	pcoff->par_p9  = (coff_1[21] << 8) + coff_1[20];
	pcoff->par_p10 = coff_1[22];
	
	hcoff->par_h1l = coff_2[1] & 0x01;
	hcoff->par_h1m = coff_2[2];
	hcoff->par_h1  = (coff_2[2] << 8) + (coff_2[1] & 0x01);
	hcoff->par_h2l = coff_2[1] >> 4;   // 7:4
	hcoff->par_h2m = coff_2[0];
	hcoff->par_h2  = (coff_2[0] << 8) + (coff_2[1] >> 4);
	hcoff->par_h3  = coff_2[3];
	hcoff->par_h4  = coff_2[4];
	hcoff->par_h5  = coff_2[5];
	hcoff->par_h6  = coff_2[6];
	hcoff->par_h7  = coff_2[7];

	gcoff->par_g1  = coff_2[12];
	gcoff->par_g2l = coff_2[10];
	gcoff->par_g2m = coff_2[11];
	gcoff->par_g2  = (coff_2[11] << 8) + coff_2[10];
	gcoff->par_g3  = coff_2[13];
	gcoff->res_heat_range = (coff_3[2] >> 4) & 0x03;
	gcoff->res_heat_value = coff_3[0];

}


void calc_res_heat(g_coff_t *gcoff, uint8_t *res_heat)
{
	printf("\nCalculating heater resistance value...\n\r");
	double var1 = ((double)gcoff->par_g1 / 16.0) + 49.0;
	double var2 = (((double)gcoff->par_g2 / 32768.0) * 0.0005) + 0.00235;
	double var3 = (double)gcoff->par_g3 / 1024.0;
	double var4 = var1 * (1.0 + (var2 * 300.0));
	double var5 = var4 + (var3 * 25.0);
	*res_heat = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + gcoff->res_heat_range)) * (1.0/(1.0 + (gcoff->res_heat_value *0.002)))) -25));
	printf("var1=%.2f\n\rvar2=%.2f\n\rvar3=%.2f\n\rvar4=%.2f\n\rvar5=%.2f\n\rres_heat=%d\n\r",
		var1, var2, var3, var4, var5, *res_heat);
}


void calc_temp(t_coff_t *tcoff, adc_val_t *adc_val, readings_t *readings)
{
	double var1 = (((double)adc_val->temp_adc / 16384.0) - ((double)tcoff->par_t1 / 1024.0)) * (double)tcoff->par_t2;
	double var2 = ((((double)adc_val->temp_adc / 131072.0) - ((double)tcoff->par_t1 / 8192.0)) *
		       (((double)adc_val->temp_adc / 131072.0) - ((double)tcoff->par_t1 / 8192.0))) * 
		      ((double)tcoff->par_t3 * 16.0);
	double t_fine = var1 + var2;
	readings->temp_comp = t_fine / 5120.0;
}


void calc_pres(p_coff_t *pcoff, adc_val_t *adc_val, readings_t *readings)
{
	double var1 = (readings->temp_comp * 2560.0) - 64000.0;
	double var2 = var1 * var1 * ((double)pcoff->par_p6 / 131072.0);
	var2 = var2 + (var1 * (double)pcoff->par_p5 * 2.0);
	var2 = (var2 / 4.0) + ((double)pcoff->par_p4 * 65536.0);
	var1 = ((((double)pcoff->par_p3 * var1 * var1) / 16384.0) +
		 ((double)pcoff->par_p2 * var1)) / 524288.0;
	var1 = (1.0 + (var1 / 32768.0)) * (double)pcoff->par_p1;

	double press_comp = 1048576.0 - (double)adc_val->pres_adc;
	press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;

	var1 = ((double)pcoff->par_p9 * press_comp * press_comp) / 2147483648.0;
	var2 = press_comp * ((double)pcoff->par_p8 / 32768.0);
	double var3 = (press_comp / 256.0) * (press_comp / 256.0) *
		 (press_comp / 256.0) * (pcoff->par_p10 / 131072.0);
	press_comp = press_comp + (var1 + var2 + var3 +
		     ((double)pcoff->par_p7 * 128.0)) / 16.0;
	readings->pres_comp = press_comp;
}


void calc_humd(h_coff_t *hcoff, adc_val_t *adc_val, readings_t *readings)
{
	double var1 = adc_val->humd_adc - (((double)hcoff->par_h1 * 16.0) + (((double)hcoff->par_h3 / 2.0) * readings->temp_comp));
	double var2 = var1 * (((double)hcoff->par_h2 / 262144.0) * (1.0 + (((double)hcoff->par_h4 / 16384.0) *
		readings->temp_comp) + (((double)hcoff->par_h5 / 1048576.0) * readings->temp_comp * readings->temp_comp)));
	double var3 = (double)hcoff->par_h6 / 16384.0;
	double var4 = (double)hcoff->par_h7 / 2097152.0;
	double hum_comp = var2 + ((var3 + (var4 * readings->temp_comp)) * var2 * var2);
	readings->humd_comp = hum_comp;
}


void calc_gas(h_coff_t *gcoff, adc_val_t *adc_val, readings_t *readings)
{
}


void get_adc(i2c_master_dev_handle_t dev_handle, adc_val_t *adc_val)
{
	printf("ADC Values: ");
	uint8_t reg_addr = 0x1f;
	uint8_t adc[8] = {0};
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &adc, sizeof(adc), -1));
	for (uint8_t i=0; i<8; i++) {
		printf("%d ", adc[i]);
	}
	adc_val->temp_adclx = adc[5]>>4;
	adc_val->temp_adcl  = adc[4];
	adc_val->temp_adcm  = adc[3];
	adc_val->temp_adc   = (adc[3]<<12) + (adc[4]<<4) + (adc[5]>>4);

	adc_val->pres_adclx = adc[2]>>4;
	adc_val->pres_adcl  = adc[1];
	adc_val->pres_adcm  = adc[0];
	adc_val->pres_adc   = (adc[0]<<12) + (adc[1]<<4) + (adc[2]>>4);

	adc_val->humd_adcl  = adc[7];
	adc_val->humd_adcm  = adc[6];
	adc_val->humd_adc   = (adc[6]<<8) + adc[7];
}

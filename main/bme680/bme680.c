
#include "bme680.h"


void bme680_init(i2c_interface_t* i2c_interface,
                 t_coff_t* tcoff, p_coff_t* pcoff, h_coff_t* hcoff, g_coff_t* gcoff)
{
    uint8_t res_heat;

    printf("\nResetting BME680...\n\r");
    i2c_interface->i2c_transmit(BME680_RESET_REG, BME680_SOFT_RESET_VAL);
    i2c_interface->delay_ms(100);

    printf("Getting calibration data...\n\r");
    get_calib(i2c_interface, tcoff, pcoff, hcoff, gcoff);

    printf("Calculating heater resistance value...\n\r");
    calc_res_heat(gcoff, &res_heat);

    printf("Setting oversampling for temperature to x2...\n\r");
    set_reg(i2c_interface, BME680_CTRL_MEAS_REG, BME680_OSRS_T_MASK, BME680_OSRS_X2_VAL);

    printf("Setting oversampling for pressure to x1...\n\r");
    set_reg(i2c_interface, BME680_CTRL_MEAS_REG, BME680_OSRS_P_MASK, BME680_OSRS_X1_VAL);

    printf("Setting oversampling for humidity to x16...\n\r");
    set_reg(i2c_interface, BME680_CTRL_HUM_REG, BME680_OSRS_H_MASK, BME680_OSRS_X16_VAL);

    printf("Setting IIR filter value for gas conversion to 127...\n\r");
    set_reg(i2c_interface, BME680_CONFIG_REG,BME680_FILTER_MASK, BME680_FILTER_127_VAL);

    printf("Enabling gas conversion...\n\r");
    set_reg(i2c_interface, BME680_CTRL_GAS_REG, BME680_RUN_GAS_MASK, BME680_GAS_ENABLE_VAL);

    printf("Setting heater set-point index to 0...\n\r");
    set_reg(i2c_interface, BME680_CTRL_GAS_REG, BME680_NB_CONV_MASK, 0);

    printf("Setting heater timer prescaler to x4...\n\r");
    set_reg(i2c_interface, BME680_GAS_WAIT_REG, BME680_GAS_WAIT_MUL_MASK, BME680_GAS_WAIT_MUL_X4_VAL);

    printf("Setting heater timer load to 25ms, this will create total 100ms timer...\n\r");
    set_reg(i2c_interface, BME680_GAS_WAIT_REG, BME680_GAS_WAIT_LOAD_MASK, 25);

    printf("Setting heater resistance value...\n\r");
    set_reg(i2c_interface, BME680_REST_HEAT_REG, 0xFF, res_heat);
}


void get_readings(i2c_interface_t* i2c_interface,
                  t_coff_t* tcoff, p_coff_t* pcoff, h_coff_t* hcoff, g_coff_t* gcoff,
                  readings_t* readings)
{
    // set force mode
    set_reg(i2c_interface, BME680_CTRL_MEAS_REG, BME680_MODE_MASK, BME680_MODE_FORCE_SET);

    // check for new data
    uint8_t reg_value = 0;
    while(reg_value == 0) {
        i2c_interface->i2c_receive(BME680_EAS_STATUS_REG, &reg_value, 1);
        i2c_interface->delay_ms(100);
        reg_value &= BME680_NEW_DATA_MASK;
    }
    // get ADC data
    adc_val_t adc_val;
    get_adc(i2c_interface, &adc_val);
    i2c_interface->delay_ms(100);

    // calculate new readings
    calc_temp ( tcoff,  &adc_val, readings );
    calc_pres ( pcoff,  &adc_val, readings );
    calc_humd ( hcoff,  &adc_val, readings );
    calc_gas  ( gcoff,  &adc_val, readings );

    // set sleep mode
    set_reg(i2c_interface, BME680_CTRL_MEAS_REG, BME680_MODE_MASK, BME680_MODE_SLEEP_SET);
}


void get_calib(i2c_interface_t* i2c_interface,
               t_coff_t* tcoff, p_coff_t* pcoff, h_coff_t* hcoff, g_coff_t* gcoff)
{
    // Get 1st Cofficients set
    uint8_t reg_addr = BME680_COFF_SET_1_ADDR;
    uint8_t coff_1[BME680_COFF_SET_1_LEN] = {0};
    i2c_interface->i2c_receive(reg_addr, coff_1, sizeof(coff_1));

    // Get 2nd Cofficient set
    reg_addr = BME680_COFF_SET_2_ADDR;
    uint8_t coff_2[BME680_COFF_SET_2_LEN] = {0};
    i2c_interface->i2c_receive(reg_addr, coff_2, sizeof(coff_2));

    // Get 3rd Cofficient set
    reg_addr = BME680_COFF_SET_3_ADDR;
    uint8_t coff_3[BME680_COFF_SET_3_LEN] = {0};
    i2c_interface->i2c_receive(reg_addr, coff_3, sizeof(coff_3));

    // Map Cofficient data to structs
    tcoff->par_t1l =   coff_2[8];
    tcoff->par_t1m =   coff_2[9];
    tcoff->par_t1  = ( coff_2[9] << 8) + coff_2[8];
    tcoff->par_t2l =   coff_1[0];
    tcoff->par_t2m =   coff_1[1];
    tcoff->par_t2  = ( coff_1[1] << 8) + coff_1[0];
    tcoff->par_t3  =   coff_1[2];

    pcoff->par_p1l =   coff_1[4];
    pcoff->par_p1m =   coff_1[5];
    pcoff->par_p1  = ( coff_1[5] << 8) + coff_1[4];
    pcoff->par_p2l =   coff_1[6];
    pcoff->par_p2m =   coff_1[7];
    pcoff->par_p2  = ( coff_1[7] << 8) + coff_1[6];
    pcoff->par_p3  =   coff_1[8];
    pcoff->par_p4l =   coff_1[10];
    pcoff->par_p4m =   coff_1[11];
    pcoff->par_p4  = ( coff_1[11] << 8) + coff_1[10];
    pcoff->par_p5l =   coff_1[12];
    pcoff->par_p5m =   coff_1[13];
    pcoff->par_p5  = ( coff_1[13] << 8) + coff_1[12];
    pcoff->par_p6  =   coff_1[15];
    pcoff->par_p7  =   coff_1[14];
    pcoff->par_p8l =   coff_1[18];
    pcoff->par_p8m =   coff_1[19];
    pcoff->par_p8  = ( coff_1[19] << 8) + coff_1[18];
    pcoff->par_p9l =   coff_1[20];
    pcoff->par_p9m =   coff_1[21];
    pcoff->par_p9  = ( coff_1[21] << 8) + coff_1[20];
    pcoff->par_p10 =   coff_1[22];

    hcoff->par_h1l =   coff_2[1] & 0x01;
    hcoff->par_h1m =   coff_2[2];
    hcoff->par_h1  = ( coff_2[2] << 8) + (coff_2[1] & 0x01);
    hcoff->par_h2l =   coff_2[1] >> 4;   // 7:4
    hcoff->par_h2m =   coff_2[0];
    hcoff->par_h2  = ( coff_2[0] << 8) + (coff_2[1] >> 4);
    hcoff->par_h3  =   coff_2[3];
    hcoff->par_h4  =   coff_2[4];
    hcoff->par_h5  =   coff_2[5];
    hcoff->par_h6  =   coff_2[6];
    hcoff->par_h7  =   coff_2[7];

    gcoff->par_g1  =   coff_2[12];
    gcoff->par_g2l =   coff_2[10];
    gcoff->par_g2m =   coff_2[11];
    gcoff->par_g2  = ( coff_2[11] << 8) + coff_2[10];
    gcoff->par_g3  =   coff_2[13];

    gcoff->res_heat_range        = ( coff_3[2] >> 4) & 0x03;
    gcoff->res_heat_value        =   coff_3[0];
    gcoff->range_switching_error =   coff_3[4];
}


void get_adc(i2c_interface_t* i2c_interface, adc_val_t* adc_val)
{
    uint8_t adc[BME680_ADC_VALUES_LEN] = {0};
    i2c_interface->i2c_receive(BME680_ADC_VALUES_ADDR, adc, sizeof(adc));

    adc_val->temp_adclx =   adc[5]>>4;
    adc_val->temp_adcl  =   adc[4];
    adc_val->temp_adcm  =   adc[3];
    adc_val->temp_adc   = ( adc[3]<<12 ) + ( adc[4]<<4 ) + ( adc[5]>>4 );

    adc_val->pres_adclx =   adc[2]>>4;
    adc_val->pres_adcl  =   adc[1];
    adc_val->pres_adcm  =   adc[0];
    adc_val->pres_adc   = ( adc[0]<<12 ) + ( adc[1]<<4 ) + ( adc[2]>>4 );

    adc_val->humd_adcl  =   adc[7];
    adc_val->humd_adcm  =   adc[6];
    adc_val->humd_adc   = ( adc[6]<<8 ) + adc[7];

    uint8_t gas[BME680_GAS_VALUES_LEN] = {0};
    i2c_interface->i2c_receive(BME680_GAS_VALUES_ADDR, gas, sizeof(gas));

    adc_val->gas_adcm  =   gas[0];
    adc_val->gas_adcl  =   gas[1]>>6;
    adc_val->gas_adc   = ( gas[0]<<2 ) + ( gas[1]>>6 );
    adc_val->gas_range =   gas[1] & 0x01;
}


void calc_res_heat(g_coff_t* gcoff, uint8_t* res_heat)
{
    double var1 = ((double)gcoff->par_g1 / 16.0) + 49.0;
    double var2 = (((double)gcoff->par_g2 / 32768.0) * 0.0005) + 0.00235;
    double var3 = (double)gcoff->par_g3 / 1024.0;
    double var4 = var1 * (1.0 + (var2 * 300.0));
    double var5 = var4 + (var3 * 25.0);
    *res_heat   = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + gcoff->res_heat_range)) *
                                           (1.0 / (1.0 + (gcoff->res_heat_value *0.002)))) -25));
}


void calc_temp(t_coff_t *tcoff, adc_val_t* adc_val, readings_t* readings)
{
    double var1   = (((double)adc_val->temp_adc / 16384.0) - ((double)tcoff->par_t1 / 1024.0)) *
                    (double)tcoff->par_t2;
    double var2   = ((((double)adc_val->temp_adc / 131072.0) - ((double)tcoff->par_t1 / 8192.0)) *
                     (((double)adc_val->temp_adc / 131072.0) - ((double)tcoff->par_t1 / 8192.0))) *
                    ((double)tcoff->par_t3 * 16.0);
    double t_fine = var1 + var2;
    readings->temp_comp = t_fine / 5120.0;
}


void calc_pres(p_coff_t* pcoff, adc_val_t* adc_val, readings_t* readings)
{
    double var1 = (readings->temp_comp * 2560.0) - 64000.0;
    double var2 = var1 * var1 * ((double)pcoff->par_p6 / 131072.0);

    var2 = var2 + (var1 * (double)pcoff->par_p5 * 2.0);
    var2 = (var2 / 4.0) + ((double)pcoff->par_p4 * 65536.0);
    var1 = ((((double)pcoff->par_p3 * var1 * var1) / 16384.0) +
            ((double)pcoff->par_p2 * var1)) / 524288.0;
    var1 = (1.0 + (var1 / 32768.0)) * (double)pcoff->par_p1;

    double press_comp = 1048576.0 - (double)adc_val->pres_adc;
    press_comp        = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;

    var1 = ((double)pcoff->par_p9 * press_comp * press_comp) / 2147483648.0;
    var2 = press_comp * ((double)pcoff->par_p8 / 32768.0);
    double var3 = (press_comp / 256.0) * (press_comp / 256.0) *
                  (press_comp / 256.0) * (pcoff->par_p10 / 131072.0);

    press_comp  = press_comp + (var1 + var2 + var3 +
                  ((double)pcoff->par_p7 * 128.0)) / 16.0;
    readings->pres_comp = press_comp;
}


void calc_humd(h_coff_t* hcoff, adc_val_t* adc_val, readings_t* readings)
{
    double var1 = adc_val->humd_adc - (((double)hcoff->par_h1 * 16.0) +
                  (((double)hcoff->par_h3 / 2.0) * readings->temp_comp));
    double var2 = var1 * (((double)hcoff->par_h2 / 262144.0) *
                  (1.0 + (((double)hcoff->par_h4 / 16384.0) *
                  readings->temp_comp) + (((double)hcoff->par_h5 / 1048576.0) *
                  readings->temp_comp * readings->temp_comp)));
    double var3 = (double)hcoff->par_h6 / 16384.0;
    double var4 = (double)hcoff->par_h7 / 2097152.0;
    double hum_comp = var2 + ((var3 + (var4 * readings->temp_comp)) * var2 * var2);
    readings->humd_comp = hum_comp;
}


void calc_gas(g_coff_t* gcoff, adc_val_t* adc_val, readings_t* readings)
{
    double const_array1[16] = { 1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1 };
    double const_array2[16] = { 8000000, 4000000, 2000000, 1000000, 499500.4995, 248262.1648, 125000,
                                63004.03226, 31281.28128, 15625, 7812.5, 3906.25, 1953.125, 976.5625,
                                488.28125, 244.140625 };
    double var1    = (1340.0 + 5.0 * gcoff->range_switching_error) * const_array1[adc_val->gas_range];
    double gas_res = var1 * const_array2[adc_val->gas_range] / (adc_val->gas_adc - 512.0 + var1);
    readings->gas_comp = gas_res;
}


void set_reg(i2c_interface_t* i2c_interface,
             uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_data)
{
    uint8_t reg_value = 0;
    // getting the old data
    i2c_interface->i2c_receive(reg_addr, &reg_value, 1);
    i2c_interface->delay_ms(100);

    // setting the new reg value
    reg_value &= ~reg_mask; // clear old values
    reg_value |= reg_data; // set new values

    // transmitting the new data
    i2c_interface->i2c_transmit(reg_addr, reg_value);
    i2c_interface->delay_ms(100);
}

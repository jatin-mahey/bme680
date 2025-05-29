
#include <stdint.h>
#include <stdio.h>

#define BME680_RESET_REG           0xE0
#define BME680_RESET_DEFAULT_VAL   0x00
#define BME680_SOFT_RESET_VAL      0xB6

#define BME680_EAS_STATUS_REG      0x1D
#define BME680_NEW_DATA_MASK       0b10000000
#define BME680_GAS_MEASURING_MASK  0b10000000
#define BME680_MEASURING_MASK      0b10000000
#define BME680_GAS_MEAS_INDEX_MASK 0b10000000

#define BME680_CTRL_MEAS_REG       0x74
#define BME680_OSRS_T_MASK         0b11100000
#define BME680_OSRS_P_MASK         0b00011100
#define BME680_OSRS_X1_VAL         0b000
#define BME680_OSRS_X2_VAL         0b001
#define BME680_OSRS_X4_VAL         0b010
#define BME680_OSRS_X8_VAL         0b011
#define BME680_OSRS_X16_VAL        0b100

#define BME680_MODE_MASK           0b00000011
#define BME680_MODE_FORCE_SET      0b00000001
#define BME680_MODE_SLEEP_SET      0b00000000

#define BME680_CTRL_HUM_REG        0x72
#define BME680_OSRS_H_MASK         0b00000111

#define BME680_CONFIG_REG          0x75
#define BME680_FILTER_MASK         0b00011100
#define BME680_FILTER_127_VAL      0b111

#define BME680_CTRL_GAS_REG        0x71
#define BME680_NB_CONV_MASK        0b00001111
#define BME680_RUN_GAS_MASK        0b00010000
#define BME680_GAS_ENABLE_VAL      1
#define BME680_GAS_DISABLE_VAL     0

#define BME680_GAS_WAIT_REG           0x64
#define BME680_GAS_WAIT_LOAD_MASK     0b00111111
#define BME680_GAS_WAIT_MUL_MASK      0b11000000
#define BME680_GAS_WAIT_MUL_X1_VAL    0b00
#define BME680_GAS_WAIT_MUL_X4_VAL    0b01
#define BME680_GAS_WAIT_MUL_X16_VAL   0b10
#define BME680_GAS_WAIT_MUL_X64_VAL   0b11

#define BME680_REST_HEAT_REG       0x5A

#define BME680_COFF_SET_1_ADDR     0x8A
#define BME680_COFF_SET_1_LEN      23
#define BME680_COFF_SET_2_ADDR     0xE1
#define BME680_COFF_SET_2_LEN      14
#define BME680_COFF_SET_3_ADDR     0x00
#define BME680_COFF_SET_3_LEN      5
#define BME680_ADC_VALUES_ADDR     0x1F
#define BME680_ADC_VALUES_LEN      7
#define BME680_GAS_VALUES_ADDR     0x2A
#define BME680_GAS_VALUES_LEN      2


typedef struct
{
    void (* i2c_transmit )( uint8_t reg_addr, uint8_t  reg_data );
    void (* i2c_receive  )( uint8_t reg_addr, uint8_t* reg_data, uint8_t size );
    void (* delay_ms     )( uint32_t delay);
} i2c_interface_t;


typedef struct readings {
    double temp_comp;
    double pres_comp;
    double humd_comp;
    double gas_comp;
} readings_t;


typedef struct adc_val {
    uint32_t temp_adc;  uint8_t  temp_adcm;  uint8_t  temp_adcl;  uint8_t  temp_adclx;
    uint32_t pres_adc;  uint8_t  pres_adcm;  uint8_t  pres_adcl;  uint8_t  pres_adclx;
    uint32_t humd_adc;  uint8_t  humd_adcm;  uint8_t  humd_adcl;
    uint16_t gas_adc;   uint8_t  gas_adcm;   uint8_t  gas_adcl;
    uint8_t  gas_range;
} adc_val_t;


typedef struct t_coff {
    uint16_t par_t1;    uint8_t  par_t1m;    uint8_t  par_t1l;
    uint16_t par_t2;    uint8_t  par_t2m;    uint8_t  par_t2l;
    uint8_t  par_t3;
    uint32_t temp_adc;  uint8_t  temp_adcm;  uint8_t  temp_adcl;  uint8_t  temp_adclx;
} t_coff_t;


typedef struct p_coff {
    uint16_t par_p1;    uint8_t  par_p1m;    uint8_t  par_p1l;
    uint16_t par_p2;    uint8_t  par_p2m;    uint8_t  par_p2l;
    uint8_t  par_p3;
    uint16_t par_p4;    uint8_t  par_p4m;    uint8_t  par_p4l;
    uint16_t par_p5;    uint8_t  par_p5m;    uint8_t  par_p5l;
    uint8_t  par_p6;
    uint8_t  par_p7;
    uint16_t par_p8;    uint8_t  par_p8m;    uint8_t  par_p8l;
    uint16_t par_p9;    uint8_t  par_p9m;    uint8_t  par_p9l;
    uint8_t  par_p10;
    uint32_t press_adc; uint8_t  press_adcm; uint8_t  press_adcl; uint8_t  press_adclx;
} p_coff_t;


typedef struct h_coff {
    uint16_t par_h1;    uint8_t  par_h1m;    uint8_t  par_h1l;
    uint16_t par_h2;    uint8_t  par_h2m;    uint8_t  par_h2l;
    uint8_t  par_h3;
    uint8_t  par_h4;
    uint8_t  par_h5;
    uint8_t  par_h6;
    uint8_t  par_h7;
    uint16_t humd_adc;  uint8_t  humd_adcm;  uint8_t  humd_adcl;
} h_coff_t;


typedef struct g_coff {
    uint8_t  par_g1;
    uint16_t par_g2;    uint8_t  par_g2m;    uint8_t  par_g2l;
    uint8_t  par_g3;
    uint8_t  res_heat_range;
    uint8_t  res_heat_value;
    uint8_t  gas_range;
    uint8_t  range_switching_error;
} g_coff_t;


void bme680_init   ( i2c_interface_t * i2c_interface, t_coff_t  * tcoff,    p_coff_t   * pcoff,    h_coff_t * hcoff,    g_coff_t * gcoff );
void get_calib     ( i2c_interface_t * i2c_interface, t_coff_t  * tcoff,    p_coff_t   * pcoff,    h_coff_t * hcoff,    g_coff_t * gcoff );
void get_readings  ( i2c_interface_t * i2c_interface, t_coff_t  * tcoff,    p_coff_t   * pcoff,    h_coff_t * hcoff,    g_coff_t * gcoff,
                     readings_t      * readings                                                                                          );
void get_adc       ( i2c_interface_t * i2c_interface, adc_val_t * adc_val                                                                );
void calc_res_heat ( g_coff_t        * gcoff,         uint8_t   * res_heat                                                               );
void calc_temp     ( t_coff_t        * tcoff,         adc_val_t * adc_val,  readings_t * readings                                        );
void calc_pres     ( p_coff_t        * pcoff,         adc_val_t * adc_val,  readings_t * readings                                        );
void calc_humd     ( h_coff_t        * hcoff,         adc_val_t * adc_val,  readings_t * readings                                        );
void calc_gas      ( g_coff_t        * gcoff,         adc_val_t * adc_val,  readings_t * readings                                        );
void set_reg       ( i2c_interface_t * i2c_interface, uint8_t     reg_addr, uint8_t      reg_mask, uint8_t    reg_data                   );

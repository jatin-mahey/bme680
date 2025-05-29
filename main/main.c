
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "bme680/bme680.h"

#define I2C_DEV_ADDR    0x76
#define I2C_PORT_NUM    0
#define I2C_SCL_PIN     5
#define I2C_SDA_PIN     7
#define I2C_CLK_SPEED   100000

static i2c_master_dev_handle_t dev_handle;

void i2c_init     ( void );
void i2c_transmit ( uint8_t reg_addr, uint8_t  reg_data );
void i2c_receive  ( uint8_t reg_addr, uint8_t* reg_data, uint8_t size );
void delay_ms     ( uint32_t delay);


void app_main()
{
    i2c_init();
    i2c_interface_t i2c_interface = {
        .i2c_transmit = i2c_transmit,
        .i2c_receive  = i2c_receive,
        .delay_ms     = delay_ms
    };
    t_coff_t tcoff;
    p_coff_t pcoff;
    h_coff_t hcoff;
    g_coff_t gcoff;
    bme680_init(&i2c_interface, &tcoff, &pcoff, &hcoff, &gcoff);

    printf("Initialization successful, starting to gather...\n\n\r");
    while (1) {
        readings_t readings;
        get_readings(&i2c_interface, &tcoff, &pcoff, &hcoff, &gcoff, &readings);
        printf("%.2f, %.2f, %.2f, %.2f\n\r",
               readings.temp_comp,
               readings.pres_comp,
               readings.humd_comp,
               readings.gas_comp
        );
        delay_ms(2000);
    }
}


void i2c_init( void )
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_PORT_NUM,
        .scl_io_num        = I2C_SCL_PIN,
        .sda_io_num        = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&i2c_mst_config, &bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = I2C_DEV_ADDR,
        .scl_speed_hz    = I2C_CLK_SPEED,
    };
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}


void i2c_receive(uint8_t reg_addr, uint8_t* reg_data, uint8_t size)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, reg_data, size, -1));
}


void i2c_transmit(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t buf[2] = {reg_addr, reg_data};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
}


void delay_ms(uint32_t delay)
{
    vTaskDelay(delay / portTICK_PERIOD_MS);
}

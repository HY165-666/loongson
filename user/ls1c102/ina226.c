#include "ina226.h"
#include "ls1c102_i2c.h"   // 你的硬件 I2C 驱动头文件
#include "config.h"
#include "test.h"




// 底层 I2C 读写函数（你需要确保这些函数可用）
static void ina226_write_reg(uint8_t reg, uint16_t value);
static uint16_t ina226_read_reg(uint8_t reg);

void ina226_init(void)
{
    // 1. 复位设备
    ina226_write_reg(INA226_REG_CONFIG, 0x8000);
    delay_ms(10);
    
    // 2. 配置工作模式
    ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_VALUE);
    
    // 3. 写入校准值（关键！）
    ina226_write_reg(INA226_REG_CAL, INA226_CAL_VALUE);
}

// 总线电压，单位 mV
uint16_t ina226_get_bus_voltage_mV(void)
{
    uint16_t raw = ina226_read_reg(INA226_REG_BUS);
    // 低3位无效，右移3位，然后乘以1.25mV/LSB
    // 用整数运算：乘以5再除以4
    uint16_t mV = (uint16_t)(((uint32_t)raw * 5 / 4));
    return mV;
}

// 电流，单位 mA（有符号）
int16_t ina226_get_current_mA(void)
{
    int16_t raw = (int16_t)ina226_read_reg(INA226_REG_CURRENT);
    // 因为校准后 LSB = 1mA，直接返回
    return raw / 10;
}

// 功率，单位 mW
uint32_t ina226_get_power_mW(void)
{
    uint16_t raw = ina226_read_reg(INA226_REG_POWER);
    // 功率 LSB = 25 * 电流LSB = 25mW
    return ((uint32_t)raw * 5)/2;
}

void ina226_read_all(uint16_t *voltage_mV, int16_t *current_mA, uint32_t *power_mW)
{
    if (voltage_mV) *voltage_mV = ina226_get_bus_voltage_mV();
    if (current_mA) *current_mA = ina226_get_current_mA();
    if (power_mW)   *power_mW   = ina226_get_power_mW();
}

// ========== 底层 I2C 读写实现（请与你的硬件驱动对接）==========
static void ina226_write_reg(uint8_t reg, uint16_t value)
{
    // 参考之前的实现，需要调用 I2C_GenerateSTART, I2C_SendData 等
    // 这里假设你已经有了正确的 I2C 发送函数
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = value >> 8;
    buf[2] = value & 0xFF;
    
    I2C_GenerateSTART(I2C, ENABLE);
    I2C_Send7bitAddress(I2C, INA226_ADDR << 1, I2C_Direction_Transmitter);
    I2C_wait(I2C);
    for (int i = 0; i < 3; i++) {
        I2C_SendData(I2C, buf[i]);
        I2C_wait(I2C);
    }
    I2C_GenerateSTOP(I2C, ENABLE);
    I2C_wait(I2C);
}

static uint16_t ina226_read_reg(uint8_t reg)
{
    uint16_t value = 0;
    
    I2C_GenerateSTART(I2C, ENABLE);
    I2C_Send7bitAddress(I2C, INA226_ADDR << 1, I2C_Direction_Transmitter);
    I2C_wait(I2C);
    I2C_SendData(I2C, reg);
    I2C_wait(I2C);
    
    I2C_GenerateSTART(I2C, ENABLE);
    I2C_Send7bitAddress(I2C, INA226_ADDR << 1, I2C_Direction_Receiver);
    I2C_wait(I2C);
    
    // 读高字节，ACK
    I2C_ReceiveData(I2C, ENABLE, DISABLE);
    I2C_wait(I2C);
    value = (uint16_t)I2C->DR << 8;
    
    // 读低字节，NACK + STOP
    I2C_ReceiveData(I2C, DISABLE, ENABLE);
    I2C_wait(I2C);
    value |= I2C->DR;
    
    return value;
}


void ina226_i2c_init(void)
{
    I2C_InitTypeDef i2c_config;
    I2C_StructInit(&i2c_config);
    i2c_config.I2C_ClockSpeed = 100000;
    i2c_config.I2C_Mode = I2C_Mode_Master;
    i2c_config.I2C_BuslockCheckEn = I2C_Buslock_Check_Enable;
    I2C_Init(I2C, &i2c_config);
    
    // 2. INA226 初始化
    ina226_init();
}
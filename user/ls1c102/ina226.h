#ifndef INA226_H
#define INA226_H

#include "test.h"

#define INA226_ADDR         0x40

// 寄存器地址
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT    0x01
#define INA226_REG_BUS      0x02
#define INA226_REG_POWER    0x03
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CAL      0x05

// 配置值：连续模式，64次平均，转换时间8.2ms
#define INA226_CONFIG_VALUE 0x4527

// 校准值：适用于 10mΩ 采样电阻，电流 LSB = 0.1mA
#define INA226_CAL_VALUE    5120

// 初始化（不再需要传入参数）
void ina226_init(void);

// 所有读数均以整数形式返回，单位分别是 mV, mA, mW
uint16_t ina226_get_bus_voltage_mV(void);
int16_t  ina226_get_current_mA(void);
uint32_t ina226_get_power_mW(void);

// 一次性读取所有值（通过指针返回，单位同上）
void ina226_read_all(uint16_t *voltage_mV, int16_t *current_mA, uint32_t *power_mW);
void ina226_i2c_init(void) ;

#endif
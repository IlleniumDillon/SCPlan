#include "driver_icm20948.h"

#include "gpio.h"
#include "spi.h"
#include "i2c.h"

#include <stdint.h>

#include "cmsis_os.h"

/** @brief sleep in ms. */
void inv_icm20948_sleep(int ms)
{
	if (osKernelGetState() != osKernelRunning)
		HAL_Delay(ms);
	else
		osDelay(ms);
}
/** @brief sleep in us. in FreeRTOS cannot get us delay, use 1ms delay.*/
void inv_icm20948_sleep_us(int us)
{
	if (osKernelGetState() != osKernelRunning)
		HAL_Delay(us / 1000 + 1);
	else
		osDelay(us / 1000 + 1);
}
/** @brief time stamp */
uint64_t inv_icm20948_get_time_us(void)
{
	return 0;
}
/** @brief */
uint64_t inv_icm20948_get_dataready_interrupt_time_us(void)
{
	return 0;
}
/** @brief spi interface: read */
int inv_icm20948_spi_read(void * context, uint8_t reg, uint8_t * buf, uint32_t len)
{
    (void) context;
    (void) reg;
    (void) buf;
    (void) len;
	return 1;
}
/** @brief spi interface: write */
int inv_icm20948_spi_write(void * context, uint8_t reg, const uint8_t * buf, uint32_t len)
{
    (void) context;
    (void) reg;
    (void) buf;
    (void) len;
	return 1;
}
/** @brief iic interface: read */
int inv_icm20948_iic_read(void * context, uint8_t reg, uint8_t * buf, uint32_t len)
{
    (void) context;
    return HAL_I2C_Mem_Read(&hi2c3, ((ICM_I2C_ADDR_REVA << 1) + 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xff);
    // HAL_I2C_Master_Transmit(&hi2c3, ICM_I2C_ADDR_REVA, &reg, 1, 0xff);
    // HAL_I2C_Master_Receive(&hi2c3, ICM_I2C_ADDR_REVA, buf, len, 0xff);
    // return 0;
}
/** @brief iic interface: write */
int inv_icm20948_iic_write(void * context, uint8_t reg, const uint8_t * buf, uint32_t len)
{
    (void) context;
    return HAL_I2C_Mem_Write(&hi2c3, (ICM_I2C_ADDR_REVA << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *) buf, len, 0xff);
    // HAL_I2C_Master_Transmit(&hi2c3, ICM_I2C_ADDR_REVA, &reg, 1, 0xff);
    // HAL_I2C_Master_Transmit(&hi2c3, ICM_I2C_ADDR_REVA, (uint8_t *) buf, len, 0xff);
    // return 0;
}

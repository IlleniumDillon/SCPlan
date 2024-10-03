#ifndef DRIVER_ICM20948_H_
#define DRIVER_ICM20948_H_

#include "Invn/Devices/SensorTypes.h"
#include "Invn/Devices/SensorConfig.h"

#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */
#define ICM_I2C_ADDR_REVA			0x68 	/* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB			0x69 	/* I2C slave address for INV device on Rev B board */

#define DEF_ST_ACCEL_FS                 2
#define DEF_ST_GYRO_FS_DPS              250
#define DEF_ST_SCALE                    32768
#define DEF_SELFTEST_GYRO_SENS			(DEF_ST_SCALE / DEF_ST_GYRO_FS_DPS)
#define DEF_ST_ACCEL_FS_MG				2000

#ifndef INV20948_ABS
#define INV20948_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

typedef void (*sensor_data_ready_cb)(void * context, uint8_t sensortype, uint64_t timestamp, const void * data, const void *arg);

extern struct inv_icm20948 icm_device;

void icm20948_driver_init(void);

void icm20948_run_selftest(void);

#endif /* DRIVER_ICM20948_H_ */

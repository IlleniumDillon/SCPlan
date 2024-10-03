#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "cmsis_os.h"

#include "motion_fx.h"

#include <stdint.h>

#include "driver_icm20948.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948MPUFifoControl.h"
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"

static const uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

struct inv_icm20948 icm_device;

static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */
static int unscaled_bias[THREE_AXES * 2];

int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static const float cfg_mounting_matrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

extern int inv_icm20948_spi_read(void * context, uint8_t reg, uint8_t * buf, uint32_t len);
extern int inv_icm20948_spi_write(void * context, uint8_t reg, const uint8_t * buf, uint32_t len);
extern int inv_icm20948_iic_read(void * context, uint8_t reg, uint8_t * buf, uint32_t len);
extern int inv_icm20948_iic_write(void * context, uint8_t reg, const uint8_t * buf, uint32_t len);

void icm20948_apply_mounting_matrix(void)
{
    int ii;

	for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) 
    {
		inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, ii);
	}
}

void icm20948_set_fsr(void)
{
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

void icm20948_interface_init(void)
{
    icm_device.serif.is_spi = false;
    icm_device.serif.read_reg = inv_icm20948_iic_read;
    icm_device.serif.write_reg = inv_icm20948_iic_write;
    icm_device.serif.max_read = 1024;
	icm_device.serif.max_write = 1024;
}

void icm20948_driver_init(void)
{
    int rc;
    uint8_t whoami;

    icm20948_interface_init();

    while (1) 
    {
        rc = inv_icm20948_get_whoami(&icm_device, &whoami);
        if (rc == 0 && whoami == EXPECTED_WHOAMI[0]) {
            break;
        }
        osDelay(100);
    }

    inv_icm20948_init_matrix(&icm_device);

    rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));

    if (rc != 0)
    {
        while (1)
        {
            osDelay(100);
        }
    }

    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
    rc = inv_icm20948_initialize_auxiliary(&icm_device);
    if (rc == -1) 
    {
        while (1) 
        {
            osDelay(100);
        }
    }

    icm20948_apply_mounting_matrix();

    icm20948_set_fsr();

    inv_icm20948_init_structure(&icm_device);

//    inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, 1);
//    inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, 1);
    inv_icm20948_enable_sensor(&icm_device, INV_ICM20948_SENSOR_ROTATION_VECTOR, 1);
}

void inv_icm20948_get_st_bias(struct inv_icm20948 * s, int *gyro_bias, int *accel_bias, int * st_bias, int * unscaled)
{
    (void)s;
	int axis, axis_sign;
	int gravity, gravity_scaled;
	int i, t;
	int check;
	int scale;

	/* check bias there ? */
	check = 0;
	for (i = 0; i < 3; i++) {
		if (gyro_bias[i] != 0)
			check = 1;
		if (accel_bias[i] != 0)
			check = 1;
	}

	/* if no bias, return all 0 */
	if (check == 0) {
		for (i = 0; i < 12; i++)
			st_bias[i] = 0;
		return;
	}

	/* dps scaled by 2^16 */
	scale = 65536 / DEF_SELFTEST_GYRO_SENS;

	/* Gyro normal mode */
	t = 0;
	for (i = 0; i < 3; i++) {
		st_bias[i + t] = gyro_bias[i] * scale;
		unscaled[i + t] = gyro_bias[i];
	}
	axis = 0;
	axis_sign = 1;
	if (INV20948_ABS(accel_bias[1]) > INV20948_ABS(accel_bias[0]))
		axis = 1;
	if (INV20948_ABS(accel_bias[2]) > INV20948_ABS(accel_bias[axis]))
		axis = 2;
	if (accel_bias[axis] < 0)
		axis_sign = -1;

	/* gee scaled by 2^16 */
	scale = 65536 / (DEF_ST_SCALE / (DEF_ST_ACCEL_FS_MG / 1000));

	gravity = 32768 / (DEF_ST_ACCEL_FS_MG / 1000) * axis_sign;
	gravity_scaled = gravity * scale;

	/* Accel normal mode */
	t += 3;
	for (i = 0; i < 3; i++) {
		st_bias[i + t] = accel_bias[i] * scale;
		unscaled[i + t] = accel_bias[i];
		if (axis == i) {
			st_bias[i + t] -= gravity_scaled;
			unscaled[i + t] -= gravity;
		}
	}
}

void icm20948_run_selftest(void)
{
    int rc;
    int gyro_bias_regular[THREE_AXES];
	int accel_bias_regular[THREE_AXES];
	static int raw_bias[THREE_AXES * 2];

    rc = inv_icm20948_run_selftest(&icm_device, gyro_bias_regular, accel_bias_regular);
    if ((rc & INV_ICM20948_SELF_TEST_OK) == INV_ICM20948_SELF_TEST_OK) 
    {
        /* On A+G+M self-test success, offset will be kept until reset */
        icm_device.selftest_done = 1;
        icm_device.offset_done = 0;
        rc = 0;
    }
    else
    {
        while (1) 
        {
            osDelay(100);
        }
    }
    icm20948_driver_init();
	inv_icm20948_get_st_bias(&icm_device, gyro_bias_regular, accel_bias_regular, raw_bias, unscaled_bias);
    inv_icm20948_set_offset(&icm_device, unscaled_bias);
	icm_device.offset_done = 1;
}


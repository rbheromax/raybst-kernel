#ifndef __YAS_PCB_TEST_H__
#define __YAS_PCB_TEST_H__

#include "yas_types.h"

/* extra */
#define	YAS_PCBTEST_EXTRA

/* error code */
#define YAS_PCB_NO_ERROR		(0)
#define YAS_PCB_ERROR_I2C		(-1)
#define YAS_PCB_ERROR_POWER		(-2)
#define YAS_PCB_ERROR_TEST_ORDER	(-3)
#define YAS_PCB_ERROR_INTERRUPT		(-4)
#define YAS_PCB_ERROR_BUSY		(-5)
#define YAS_PCB_ERROR_OVERFLOW		(-6)
#define YAS_PCB_ERROR_UNDERFLOW		(-7)
#define YAS_PCB_ERROR_DIRCALC		(-8)
#define YAS_PCB_ERROR_NOT_SUPPORTED	(-9)
#define YAS_PCB_ERROR_CALREG		(-10)
#define YAS_PCB_ERROR_ARG		(-128)

/* addr */
#define YAS_PCB_ADDR_SLAVE		(0x2E)

#define YAS_PCB_ADDR_ID			(0x80)
#define YAS_PCB_ADDR_COIL		(0x81)
#define YAS_PCB_ADDR_MEASURE_COMMAND	(0x82)
#define YAS_PCB_ADDR_CONFIG		(0x83)
#define YAS_PCB_ADDR_MEASURE_INTERVAL	(0x84)
#define YAS_PCB_ADDR_OFFSET		(0x85)
#define YAS_PCB_ADDR_TEST1		(0x88)
#define YAS_PCB_ADDR_TEST2		(0x89)
#define YAS_PCB_ADDR_CAL		(0x90)
#define YAS_PCB_ADDR_MEASURE_DATA	(0xB0)

/* V Core */
#define YAS_VCORE			(18)

/* filter */
#define YAS_PCB_FILTER_ENABLE

#ifdef YAS_PCB_FILTER_ENABLE
#define YAS_PCB_MAG_MAX_FILTER_LEN		(30)
#define YAS_PCB_MAG_DEFAULT_FILTER_NOISE_X	(144) /* sd: 1200 nT */
#define YAS_PCB_MAG_DEFAULT_FILTER_NOISE_Y	(144) /* sd: 1200 nT */
#define YAS_PCB_MAG_DEFAULT_FILTER_NOISE_Z	(144) /* sd: 1200 nT */
#define YAS_PCB_MAG_DEFAULT_FILTER_LEN	(20)
#endif

#define YAS_PCB_NOISE_LEVEL_MEASURE_NUM	(200)
#define YAS_PCB_NOISE_LEVEL_MEASURE_DELAY	(50)
#define YAS_PCB_NOISE_LEVEL_DUMMY_NUM	YAS_PCB_MAG_DEFAULT_FILTER_LEN

struct yas_pcb_test_callback {
	int	(*power_on)(void);
	int	(*power_off)(void);
	int	(*i2c_open)(void);
	int	(*i2c_close)(void);
	int	(*i2c_write)(uint8_t, uint8_t, const uint8_t *, int);
	int	(*i2c_read)(uint8_t, uint8_t, uint8_t *, int);
	void	(*msleep)(int);
	int	(*read_intpin)(int *);
};

struct yas_pcb_test {
	int	(*power_on_and_device_check)(int *);
	int	(*initialization)(void);
	int	(*offset_control_measurement_and_set_offset_register)
			(int *, int *, int *);
	int	(*direction_measurement)(int *);
	int	(*sensitivity_measurement_of_magnetic_sensor_by_test_coil)
			(int *, int *);
	int	(*magnetic_field_level_check)(int *, int *, int *);
	int	(*noise_level_check)(int *, int *, int *);
	int	(*power_off)(void);
	struct yas_pcb_test_callback callback;
};

/* prototype functions */
#ifdef __cplusplus
extern "C" {
#endif

int yas_pcb_test_init(struct yas_pcb_test *);

#ifdef __cplusplus
}
#endif

#endif	/* ! __YAS_PCB_TEST_H__ */

/* end of file */

#ifndef __TAOS_H__
#define __TAOS_H__


/* i2c */
#define I2C_M_WR 0 /* for i2c */
#define I2c_M_RD 1 /* for i2c */

/* sensor type */
#define TAOS_LIGHT		0
#define TAOS_PROXIMITY	1
#define TAOS_ALL		2

/* power control */
#define ON	1
#define OFF	0

#define	MAX_LUX				32768
/* for proximity adc avg */
#define PROX_READ_NUM 40
#define TAOS_PROX_MAX 1023
#define TAOS_PROX_MIN 0

enum TAOS_ALS_FOPS_STATUS {
	TAOS_ALS_CLOSED = 0,
	TAOS_ALS_OPENED = 1,
};

enum TAOS_PRX_FOPS_STATUS {
	TAOS_PRX_CLOSED = 0,
	TAOS_PRX_OPENED = 1,
};

enum TAOS_CHIP_WORKING_STATUS {
	TAOS_CHIP_UNKNOWN = 0,
	TAOS_CHIP_WORKING = 1,
	TAOS_CHIP_SLEEP = 2
};

/* driver data */
struct taos_data {
	struct input_dev *prox_input_dev;
	struct input_dev *light_input_dev;
	struct i2c_client *client;
	struct work_struct work_prox;  /* for proximity sensor */
	struct work_struct work_light; /* for light_sensor     */
	struct work_struct work_ptime; /* for proximity reset    */
	struct class *lightsensor_class;
	struct class *proximity_class;
	struct hrtimer timer;
	struct hrtimer ptimer;
	struct workqueue_struct *taos_wq;
	struct workqueue_struct *taos_test_wq;
	struct wake_lock prx_wake_lock;
	struct taos_platform_data *pdata;
	struct mutex power_lock;
	struct mutex prox_mutex;
	struct i2c_client *opt_i2c_client;

	enum TAOS_ALS_FOPS_STATUS taos_als_status;
	enum TAOS_PRX_FOPS_STATUS taos_prx_status;
	enum TAOS_CHIP_WORKING_STATUS taos_chip_status;

	ktime_t light_polling_time;
	ktime_t prox_polling_time;
	bool light_enable;
	bool proximity_enable;
	short proximity_value;
	u16 chipID;
	int irdata;		/* Ch[1] */
	int cleardata;	/* Ch[0] */
	int	irq;
	int avg[3];
	/* Auto Calibration */
	u8 offset_value;
	u8 initial_offset;
	int cal_result;
	int threshold_high;
	int threshold_low;
};

/* platform data */
struct taos_platform_data {
	int als_int;
	int (*power_en)(int);
	int (*power_led)(int);
	unsigned int wakeup;
	int prx_thrsh_hi_param;
	int prx_thrsh_lo_param;
	int prx_thrsh_hi_calparam;
	int prx_thrsh_lo_calparam;
	int als_time_param;
	int intr_filter_param;
	int prx_pulse_cnt_param;
	int ga;
	int coef_a;
	int coef_b;
	int coef_c;
	int coef_d;
};

/* prototype */
int taos_get_lux(struct taos_data *taos);
void taos_on(struct taos_data *taos, int type);
void taos_off(struct taos_data *taos, int type);

#endif

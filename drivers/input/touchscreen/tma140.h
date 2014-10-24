#ifndef _TMA140_H_
#define _TMA140_H_

extern struct class *sec_class;
extern int tsp_charger_type_status;
extern int cypress_update(int);
extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int set_irq_type(unsigned int irq, unsigned int type);

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);
void set_tsp_for_ta_detect(int);
void touch_ctrl_regulator(int on_off);
EXPORT_SYMBOL(touch_ctrl_regulator);

#endif /*_TMA140_H_*/

#ifndef LIBTOUCH_H
#define LIBTOUCH_H
#include "spi_arbitrer.h"

#define GPIO_PA0 0
#define GPIO_PC0 0
#define GPIO_PB12 12
#define GPIO_PD12 12
#define GPIO_PD10 10
#define GPIO_PD11 11
#define SYSCFG_EXTICR1 ((uint32_t*)(SYSCFG_BASE+0x8))
#define SYSCFG_EXTICR4 ((uint32_t*)(SYSCFG_BASE+0x14))
#define SYSCFG_EXTICR3 ((uint32_t*)(SYSCFG_BASE+0x10))
#define A0_BIT (1<<4)
#define A1_BIT (1<<5)
#define A2_BIT (1<<6)
#define S_BIT  (1<<7)
#define MODE_BIT (1<<3)
#define SER_BIT (1<<2)
#define PD1_BIT (1<<1)
#define PD0_BIT (1)

#define TOUCH_NSS_BIT (ad7843_dev_infos.gpios[TOUCH_NSS].pin)
#define UP_TOUCH_NSS {sys_cfg(CFG_GPIO_SET,(uint8_t)((ad7843_dev_infos.gpios[TOUCH_NSS].port<<4) + TOUCH_NSS_BIT),1);}
#define DOWN_TOUCH_NSS {sys_cfg(CFG_GPIO_SET,(uint8_t)((ad7843_dev_infos.gpios[TOUCH_NSS].port<<4) + TOUCH_NSS_BIT),0);}

int         touch_read_12bits(uint8_t command);
int         touch_read_X_SER();
int         touch_read_Y_SER();
int         touch_read_X_DFR();
int         touch_read_Y_DFR();
int         touch_pin_init();
void        touch_init_penirq();
int         touch_getx();
int         touch_gety();
int         touch_is_touched();
void        touch_refresh_pos();
uint8_t     touch_early_init();
uint8_t     touch_init();
void        touch_enable_exti();
void        touch_reactivate_PENIRQ();
#endif

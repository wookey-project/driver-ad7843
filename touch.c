#include "autoconf.h"
#include "libc/types.h"
#include "libtft.h"
#include "libspi.h"
#include "api/libtouch.h"
#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "libc/regutils.h"
#include "libc/syscall.h"
#include "generated/ad7843.h"

/*
 * This lib requires:
 * BUS   permission
 * DMA   permission
 * TIMER permission
 */
static const char name[6] = "touch";

/* Variables */
static int  posx, posy;
//static int  timer_running = -1;
//static uint8_t EXTI_enable = 1;
int         devdesc_touch;
//extern int  devdesc_timer2;

/***********************************************
 * SDIO block startup function
 ***********************************************/

static void power_up(void)
{
}

/*****************************************
 * IRQ Handlers
 *
 * prototype for irq handlers is:
 * static void my_irq_handler(uint8_t irq, // IRQ number
 *                           uint32_t sr   // content of posthook.status,
 *                           uint32_t dr   // content of posthook.data)
 *
 *****************************************/

/*****************************************
 * Initialization functions
 *****************************************/
void        EXTI15_10_IRQHandler(uint8_t irq __attribute__ ((unused)),
                                 uint32_t status __attribute__ ((unused)),
                                 uint32_t data __attribute__ ((unused)));
static volatile uint8_t is_touched=0;

static device_t    dev;

uint8_t touch_early_init(void)
{
    uint8_t     ret = 0;
    memset((void*)&dev, 0, sizeof(device_t));
    is_touched=0;
    //timer2_early_init();        /* Need that to be done before going futher */

    /*******************************
     * first, SDIO device declaration
     *******************************/
    /*
     * declare the SDIO device, during initialization phase
     * This function create a device_t, fullfill it, and execute a
     * sys_init(INIT_DEVACCESS) syscall.
     */
    memcpy(dev.name, name, strlen(name));

    dev.gpio_num = 3;

    /* DECLARE Touch NSS GPIO_PD10 */
    dev.gpios[2].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE |
        GPIO_MASK_SET_SPEED;
    dev.gpios[2].kref.port = ad7843_dev_infos.gpios[TOUCH_BUSY].port;
    dev.gpios[2].kref.pin =  ad7843_dev_infos.gpios[TOUCH_BUSY].pin;
    dev.gpios[2].pupd = GPIO_NOPULL;
    dev.gpios[2].mode = GPIO_PIN_INPUT_MODE;
    dev.gpios[2].speed = GPIO_PIN_VERY_HIGH_SPEED;

    /* DECLARE Touch NSS GPIO_PD10 */
    dev.gpios[0].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE |
        GPIO_MASK_SET_SPEED;
    dev.gpios[0].kref.port = ad7843_dev_infos.gpios[TOUCH_NSS].port;
    dev.gpios[0].kref.pin = ad7843_dev_infos.gpios[TOUCH_NSS].pin;
    dev.gpios[0].mode = GPIO_PIN_OUTPUT_MODE;
    dev.gpios[0].pupd = GPIO_PULLUP;
    dev.gpios[0].type = GPIO_PIN_OTYPER_PP;
    dev.gpios[0].speed = GPIO_PIN_VERY_HIGH_SPEED;

    //Then configure pin PD12 (the EXTI pin)
    dev.gpios[1].mask = GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED |
	    //GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_EXTI;
	    GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_EXTI;
    dev.gpios[1].type = GPIO_PIN_OTYPER_OD;
    dev.gpios[1].speed = GPIO_PIN_HIGH_SPEED;
    dev.gpios[1].kref.port = ad7843_dev_infos.gpios[TOUCH_EXTI].port;
    dev.gpios[1].kref.pin = ad7843_dev_infos.gpios[TOUCH_EXTI].pin;
    dev.gpios[1].mode = GPIO_PIN_INPUT_MODE;
    dev.gpios[1].pupd = GPIO_NOPULL;
    dev.gpios[1].exti_trigger = GPIO_EXTI_TRIGGER_BOTH;
    dev.gpios[1].exti_handler = EXTI15_10_IRQHandler;
    dev.gpios[1].exti_lock = GPIO_EXTI_LOCKED;

    ret = sys_init(INIT_DEVACCESS, &dev, &devdesc_touch);

    return ret;
}

/*
*for penirq being serviced correctly we need a first correct SP%I communication
* So we do it after SPI is init and SPI is init after TOUCH_NSS is UP!
*/
uint8_t touch_init(void)
{
    /*FIXME: Check that SPI is clocked */

    lock_bus(2);
    DOWN_TOUCH_NSS;
#if CONFIG_WOOKEY_V1 || CONFIG_WOOKEY_EMMC
    spi1_master_send_byte_sync(S_BIT | A2_BIT | A0_BIT);  //VREF_ON
    spi1_master_send_byte_sync( A1_BIT);   //Scratch
    spi1_master_send_byte_sync( A2_BIT | A1_BIT);  //Scratch
    spi1_master_send_byte_sync( A2_BIT | A0_BIT);
#elif defined(CONFIG_WOOKEY_V2) || defined(CONFIG_WOOKEY_V3)
    spi2_master_send_byte_sync(S_BIT | A2_BIT | A0_BIT);  //VREF_ON
    spi2_master_send_byte_sync( A1_BIT);   //Scratch
    spi2_master_send_byte_sync( A2_BIT | A1_BIT);  //Scratch
    spi2_master_send_byte_sync( A2_BIT | A0_BIT);
#else
#error "unsupported board"
#endif
    UP_TOUCH_NSS;
    unlock_bus();
    return 0;
}

/* Miscelleanous function for send commands and receiveing datas */

//Nous on est SPI 4wire
//J2 J3 J4 J5 short et les autres open

//static
int touch_read_12bits(uint8_t command)
{
    volatile int res;
    volatile uint8_t     tmpres;
    uint8_t     br = 3;


#if CONFIG_WOOKEY_V1 || CONFIG_WOOKEY_EMMC
    spi1_disable();
    br = spi1_get_baudrate();
    spi1_set_baudrate(SPI_BAUDRATE_750KHZ);
    spi1_enable();
    /*DOWN the touch CS line */
    DOWN_TOUCH_NSS;
    /* send the command */
    res = spi1_master_send_byte_sync( S_BIT | command);    //S_BIT for control
    tmpres = spi1_master_send_byte_sync( 0);   //dont care
    res = ((tmpres & 0x7f) << 5);
    tmpres = spi1_master_send_byte_sync( 0);   //dont care
    res |= (tmpres >> 3);
    UP_TOUCH_NSS;
    spi1_disable();
    spi1_set_baudrate(br);
    spi1_enable();
#elif defined(CONFIG_WOOKEY_V2) || defined(CONFIG_WOOKEY_V3)
    spi2_disable();
    br = spi2_get_baudrate();
    spi2_set_baudrate(SPI_BAUDRATE_750KHZ);
    spi2_enable();
    /*DOWN the touch CS line */
    DOWN_TOUCH_NSS;
    /* send the command */
    res = spi2_master_send_byte_sync( S_BIT | command);    //S_BIT for control
   /* Wait Busy Line to got down*/
    tmpres = spi2_master_send_byte_sync( 0);   //dont care
    res = ((tmpres & 0x7f) << 5);
    tmpres = spi2_master_send_byte_sync( 0);   //dont care
    res |= (tmpres >> 3);
    UP_TOUCH_NSS;
    spi2_disable();
    spi2_set_baudrate(br);
    spi2_enable();
#endif
    return res;
}

int touch_read_X_SER()
{
    int         tmp = 0;
    int         i;
    for (i = 0; (i < 64) && (is_touched); i++) {
	tmp+=touch_read_12bits(A0_BIT|SER_BIT|PD1_BIT);
    }
    return tmp / i;
}

int touch_read_Y_SER()
{
    int         tmp = 0;
    int         i;
    for (i = 0; (i < 64) && (is_touched) ; i++) {
        tmp += touch_read_12bits(A2_BIT | SER_BIT | PD1_BIT);
    }
    return tmp / i;
}

int touch_read_X_DFR()
{
    int         tmp = 0;
    int         i;
    for (i = 0; (i < 16) && (is_touched); i++) {
      //printf("read X istouched %d\n",is_touched);
        //tmp+=touch_read_12bits(S_BIT|A0_BIT|PD0_BIT|PD1_BIT);
        //tmp+=touch_read_12bits(S_BIT|A0_BIT|PD1_BIT);
        tmp += touch_read_12bits(S_BIT | A0_BIT | PD1_BIT);
    }
    return tmp / i - 200;
}

int touch_read_Y_DFR()
{
    int         tmp = 0;
    int         i;
    for (i = 0; (i < 16) && (is_touched) ; i++) {
        //tmp+=touch_read_12bits(S_BIT|A2_BIT|A0_BIT|PD0_BIT|PD1_BIT);
        //tmp+=touch_read_12bits(S_BIT|A2_BIT|A0_BIT|PD1_BIT);
        tmp += touch_read_12bits(S_BIT | A2_BIT | A0_BIT | PD1_BIT);
    }
    return tmp / i - 200;
}

void touch_reactivate_PENIRQ()
{
    touch_read_12bits(S_BIT | A2_BIT | A0_BIT | PD1_BIT);
}

void EXTI15_10_IRQHandler(uint8_t irq __attribute__ ((unused)),
                          uint32_t status __attribute__ ((unused)),
                          uint32_t data __attribute__ ((unused)) )
{
  uint8_t tmp;
    sys_cfg(CFG_GPIO_GET, (uint8_t) (
              (ad7843_dev_infos.gpios[TOUCH_EXTI].port<<4)+
                ad7843_dev_infos.gpios[TOUCH_EXTI].pin), &tmp);
   is_touched=!tmp;
   return;
}


/*
 * WARNING!!!!!! INVERTED X AND Y  SIZE HERE!
 */
int touch_getx()
{
    return (posx < 3400) ? 320 * (3400 - posx) / 3400 : 0;
}

int touch_gety()
{
    return (posy < 3400) ? 240 * (3400 - posy) / 3400 : 0;
}

void touch_enable_exti()
{
    sys_cfg(CFG_GPIO_UNLOCK_EXTI, (uint8_t)dev.gpios[1].kref.val);
}

int touch_is_touched()
{
    uint8_t tmp=1;
    sys_cfg(CFG_GPIO_GET, (uint8_t) (
              (ad7843_dev_infos.gpios[TOUCH_EXTI].port<<4)+
                ad7843_dev_infos.gpios[TOUCH_EXTI].pin), &tmp);
    return !tmp;
    //return is_touched; //timer_running;
}

void touch_refresh_pos()
{
#if TOUCH_DEBUG
    printf("Positions lue DFR %x %x \n",touch_read_X_DFR(),touch_read_Y_DFR());
#endif
  /*printf("Positions lue DFR %x %x SER %x %x\n",touch_read_X_DFR(),touch_read_Y_DFR(),
                                             touch_read_X_SER(),touch_read_Y_SER());
   */
    posx = touch_read_X_DFR();
    posy = touch_read_Y_DFR();
}

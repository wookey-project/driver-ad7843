#include "api/types.h"
#include "kernel/exported/devices.h"
#include "kernel/exported/dmas.h"
#include "libtft.h"
#include "libspi.h"
#include "api/libtouch.h"
#include "api/print.h"
#include "api/regutils.h"
#include "api/syscall.h"

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

uint8_t touch_early_init(void)
{
    uint8_t     ret = 0;

    //timer2_early_init();        /* Need that to be done before going futher */

    /*******************************
     * first, SDIO device declaration
     *******************************/
    device_t    dev = { 0 };
    /*
     * declare the SDIO device, during initialization phase
     * This function create a device_t, fullfill it, and execute a
     * sys_init(INIT_DEVACCESS) syscall.
     */
    memcpy(dev.name, name, strlen(name));

    dev.gpio_num = 2;

    /* DECLARE Touch NSS GPIO_PD10 */
    dev.gpios[0].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE |
        GPIO_MASK_SET_SPEED;
    dev.gpios[0].kref.port = GPIO_PD;
    dev.gpios[0].kref.pin = 10;
    dev.gpios[0].mode = GPIO_PIN_OUTPUT_MODE;
    dev.gpios[0].pupd = GPIO_PULLUP;
    dev.gpios[0].type = GPIO_PIN_OTYPER_PP;
    dev.gpios[0].speed = GPIO_PIN_VERY_HIGH_SPEED;

    //Then configure pin PD12 (the EXTI pin)
    dev.gpios[1].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_EXTI;
    dev.gpios[1].kref.port = GPIO_PD;
    dev.gpios[1].kref.pin = 12;
    dev.gpios[1].mode = GPIO_PIN_INPUT_MODE;
    dev.gpios[1].pupd = GPIO_NOPULL;
    dev.gpios[1].exti_trigger = GPIO_EXTI_TRIGGER_BOTH;
    dev.gpios[1].exti_handler = EXTI15_10_IRQHandler;

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
    spi_master_send_byte_sync(1, S_BIT | A2_BIT | A0_BIT);  //VREF_ON
    spi_master_send_byte_sync(1, A1_BIT);   //Scratch
    spi_master_send_byte_sync(1, A2_BIT | A1_BIT);  //Scratch
    spi_master_send_byte_sync(1, A2_BIT | A0_BIT);
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
    uint8_t     tmpres;
    uint32_t    stock;
    spi_disable(1);
    stock = read_reg_value(r_CORTEX_M_SPI1_CR1);
    write_reg_value(r_CORTEX_M_SPI1_CR1, (stock & ~(7 << 3)) | (6 << 3));
    spi_enable(1);
    /*DOWN the touch CS line */
    DOWN_TOUCH_NSS;
    /* send the command */
    res = spi_master_send_byte_sync(1, S_BIT | command);    //S_BIT for control
    tmpres = spi_master_send_byte_sync(1, 0);   //dont care
    res = ((tmpres & 0x7f) << 5);
    tmpres = spi_master_send_byte_sync(1, 0);   //dont care
    res |= (tmpres >> 3);
    UP_TOUCH_NSS;
    spi_disable(1);
    write_reg_value(r_CORTEX_M_SPI1_CR1, stock);
    spi_enable(1);
    return res;
}

int touch_read_X_SER()
{
    int         tmp = 0;
    int         i;
    for (i = 0; i < 64; i++) {
        tmp += touch_read_12bits(A0_BIT | SER_BIT);
    }
    return tmp / 64;
}

int touch_read_Y_SER()
{
    int         tmp = 0;
    int         i;
    for (i = 0; i < 64; i++) {
        tmp += touch_read_12bits(A2_BIT | SER_BIT);
    }
    return tmp / 64;
}

int touch_read_X_DFR()
{
    int         tmp = 0;
    int         i;
    for (i = 0; i < 64; i++) {
        //tmp+=touch_read_12bits(S_BIT|A0_BIT|PD0_BIT|PD1_BIT);
        //tmp+=touch_read_12bits(S_BIT|A0_BIT|PD1_BIT);
        tmp += touch_read_12bits(S_BIT | A0_BIT | PD1_BIT);
    }
    return tmp / 64 - 200;
}

int touch_read_Y_DFR()
{
    int         tmp = 0;
    int         i;
    for (i = 0; i < 64; i++) {
        //tmp+=touch_read_12bits(S_BIT|A2_BIT|A0_BIT|PD0_BIT|PD1_BIT);
        //tmp+=touch_read_12bits(S_BIT|A2_BIT|A0_BIT|PD1_BIT);
        tmp += touch_read_12bits(S_BIT | A2_BIT | A0_BIT | PD1_BIT);
    }
    return tmp / 64 - 200;
}

void EXTI15_10_IRQHandler(uint8_t irq __attribute__ ((unused)),
                          uint32_t status __attribute__ ((unused)),
                          uint32_t data __attribute__ ((unused)) )
{
return ;
#if 0
    if (!EXTI_enable)
        return;
    //set_reg_bits(r_CORTEX_M_NVIC_ICER1,0x80);//Disable interrupt during reading
    //read_reg_value(r_CORTEX_M_NVIC_ICPR1);
    /*
     * BEGIN EWOK EXTI is handled by Ewok and we should only reach here if
     * it is our interrupt
     *if(read_reg_value(EXTI_PR)&(1<<12))
     *
     {
     */
    // set_reg_bits(EXTI_PR,(1<<12));
    if (timer_running) {
        clear_wait(2);          //if timer was waiting for the bus
        // clear its flags
        timer2_disable();
        timer_running = 0;
    } else {
        timer2_enable();
        timer_running = 1;
    }

    /*  } END EWOK */
//  set_reg_bits(r_CORTEX_M_NVIC_ICPR1,0x80);//
//  set_reg_bits(EXTI_PR,(1<<12));
#endif
}

#if 0
/* the TIM2_IRQHandler! */
void TIM2_IRQHandler(uint8_t irq __attribute__((unused)) , // IRQ number
                     uint32_t sr __attribute__((unused)) ,  // content of posthook.status,
                     uint32_t dr __attribute__((unused)) )  // content of posthook.data)
{
    int         tmpx, tmpy;

    //Reset counter and Clear the pending flags
    timer2_disable();
    write_reg_value(r_CORTEX_M_TIM2CNT, read_reg_value(r_CORTEX_M_TIM2ARR));
    write_reg_value(r_CORTEX_M_TIM2SR, 0);

    //First Mask the EXTI12 interrupt as read posX and posy may
    //make some noise on the interrupt line
    //clear_reg_bits(EXTI_IMR,(1<<12));
    EXTI_enable = 0;

    /* Wait for any already launched SPI transfert to complete */
    if (try_lock_bus(2))        //Can we get the bus
        //otherwise just put a flag in the wait queue and wait for reschedule
    {
        //make measurement
        tmpx = touch_read_X_DFR();
        tmpy = touch_read_Y_DFR();

        posx = tmpx;
        posy = tmpy;
        unlock_bus();
    }
    //reenable timer and unmask the interrupt
    timer2_enable();
    // set_reg_bits(EXTI_IMR,(1<<12));
    EXTI_enable = 1;

    //Check if the current level of the interrupt pin is up
    //meaning that the touch has been released while reading
    //the touch screen
    {
        uint8_t     is_released;
        sys_cfg(CFG_GPIO_GET, (uint8_t) ((('D' - 'A') << 4) + 12),
                &is_released);
        if (is_released
            // (read_reg_value(GPIOD_IDR)&(1<<12))
            ) {
            //set_reg_bits(EXTI_SWIER,(1<<12));
            timer2_disable();
            timer_running = 0;
        }
    }
}
#endif

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

#if 0
int touch_is_touched()
{
    return timer_running;
}
#endif

void touch_refresh_pos()
{
    lock_bus(2);
    posx = touch_read_X_DFR();
    posy = touch_read_Y_DFR();
    unlock_bus();
}

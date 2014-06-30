/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013 Chuck McManis <cmcmanis@mcmanis.com>
 * Copyright (C) 2013 Onno Kortmann <onno@gmx.net>
 * Copyright (C) 2013 Frantisek Burian <BuFran@seznam.cz> (merge)
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>

#include "stepper.h"
//#include "printf.h"


//static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
//static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);

#define SYSTICK_FREQ 1000
#define M1 GPIO8
#define M2 GPIO9
#define M3 GPIO10
#define M4 GPIO11
#define PERIOD 17000
#define PULSE 4000

#define E1_PORT GPIOA
#define E1 GPIO6
#define E2_PORT GPIOA
#define E2 GPIO7
#define E3_PORT GPIOB
#define E3 GPIO0
#define E4_PORT GPIOB
#define E4 GPIO1
#define LCD_SPI_NSS GPIO12

#define PRINTER_PORT GPIOA
#define PRINTER_LATCH GPIO0
#define PRINTER_STROBE1 GPIO1

volatile uint32_t sys_cnt;
StepperMotor engine(GPIOA, M1,M2,M3,M4);
void delay(volatile uint32_t);


/* Called when systick fires */
void sys_tick_handler(void)
{
//         gpio_toggle(GPIOA,GPIO1);
     sys_cnt++;
     if (sys_cnt++ / 100) {
         sys_cnt=0;
         //gpio_toggle(GPIOA,GPIO1);
//         gpio_set(GPIOB,LCD_SPI_NSS);
        // spi_send8(SPI1,0xf);
 //        gpio_clear(GPIOB,LCD_SPI_NSS);
         //engine.step(5);
     }

//    gpio_toggle(GPIOA, GPIO9);
}

/* Set up timer to fire freq times per second */
static void systick_setup(int freq)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    /* clear counter so it starts right away */
    STK_CVR = 0;

    systick_set_reload(rcc_core_frequency / freq);
    systick_counter_enable();
    systick_interrupt_enable();
}



static void usart_setup(void)
{
/* Setup USART2 parameters. */
usart_set_baudrate(USART1, 115200);
usart_set_databits(USART1, 8);
usart_set_parity(USART1, USART_PARITY_NONE);
usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
usart_set_mode(USART1, USART_MODE_TX);
usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

/* Finally enable the USART. */
usart_enable(USART1);
}

static void gpio_setup(void)
{
    /* Select pin functions. PC8/PC9 are the two LEDs on the
      STM32F0DISCOVERY board. */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, M1|M2|M3|M4);

    /* testy timer'a */
    //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    /* printer LATCH/STROBE */
    gpio_mode_setup(PRINTER_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLDOWN,PRINTER_LATCH);
    gpio_mode_setup(PRINTER_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLDOWN,PRINTER_STROBE1);
    //gpio_clear(PRINTER_PORT,PRINTER_STROBE1);
    gpio_clear(PRINTER_PORT,PRINTER_LATCH);

    /* set GPIOA to AF 0 */
    //gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	/* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2|GPIO3);
    /* wyjście PA6 AF1 (TIM3_CH1 */
    gpio_mode_setup(E1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, E1);
    gpio_mode_setup(E2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, E2);
    gpio_mode_setup(E3_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, E3);
    gpio_mode_setup(E4_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, E4);

    gpio_set_af(GPIOA, GPIO_AF1, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO3);
    /* AF TIM3_CH1, TIM3_CH1 */
    gpio_set_af(E1_PORT,GPIO_AF1,E1);
    gpio_set_af(E2_PORT,GPIO_AF1,E2);
    gpio_set_af(E3_PORT,GPIO_AF1,E3);
    gpio_set_af(E4_PORT,GPIO_AF1,E4);

    /* SPI1 setup */
    /* software NSS */
    //gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,LCD_SPI_NSS);
    gpio_mode_setup(GPIOB,GPIO_MODE_AF,GPIO_PUPD_NONE, LCD_SPI_NSS|GPIO13|GPIO14|GPIO15);
    gpio_set_af(GPIOB,GPIO_AF0,LCD_SPI_NSS|GPIO13|GPIO14|GPIO15);

}

static void timer_setup(void)
{
/* Enable TIM3 clock. */
rcc_periph_clock_enable(RCC_TIM3);
timer_reset(TIM3);
/* Timer global mode: - No divider, Alignment edge, Direction up */
timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
timer_continuous_mode(TIM3);
timer_set_period(TIM3, PERIOD);
timer_disable_preload(TIM3);

/* prescaler F_SYS/48 = TIM3 clock is 1 MHz */
timer_set_prescaler(TIM3,48);
//timer_disable_oc_output(TIM3, TIM_OC2 | TIM_OC3 | TIM_OC4);
timer_enable_oc_output(TIM3, TIM_OC1);
timer_disable_oc_output(TIM3,TIM_OC2);

timer_enable_oc_output(TIM3, TIM_OC3);
//timer_enable_oc_output(TIM3, TIM_OC4);
// motor ch1
timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
// motor ch2
timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
// motor ch3
timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
// motor ch3
timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);


/* disable preload */
timer_disable_oc_preload(TIM3, TIM_OC1);
timer_disable_oc_preload(TIM3, TIM_OC2);
timer_disable_oc_preload(TIM3, TIM_OC3);
timer_disable_oc_preload(TIM3, TIM_OC4);

/* polarity */
timer_set_oc_polarity_high(TIM3,TIM_OC1);
timer_set_oc_polarity_high(TIM3,TIM_OC2);
timer_set_oc_polarity_high(TIM3,TIM_OC3);
timer_set_oc_polarity_high(TIM3,TIM_OC4);
//timer_enable_oc_clear(TIM3, TIM_OC1);
//timer_set_oc_slow_mode(TIM3, TIM_OC1);
timer_set_oc_value(TIM3, TIM_OC1, PULSE);
timer_set_oc_value(TIM3, TIM_OC2, PULSE*3);
timer_set_oc_value(TIM3, TIM_OC3, PULSE*2);
timer_set_oc_value(TIM3, TIM_OC4, PULSE*4);
//timer_generate_event(TIM3,TIM_EGR_CC1G);
//timer_enable_update_event(TIM3);
nvic_enable_irq(NVIC_TIM3_IRQ);
timer_enable_irq(TIM3,TIM_DIER_CC1IE);
timer_enable_irq(TIM3,TIM_DIER_CC2IE);
timer_enable_irq(TIM3,TIM_DIER_CC3IE);
timer_enable_irq(TIM3,TIM_DIER_CC4IE);
//timer_enable_irq(TIM3,TIM_DIER_CC2IE);
timer_enable_irq(TIM3,TIM_DIER_UIE);
/* Set the timer trigger output (for the DAC) to the channel 1 output
compare */
//timer_set_master_mode(TIM3, TIM_CR2_MMS_COMPARE_OC1REF);
timer_enable_counter(TIM3);

}

/* set STM32 to clock by 48MHz from HSI oscillator */
static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();

    /* Enable clocks to the GPIO subsystems */
    //rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_SPI1);
}
static void spi_setup(void)
{
spi_set_master_mode(SPI1);
spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
spi_set_clock_polarity_1(SPI1);
spi_set_clock_phase_0(SPI1);
spi_set_bidirectional_transmit_only_mode(SPI1);
//spi_set_unidirectional_mode(SPI1); /* bidirectional but in 3-wire */
spi_set_data_size(SPI1, SPI_CR2_DS_16BIT);
//spi_enable_software_slave_management(SPI1);
spi_send_msb_first(SPI1);
spi_set_nss_low(SPI1);
//spi_enable_ss_output(SPI1);
//spi_fifo_reception_threshold_8bit(SPI1);
//SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;
spi_enable(SPI1);
}

static void mco_setup(void)
{
    /* Enable system clock output on pin PA8 (so it can be checked with a
      scope) */
    rcc_set_mco(RCC_CFGR_MCO_SYSCLK);
}
void delay(volatile uint32_t t) {
    t *= 37;
    while(t--);

}
void putc(unsigned char c)
{
    usart_send_blocking(USART1,c);
}
void puts(const unsigned char *s)
{
    while(*s) {
        usart_send_blocking(USART1,*s);
        s++;
    }
}
void print(const char *s)
{

    // loop through entire string
    while (*s) {
    	if ( *s == '\n') {
        usart_send_blocking(USART1,'\r');
        //usart_send_blocking(USART1,'\n');
    	}
    	usart_send_blocking(USART1,*s);
        s++;
    }
}
static inline void latch_h(void) { gpio_set(PRINTER_PORT,PRINTER_LATCH);};
static inline void latch_l(void) { gpio_clear(PRINTER_PORT,PRINTER_LATCH);};
static inline void strobe1(void) {
gpio_set(PRINTER_PORT,PRINTER_STROBE1);
delay(1);
gpio_clear(PRINTER_PORT,PRINTER_STROBE1);
}

void printer_send(uint32_t data)
{
    int i = 0;
    latch_l();
    for(i = 0; i<18; i++) {
    spi_send(SPI1,data);
    }
    delay(6);
    latch_h();
    delay(1);
    latch_l();
    delay(1);
    strobe1();


}
int main(void)
{

    uint16_t printer_data = 0xff;
    clock_setup();
    gpio_setup();
    mco_setup();
    usart_setup();
    spi_setup();
    //timer_setup();
    systick_setup(SYSTICK_FREQ);
    print("Hello from STM32F0\n");
   // StepperMotor engine(GPIOA, M1,M2,M3,M4);
    //gpio_port_write(GPIOA,0);
//    gpio_set(GPIOA,GPIO1);
    //engine.step(100);
    while (1) {
        printer_send(printer_data);
        engine.step(1);
        delay(200);
       //engine.step(1);
       /*
    for(i = 0; i < sizeof(steps)/sizeof(steps[0]);i++) {
    gpio_port_write(GPIOA,steps[i]);
    delay(3);
    }
    */
    };
}

void tim3_isr(void)
{
    if (timer_interrupt_source(TIM3,TIM_SR_UIF)) {
        timer_clear_flag(TIM3,TIM_SR_UIF);
        timer_set_oc_value(TIM3, TIM_OC1, PULSE);
        timer_disable_oc_output(TIM3,TIM_OC2);
        timer_disable_oc_output(TIM3,TIM_OC4);
        //timer_disable_irq(TIM3,TIM_OC2);
        //timer_disable_irq(TIM3,TIM_OC4);
        timer_enable_irq(TIM3,TIM_DIER_CC1IE);
        //gpio_set(GPIOA,GPIO1);
    }
    if ( timer_interrupt_source(TIM3,TIM_SR_CC1IF)) {
        timer_clear_flag(TIM3,TIM_SR_CC1IF);
        //if (PERIOD >= TIM3_CCR1) {
        //gpio_clear(GPIOA,GPIO1);
        timer_enable_oc_output(TIM3, TIM_OC2);
        //timer_enable_irq(TIM3,TIM_DIER_CC2IE);
       // }

    }
    if(timer_interrupt_source(TIM3,TIM_SR_CC2IF)) {
        timer_clear_flag(TIM3,TIM_SR_CC2IF);
        timer_set_oc_value(TIM3, TIM_OC1, PULSE*4);
        //timer_enable_oc_output(TIM3,TIM_OC1);
        timer_disable_oc_output(TIM3,TIM_OC2);
        //timer_disable_irq(TIM3,TIM_DIER_CC2IE);
    }
    if(timer_interrupt_source(TIM3,TIM_SR_CC3IF)) {
        timer_clear_flag(TIM3,TIM_SR_CC3IF);
        timer_enable_oc_output(TIM3,TIM_OC4);
        //timer_enable_irq(TIM3,TIM_DIER_CC4IE);
    }
    if(timer_interrupt_source(TIM3,TIM_SR_CC4IF)) {
        //timer_disable_oc_output(TIM3,TIM_OC4);
    }
}

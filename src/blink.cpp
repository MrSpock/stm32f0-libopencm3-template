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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "stepper.h"

#define SYSTICK_FREQ 1000
#define M1 GPIO8
#define M2 GPIO9
#define M3 GPIO10
#define M4 GPIO11

uint32_t sys_cnt;
void delay(volatile uint32_t);

/* Called when systick fires */
void sys_tick_handler(void)
{
     sys_cnt++;
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

/* set STM32 to clock by 48MHz from HSI oscillator */
static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();

    /* Enable clocks to the GPIO subsystems */
    //rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);
}

static void gpio_setup(void)
{
    /* Select pin functions. PC8/PC9 are the two LEDs on the
      STM32F0DISCOVERY board. */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, M1|M2|M3|M4);

    /* set GPIOA to AF 0 */
    //gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
}

static void mco_setup(void)
{
    /* Enable system clock output on pin PA8 (so it can be checked with a
      scope) */
    rcc_set_mco(RCC_CFGR_MCO_SYSCLK);
}
void delay(volatile uint32_t t) {
    t *= 3700;
    while(t--);

}
int main(void)
{
    clock_setup();
    gpio_setup();
    mco_setup();
    systick_setup(SYSTICK_FREQ);
    StepperMotor engine(GPIOA, M1,M2,M3,M4);

    gpio_port_write(GPIOA,0);
    //engine.step(100);
    while (1) {
       engine.step(100);
       /*
    for(i = 0; i < sizeof(steps)/sizeof(steps[0]);i++) {
    gpio_port_write(GPIOA,steps[i]);
    delay(3);
    }
    */
    /*
    gpio_set(GPIOA,M1|M3);
    delay(15);
    gpio_set(GPIOA,M1|M2);
    delay(800);
    gpio_set(GPIOA,M2);
    delay(800);
    gpio_set(GPIOA,M3|M2);
    delay(800);
    gpio_set(GPIOA,M3);
    delay(800);
    gpio_set(GPIOA,M3|M4);
    delay(800);
    gpio_set(GPIOA,M4);
    delay(800);
    gpio_set(GPIOA,M4|M1);
    delay(800);
*/
//    delay(10);
    };
}



#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "../include/stepper.h"

StepperMotor::StepperMotor(uint32_t GPIO_P, uint32_t in1, uint32_t in2, uint32_t in3, uint32_t in4):m1(in1),m2(in2),m3(in3),m4(in4),GPIO_PORT(GPIO_P)
{
    /*
    this->GPIO_PORT = GPIO_P;
    this->m1 = in1;
    this->m2 = in2;
    this->m3 = in3;
    this->m4 = in4;
    //m2|m3, m2|m4, m4|m1 );
    */
    gpio_mode_setup(GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, m1|m2|m3|m4);
}

void StepperMotor::delay(volatile uint32_t t) {
        t *= 3700;
        while(t--);
                
}

void StepperMotor::step(uint32_t no_steps)
{
//    uint32_t step_code[] = { m1|m3,m2|m3,m2|m4,m1|m4};
    //uint32_t i;
    volatile uint32_t j;
    for(j = 0;j<=no_steps;++j)
    //while (no_steps < j)
    {
         gpio_clear(GPIO_PORT,m4);
         gpio_set(GPIO_PORT,m1);
         gpio_set(GPIO_PORT,m3);
         this->delay(4);
         gpio_clear(GPIO_PORT,m1);
         gpio_set(GPIO_PORT,m2);
         this->delay(4);
         gpio_clear(GPIO_PORT,m3);
         gpio_set(GPIO_PORT,m4);
         this->delay(4);
         gpio_clear(GPIO_PORT,m2);
         gpio_set(GPIO_PORT,m1);
         this->delay(4);
         /*
         for(i = 0; i < sizeof(step_code)/sizeof(step_code[0]);i++) {
          //gpio_set(GPIO_PORT,step_code[i]);
          //gpio_port_write(GPIO_PORT,step_code[i]);
          this->delay(4);
    }
    */
    }
}




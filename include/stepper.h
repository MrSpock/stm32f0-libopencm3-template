#ifndef STEPPER_H
#define STEPPER_H

class StepperMotor {
    void delay(volatile uint32_t t);
    uint32_t m1,m2,m3,m4;
    uint32_t GPIO_PORT;

    public:
        StepperMotor(uint32_t GPIO_PORT , uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4);
        void step(uint32_t n);
        
};


#endif


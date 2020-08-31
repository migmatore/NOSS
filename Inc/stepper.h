//
// Created by kazak on 27.07.2020.
//

#ifndef NOSS_STEPPER_H
#define NOSS_STEPPER_H

#include "main.h"

#define abs(x) ((x)>0?(x):-(x))

#define IN_GPIO_PORT GPIOB
#define in1 IN1_Pin
#define in2 IN2_Pin
#define in3 IN3_Pin
#define in4 IN4_Pin

typedef struct {
    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in ms, based on speed
    int number_of_steps;      // total number of steps this motor can take
    int pin_count;            // how many pins are in use.
    int step_number;          // which step the motor is on

    // motor pin numbers:
    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;
    int motor_pin_5;          // Only 5 phase motor

    unsigned long last_step_time; // time stamp in us of when the last step was taken
} stepper_t;

void stepper_init(stepper_t *stepper, int number_of_steps, int motor_pin_1, int motor_pin_2,
                  int motor_pin_3, int motor_pin_4);

void setSpeed(stepper_t *stepper, long whatSpeed);

void step(stepper_t *stepper, int number_of_steps);

void stepMotor(stepper_t *stepper, int this_step);

#endif //NOSS_STEPPER_H

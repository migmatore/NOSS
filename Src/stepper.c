//
// Created by kazak on 27.07.2020.
//

#include "stepper.h"
#include "stm32f3xx_hal.h"

void stepper_init(stepper_t *stepper, int number_of_steps, int motor_pin_1, int motor_pin_2,
                  int motor_pin_3, int motor_pin_4) {
    stepper->step_number = 0;    // which step the motor is on
    stepper->direction = 0;      // motor direction
    stepper->last_step_time = 0; // time stamp in us of the last step taken
    stepper->number_of_steps = number_of_steps; // total number of steps for this motor

    // Arduino pins for the motor control connection:
    stepper->motor_pin_1 = motor_pin_1;
    stepper->motor_pin_2 = motor_pin_2;
    stepper->motor_pin_3 = motor_pin_3;
    stepper->motor_pin_4 = motor_pin_4;

    // pin_count is used by the stepMotor() method:
    stepper->pin_count = 4;
}

void setSpeed(stepper_t *stepper, long whatSpeed) {
    stepper->step_delay = 60L * 1000L * 1000L / stepper->number_of_steps / whatSpeed;
}

void step(stepper_t *stepper, int steps_to_move) {
    int steps_left = abs(steps_to_move);  // how many steps to take

    // determine direction based on whether steps_to_mode is + or -:
    if (steps_to_move > 0) { stepper->direction = 1; }
    if (steps_to_move < 0) { stepper->direction = 0; }

    //volatile uint32_t now = 0;
    // decrement the number of steps, moving one step each time:
    while (steps_left > 0) {
        uint32_t now = HAL_GetTick();
        // move only if the appropriate delay has passed:
        if (now - stepper->last_step_time >= stepper->step_delay) {
            // get the timeStamp of when you stepped:
            stepper->last_step_time = now;
            // increment or decrement the step number,
            // depending on direction:
            if (stepper->direction == 1) {
                stepper->step_number++;
                if (stepper->step_number == stepper->number_of_steps) {
                    stepper->step_number = 0;
                }
            } else {
                if (stepper->step_number == 0) {
                    stepper->step_number = stepper->number_of_steps;
                }
                stepper->step_number--;
            }
            // decrement the steps left:
            steps_left--;
            // step the motor to step number 0, 1, ..., {3 or 10}
            if (stepper->pin_count == 5)
                stepMotor(stepper, stepper->step_number % 10);
            else
                stepMotor(stepper, stepper->step_number % 4);
        }
    }
}

void stepMotor(stepper_t *stepper, int thisStep) {
    if (stepper->pin_count == 2) {
        switch (thisStep) {
            case 0:  // 01
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_SET);
                break;
            case 1:  // 11
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_SET);
                break;
            case 2:  // 10
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_RESET);
                break;
            case 3:  // 00
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_RESET);
                break;
        }
    }
    if (stepper->pin_count == 4) {
        switch (thisStep) {
            case 0:  // 1010
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_3, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_4, GPIO_PIN_RESET);
                break;
            case 1:  // 0110
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_3, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_4, GPIO_PIN_RESET);
                break;
            case 2:  //0101
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_3, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_4, GPIO_PIN_SET);
                break;
            case 3:  //1001
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_2, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_3, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN_GPIO_PORT, stepper->motor_pin_4, GPIO_PIN_SET);
                break;
        }
    }

//    if (this->pin_count == 5) {
//        switch (thisStep) {
//            case 0:  // 01101
//                digitalWrite(motor_pin_1, LOW);
//                digitalWrite(motor_pin_2, HIGH);
//                digitalWrite(motor_pin_3, HIGH);
//                digitalWrite(motor_pin_4, LOW);
//                digitalWrite(motor_pin_5, HIGH);
//                break;
//            case 1:  // 01001
//                digitalWrite(motor_pin_1, LOW);
//                digitalWrite(motor_pin_2, HIGH);
//                digitalWrite(motor_pin_3, LOW);
//                digitalWrite(motor_pin_4, LOW);
//                digitalWrite(motor_pin_5, HIGH);
//                break;
//            case 2:  // 01011
//                digitalWrite(motor_pin_1, LOW);
//                digitalWrite(motor_pin_2, HIGH);
//                digitalWrite(motor_pin_3, LOW);
//                digitalWrite(motor_pin_4, HIGH);
//                digitalWrite(motor_pin_5, HIGH);
//                break;
//            case 3:  // 01010
//                digitalWrite(motor_pin_1, LOW);
//                digitalWrite(motor_pin_2, HIGH);
//                digitalWrite(motor_pin_3, LOW);
//                digitalWrite(motor_pin_4, HIGH);
//                digitalWrite(motor_pin_5, LOW);
//                break;
//            case 4:  // 11010
//                digitalWrite(motor_pin_1, HIGH);
//                digitalWrite(motor_pin_2, HIGH);
//                digitalWrite(motor_pin_3, LOW);
//                digitalWrite(motor_pin_4, HIGH);
//                digitalWrite(motor_pin_5, LOW);
//                break;
//            case 5:  // 10010
//                digitalWrite(motor_pin_1, HIGH);
//                digitalWrite(motor_pin_2, LOW);
//                digitalWrite(motor_pin_3, LOW);
//                digitalWrite(motor_pin_4, HIGH);
//                digitalWrite(motor_pin_5, LOW);
//                break;
//            case 6:  // 10110
//                digitalWrite(motor_pin_1, HIGH);
//                digitalWrite(motor_pin_2, LOW);
//                digitalWrite(motor_pin_3, HIGH);
//                digitalWrite(motor_pin_4, HIGH);
//                digitalWrite(motor_pin_5, LOW);
//                break;
//            case 7:  // 10100
//                digitalWrite(motor_pin_1, HIGH);
//                digitalWrite(motor_pin_2, LOW);
//                digitalWrite(motor_pin_3, HIGH);
//                digitalWrite(motor_pin_4, LOW);
//                digitalWrite(motor_pin_5, LOW);
//                break;
//            case 8:  // 10101
//                digitalWrite(motor_pin_1, HIGH);
//                digitalWrite(motor_pin_2, LOW);
//                digitalWrite(motor_pin_3, HIGH);
//                digitalWrite(motor_pin_4, LOW);
//                digitalWrite(motor_pin_5, HIGH);
//                break;
//            case 9:  // 00101
//                digitalWrite(motor_pin_1, LOW);
//                digitalWrite(motor_pin_2, LOW);
//                digitalWrite(motor_pin_3, HIGH);
//                digitalWrite(motor_pin_4, LOW);
//                digitalWrite(motor_pin_5, HIGH);
//                break;
//        }
//    }
}
//
// Created by rover on 25.06.22.
//

#ifndef HELLO_2_MSGS_H
#define HELLO_2_MSGS_H

typedef enum {
    BTN1    = 1,
    BTN2    = 2,
    BTN3    = 3,
} EButton;

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    EButton button;
} Button;

Button BUTTONS[3] = {
        { .GPIOx = GPIOA, .GPIO_Pin = BTN1_Pin, .button = BTN1 },
        { .GPIOx = GPIOA, .GPIO_Pin = BTN2_Pin, .button = BTN2 },
        { .GPIOx = GPIOB, .GPIO_Pin = BTN3_Pin, .button = BTN3 },
};

typedef enum {
    PRESSED,
    RELEASED,
} EButtonState;

typedef struct {
    EButton button;
    EButtonState state;
} TButtonMsg;

#endif //HELLO_2_MSGS_H

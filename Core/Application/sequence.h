//
// Created by rover on 26.06.22.
//

#ifndef HELLO_2_SEQUENCE_H
#define HELLO_2_SEQUENCE_H

#include "libs.h"
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"


typedef enum {
    EActionSet,
    EActionWait,
    EActionEnd,
} EActionType;

typedef struct {
    u32_t *channel;
    i32_t value;
    EActionType type;
    i8_t duration;
} TAction;

typedef struct {
    TAction *actions;
    i8_t elemCnt;
} TSequence;

TSequence sequence = {
        {
                {1, 1, EActionSet, 10},

            }, 2
};

void Sequence_Process(TSequence *seq) {
    for (i8_t i = 0; i < seq->elemCnt; ++i) {
        TAction action = seq->actions[i];
        switch (action.type) {
            case EActionSet:
                *action.channel = action.value;
                break;
            case EActionWait:
                osDelay(action.duration);
                break;
            case EActionEnd:
                return;
        }
    }
}

#endif //HELLO_2_SEQUENCE_H

//
// Created by rover on 26.06.22.
//

#ifndef HELLO_2_SEQUENCE_H
#define HELLO_2_SEQUENCE_H

#include "libs.h"
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.h"

namespace Seq {

    class PWM {
    public:
        PWM() = default;

        explicit PWM(volatile u32_t *reg, u32_t initValue, i32_t period = 255) {
            this->reg = reg;
            *(this->reg) = initValue;
            this->period = period;
        }

        void set(i32_t value) {
            *(this->reg) = value;
        }

        i32_t get() {
            return *reg;
        }

        void add(i32_t value) {
            *reg += value;
        }

    private:
        volatile u32_t *reg;
        u16_t period;
    };

    class Sequencer {
    public:
        Sequencer() = default;

        explicit Sequencer(PWM pwm) {
            this->pwm = pwm;
            this->targetValue = this->pwm.get();
            this->animationInProgress = false;
            this->increment = 0;
        }

        void set(i32_t value) {
            this->set(value, 0);
        }

        /**
         * @param value to be set
         * @param duration (animation time in ms)
         */
        void set(i32_t value, i32_t duration) {
            targetValue = value;
            i32_t diff = targetValue - pwm.get();
            if (diff == 0) {
                animationInProgress = false;
                return;
            }
            if (diff < 0) increment = -1;
            else increment = 1;

            ticksToIncrement = duration / diff * increment;
            currentTickCnt = ticksToIncrement;
            animationInProgress = true;
        }

        bool tick() {
            if (!animationInProgress) {
                return animationInProgress;
            }
            if (ticksToIncrement == 0) {
                pwm.set(targetValue);
                animationInProgress = false;
                return animationInProgress;
            }
            if (--currentTickCnt == 0) {
                currentTickCnt = ticksToIncrement;
                pwm.add(increment);
                if (pwm.get() == targetValue) {
                    animationInProgress = false;
                }
            }
            return animationInProgress;
        }
    private:
        PWM pwm;
        i32_t targetValue;
        // How many ticks have to happen before increment by 1
        i32_t ticksToIncrement;
        i32_t currentTickCnt;
        i32_t increment;
        bool animationInProgress;
    };

    struct RGB {
        i32_t r, g ,b;
    };

    class Led {
    public:
        Led(volatile u32_t *r, volatile u32_t *g, volatile u32_t *b) {
            this->r = Sequencer(PWM(r, 255, 255));
            this->g = Sequencer(PWM(g, 255, 255));
            this->b = Sequencer(PWM(b, 255, 255));
        }

        Led& set(RGB value) {
            set(value, 0);
            return *this;
        }

        Led& set(RGB value, i32_t duration) {
            r.set(value.r, duration);
            g.set(value.g, duration);
            b.set(value.b, duration);

            while (r.tick() | g.tick() | b.tick()) {
                osDelay(1);
            }
            return *this;
        }

        Led& wait(i32_t delay) {
            osDelay(delay);
            return *this;
        }
    private:
        Sequencer r, g, b;
    };

    class Vibro {
    public:
        Vibro(volatile u32_t *v) {
            this->vibro = Sequencer(PWM(v, 99, 99));
        }

        Vibro& set(i32_t value) {
            set(value, 0);
            return *this;
        }

        Vibro& set(i32_t value, i32_t duration) {
            vibro.set(value, duration);

            while (vibro.tick()) {
                osDelay(1);
            }
            return *this;
        }

        Vibro& wait(i32_t delay) {
            osDelay(delay);
            return *this;
        }
    private:
        Sequencer vibro;
    };
} // END NAMESPACE SEQ

#endif //HELLO_2_SEQUENCE_H

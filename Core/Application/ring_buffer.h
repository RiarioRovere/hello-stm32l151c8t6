//
// Created by rover on 03.07.22.
//

#ifndef HELLO_2_RING_BUFFER_H
#define HELLO_2_RING_BUFFER_H

#include "libs.h"

constexpr i32_t BUF_SIZE = 100;

class TRingBuffer {
public:
    TRingBuffer() {
        head = 0;
        tail = BUF_SIZE - 1;
        wordsCnt = 0;
    }
    void Add(u8_t value) {
        tail = next(tail);
        buffer[tail] = value;
        if (value == '\n') {
            ++wordsCnt;
        }
    }
    u8_t Take() {
        u8_t rt = buffer[head];
        head = next(head);
        if (rt == '\n') {
            --wordsCnt;
        }
        return rt;
    }
    i32_t Available() {
        if (tail >= head) return tail - head;
        return tail + BUF_SIZE - head;
    }

    i32_t WordsCnt() {
        return wordsCnt;
    }
private:
    i32_t next(i32_t ptr) {
        if (++ptr == BUF_SIZE) {
            ptr = 0;
        }
        return ptr;
    }

    u8_t buffer[BUF_SIZE];
    i32_t head, tail;
    i32_t wordsCnt;
};

#endif //HELLO_2_RING_BUFFER_H

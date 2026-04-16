#ifndef _XINPUTPAD_H_
#define _XINPUTPAD_H_
#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t rid;
    uint8_t rsize;
    uint16_t buttons;
    uint8_t lt;
    uint8_t rt;
    int16_t l_x;
    int16_t l_y;
    int16_t r_x;
    int16_t r_y;
    uint8_t reserved_1[6];
} ReportDataXinput;

#endif
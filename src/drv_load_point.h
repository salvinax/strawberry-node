#ifndef LOAD_POINT_H
#define LOAD_POINT_H

#include <stddef.h>
#include <stdint.h>

struct load_sample 
{
    int32_t integer;
    int32_t fractional;
    int32_t weight_mg;
};

int hx711_probe_init(void);
void measure(struct load_sample *labs);
void calibrate(void);
#endif
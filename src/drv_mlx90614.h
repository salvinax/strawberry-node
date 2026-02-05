#pragma once
#include <zephyr/device.h>
#include <stdint.h>

#define MLX90614_ADDR 0x5A  

struct mlx90614_sample {
    float amb_c;
    float obj_c;
};

int mlx90614_init(void);
int mlx90614_read(struct mlx90614_sample *out);

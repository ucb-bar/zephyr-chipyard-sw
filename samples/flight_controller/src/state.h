#pragma once

#include <zephyr/kernel.h>

struct fc_state {
    float roll;
    float pitch;
    float yaw;

    float px;
    float py;
    float pz;

    float vx;
    float vy;
    float vz;
};

extern struct fc_state shared_state;
extern struct k_mutex state_mutex;

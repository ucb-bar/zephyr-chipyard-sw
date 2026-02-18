#include "state.h"

struct fc_state shared_state = {
    .roll = 0,
    .pitch = 0,
    .yaw = 0,
    .px = 0,
    .py = 0,
    .pz = 0,
    .vx = 0,
    .vy = 0,
    .vz = 0
};

struct k_mutex state_mutex;

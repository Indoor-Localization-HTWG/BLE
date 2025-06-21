#ifndef EST_POS_H
#define EST_POS_H

#include "aoa_calc.h"

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/bluetooth.h>

#define NUM_BEACONS 1

struct beacon_data
{
    bt_addr_le_t addr;
    double yaw[MAX_AOA_SAMPLES];
    double pitch[MAX_AOA_SAMPLES];
    int sample_count;
    int write_idx;
    beacon_t position;
};

extern struct beacon_data beacons[NUM_BEACONS];

bool estimate_position(point3d_t prev_xy, point3d_t *est_xy);
void init_beacons(void);

#endif // EST_POS_H
#ifndef EST_POS_H
#define EST_POS_H

#include "aoa_calc.h"

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/bluetooth.h>

#define NUM_BEACONS 1

typedef struct
{
    point2d_t position;
    double angle; // Angle in degrees
} beacon_t;

struct beacon_data
{
    bt_addr_le_t addr;
    double aoa_samples[MAX_AOA_SAMPLES];
    int sample_count;
    int write_idx;
    beacon_t position;
};

extern struct beacon_data beacons[NUM_BEACONS];

bool estimate_position(point2d_t prev_xy, point2d_t *est_xy);
void init_beacons(void);

#endif // EST_POS_H
#ifndef AOA_CALC_H
#define AOA_CALC_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/direction.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_AOA_SAMPLES 32
#define NUM_BEACONS 2

typedef struct
{
	double x;
	double y;
} point2d_t;

typedef struct
{
	point2d_t position;
	double angle; // Angle in degrees
} beacon_t;

struct beacon_data
{
	bt_addr_le_t addr;
	double aoa_samples[MAX_AOA_SAMPLES];
	int write_idx;
	int sample_count;
	beacon_t position;
};

extern struct beacon_data beacons[NUM_BEACONS];

// All functions now use return parameters and return void or bool for success
bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report,
				   uint8_t num_antennas, double *angle_deg);
double normalize_angle_180(double angle);
bool gradient_descent(point2d_t prev_xy, point2d_t *est_xy);
bool estimate_position(point2d_t prev_xy, point2d_t *est_xy);
void init_beacons(void);

#endif // AOA_CALC_H
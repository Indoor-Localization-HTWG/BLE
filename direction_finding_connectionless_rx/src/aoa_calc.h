#ifndef AOA_CALC_H
#define AOA_CALC_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/direction.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT 299792458.0f
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define BLE_FREQ 2.402e9f // Hz (for channel 37)
#define LAMBDA (SPEED_OF_LIGHT / BLE_FREQ)
#define D 0.055f // Antenna spacing (meters)

#define REFERENCE_SAMPLES 16
#define ANTENNA_PATTERN_LEN 16

#define AOA_SMOOTH_WINDOW 8
#define MAX_AOA_SAMPLES 32

typedef struct
{
	double x;
	double y;
} point2d_t;

typedef struct
{
	point2d_t position;
	double angle; // Angle in degrees
} own_beacon_t;

struct own_beacon_data
{
	bt_addr_le_t addr;
	double aoa_samples[MAX_AOA_SAMPLES];
	int write_idx;
	int sample_count;
	own_beacon_t position;
};

extern struct own_beacon_data own_beacon;

// All functions now use return parameters and return void or bool for success
bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report,
				   uint8_t num_antennas, double *angle_deg);
double normalize_angle_180(double angle);
double deg2rad(double deg);
bool gradient_descent(point2d_t prev_xy, point2d_t *est_xy);
bool estimate_position(point2d_t prev_xy, point2d_t *est_xy);
void init_beacon(void);

#endif // AOA_CALC_H
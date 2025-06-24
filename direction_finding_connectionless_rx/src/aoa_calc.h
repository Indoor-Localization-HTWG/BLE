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
#define BLE_FREQ 2.44e9f // Hz (for channel 37)
#define LAMBDA (SPEED_OF_LIGHT / BLE_FREQ)
#define D 0.05f // Antenna spacing (meters)

#define ANTENNA_PATTERN_LEN 16

#define AOA_SMOOTH_WINDOW 8
#define MAX_AOA_SAMPLES 32
#define REFFERENCE_SAMPLES 8

typedef struct
{
	double x;
	double y;
	double z;
} point3d_t;

typedef struct
{
	double yaw;	  // rototion Angle in degrees
	double pitch; // Elevation angle in degrees
} rot3d_t;

typedef struct
{
	point3d_t position;
	rot3d_t rotation;
} beacon_t;

struct own_beacon_data
{
	bt_addr_le_t addr;
	double yaw[MAX_AOA_SAMPLES];
	double pitch[MAX_AOA_SAMPLES];
	int write_idx;
	int sample_count;
	beacon_t position;
};

extern struct own_beacon_data own_beacon;

// All functions now use return parameters and return void or bool for success
bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report, rot3d_t *angle_deg);
double normalize_angle_180(double angle);
double deg2rad(double deg);
bool gradient_descent(point3d_t prev_xy, point3d_t *est_xy);
bool estimate_position(point3d_t prev_xy, point3d_t *est_xy);
void init_beacon(void);

#endif // AOA_CALC_H
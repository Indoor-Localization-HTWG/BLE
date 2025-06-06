#ifndef AOA_CALC_H
#define AOA_CALC_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/direction.h>
#include <stdint.h>

#define MAX_AOA_SAMPLES 32
#define NUM_BEACONS     3

typedef struct {
	double x;
	double y;
} point2d_t;

typedef struct {
	point2d_t position;
	double angle; // Angle in degrees
} beacon_t;

struct beacon_data {
	bt_addr_le_t addr;
	double aoa_samples[MAX_AOA_SAMPLES];
	int write_idx;
	int sample_count;
};

extern struct beacon_data beacons[NUM_BEACONS];
extern beacon_t beacon_positions[NUM_BEACONS];

double calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report,
		     uint8_t num_antennas);
double deg2rad(double deg);
double distance_to_line(double x, double y, double beacon_x, double beacon_y, double aoa_deg,
			double beacon_rotation_deg);
double total_cost(double x, double y);
void compute_gradient(double x, double y, double *grad_x, double *grad_y);
void gradient_descent(double *out_x, double *out_y);
void estimate_position(void);
void init_beacons(void);

#endif // AOA_CALC_H

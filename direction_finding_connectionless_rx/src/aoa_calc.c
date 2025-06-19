#include "aoa_calc.h"
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/direction.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <string.h>
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
static double aoa_window[AOA_SMOOTH_WINDOW];
static int aoa_idx = 0;
static int aoa_count = 0;

struct beacon_data beacons[NUM_BEACONS];

double deg2rad(double deg)
{
	return deg * (double)M_PI / 180.0f;
}

double normalize_angle_180(double angle)
{
	double norm = fmod(angle + 180.0, 360.0);
	if (norm < 0)
		norm += 360.0;
	return norm - 180.0;
}

// Returns true if AoA could be calculated, false otherwise
bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report,
				   uint8_t max_antennas, double *angle_deg)
{
	double phases[ANTENNA_PATTERN_LEN];
	int valid_count = 0;

	for (int i = 0; i < max_antennas; i++)
	{
		int idx = REFERENCE_SAMPLES + i;
		int I = report->sample[idx].i;
		int Q = report->sample[idx].q;
		if (I == 0 && Q == 0)
			continue; // skip zero IQ
		phases[valid_count++] = atan2f(Q, I);
	}

	if (valid_count < 2)
	{
		printk("Not enough valid samples to calculate AoA\n");
		return false;
	}

	double avg_delta_phi = 0.0;
	for (int i = 0; i < valid_count - 1; i++)
	{
		double delta = phases[i + 1] - phases[i];
		if (delta > M_PI)
			delta -= 2 * M_PI;
		if (delta < -M_PI)
			delta += 2 * M_PI;
		avg_delta_phi += delta;
	}
	avg_delta_phi /= (valid_count - 1);

	double x = (avg_delta_phi * LAMBDA) / (2 * M_PI * D);
	if (x > 1.0)
		x = 1.0;
	if (x < -1.0)
		x = -1.0;
	double angle_rad = asinf(x);
	*angle_deg = angle_rad * 180.0 / M_PI;
	return true;
}

void smooth_aoa(double new_angle, double *smoothed_angle)
{
	aoa_window[aoa_idx] = new_angle;
	aoa_idx = (aoa_idx + 1) % AOA_SMOOTH_WINDOW;
	if (aoa_count < AOA_SMOOTH_WINDOW)
		aoa_count++;
	double sum = 0;
	for (int i = 0; i < aoa_count; i++)
		sum += aoa_window[i];
	*smoothed_angle = sum / aoa_count;
}
const beacon_t beacon_position[NUM_BEACONS] = {
	{{0.0f, 0.0f}, 0},	 // Beacon 1 position and z-rotation
	{{1.0f, 0.0f}, -90}, // Beacon 2 position and z-rotation
};
const char *const beacon_adress[NUM_BEACONS] = {
	"FA:23:5E:09:F1:39", // Beacon 1 address
	"FA:23:5E:09:F1:39", // Beacon 2 address
};

void init_beacons(void)
{
	for (int i = 0; i < NUM_BEACONS; i++)
	{
		beacons[i].addr = (bt_addr_le_t){0};
		bt_addr_le_from_str(beacon_adress[i], "random", &beacons[i].addr);
		beacons[i].write_idx = 0;
		beacons[i].sample_count = 0;
		memset(beacons[i].aoa_samples, 0, sizeof(beacons[i].aoa_samples));
		beacons[i].position = beacon_position[i];
	}
}

void distance_to_line(point2d_t p, point2d_t beacon, double aoa_deg, double beacon_rotation_deg, double *dist)
{
	double absolute_angle_deg = aoa_deg + beacon_rotation_deg;
	double angle_rad = deg2rad(absolute_angle_deg);
	double dx = p.x - beacon.x;
	double dy = p.y - beacon.y;
	*dist = dx * sinf(angle_rad) - dy * cosf(angle_rad);
}

void total_cost(point2d_t p, double *cost)
{
	*cost = 0.0f;
	for (int i = 0; i < NUM_BEACONS; i++)
	{
		if (beacons[i].sample_count == 0)
		{
			continue;
		}
		int n = beacons[i].sample_count < 3 ? beacons[i].sample_count : 3;
		double sum_angle = 0.0f;
		for (int j = 0; j < n; j++)
		{
			int idx =
				(beacons[i].write_idx - 1 - j + MAX_AOA_SAMPLES) % MAX_AOA_SAMPLES;
			sum_angle += beacons[i].aoa_samples[idx];
		}
		double avg_angle = sum_angle / n;
		double dist;
		distance_to_line(
			p,
			beacon_position[i].position,
			avg_angle,
			beacon_position[i].angle,
			&dist);
		*cost += dist * dist;
	}
}

void compute_gradient(point2d_t p, point2d_t *grad)
{
	double h = 0.001f;
	point2d_t p_x_plus = {p.x + h, p.y};  // Point shifted +h in x
	point2d_t p_x_minus = {p.x - h, p.y}; // Point shifted -h in x
	point2d_t p_y_plus = {p.x, p.y + h};  // Point shifted +h in y
	point2d_t p_y_minus = {p.x, p.y - h}; // Point shifted -h in y

	double cost_x_plus, cost_x_minus, cost_y_plus, cost_y_minus;
	total_cost(p_x_plus, &cost_x_plus);
	total_cost(p_x_minus, &cost_x_minus);
	total_cost(p_y_plus, &cost_y_plus);
	total_cost(p_y_minus, &cost_y_minus);

	grad->x = (cost_x_plus - cost_x_minus) / (2.0f * h);
	grad->y = (cost_y_plus - cost_y_minus) / (2.0f * h);
}

// Returns true if successful, false if not enough data
bool gradient_descent(point2d_t prev_xy, point2d_t *est_xy)
{
	int max_iterations = 50;
	double epsilon = 1e-1f;
	int i = 0;
	point2d_t grad;
	double norm = INFINITY;
	point2d_t p = prev_xy;

	// Check if enough data is available
	for (int i = 0; i < NUM_BEACONS; i++)
	{
		if (beacons[i].sample_count < 3)
		{
			printk("Nicht genügend AoA-Daten für Beacon %d\n", i);
			*est_xy = prev_xy;
			return false;
		}
	}

	do
	{
		double learning_rate = 0.01f;
		compute_gradient(p, &grad);
		norm = sqrtf(grad.x * grad.x + grad.y * grad.y);
		printk("Iteration %d: x = %d, y = %d, grad_x = %d, grad_y = %d, norm = %d\n",
			   i, (int)(p.x * 1000), (int)(p.y * 1000), (int)(grad.x * 1000), (int)(grad.y * 1000), (int)(norm * 1000000));
		p.x -= learning_rate * grad.x;
		p.y -= learning_rate * grad.y;
	} while (++i < max_iterations && norm > epsilon);

	est_xy->x = p.x;
	est_xy->y = p.y;
	return true;
}

// Returns true if successful, false if not enough data
bool estimate_position(point2d_t prev_xy, point2d_t *est_xy)
{
	return gradient_descent(prev_xy, est_xy);
}
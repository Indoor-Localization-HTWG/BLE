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

#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT 299792458.0f
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define BLE_FREQ 2.402e9f // Hz (for channel 37)
#define LAMBDA (SPEED_OF_LIGHT / BLE_FREQ)
#define D (LAMBDA / 2.0f) // Antenna spacing (meters)

struct beacon_data beacons[NUM_BEACONS];

beacon_t beacon_positions[NUM_BEACONS] = {
	{{0.0f, 0.0f}, 0},	 // Beacon 1 position and z-rotation
	{{1.0f, 0.0f}, -90}, // Beacon 2 position and z-rotation
	{{0.0f, 1.0f}, 90}	 // Beacon 3 position and z-rotation
};

double deg2rad(double deg)
{
	return deg * (double)M_PI / 180.0f;
}

double calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report,
					 uint8_t num_antennas)
{
	double phases[num_antennas];
	for (int i = 0; i < num_antennas; i++)
	{
		phases[i] = atan2f(report->sample[i].q, report->sample[i].i);
	}
	double avg_delta_phi = 0.0f;
	for (int i = 0; i < num_antennas - 1; i++)
	{
		double delta = phases[i + 1] - phases[i];
		if (delta > M_PI)
		{
			delta -= 2 * M_PI;
		}
		if (delta < -M_PI)
		{
			delta += 2 * M_PI;
		}
		avg_delta_phi += delta;
	}
	avg_delta_phi /= (num_antennas - 1);
	double angle_rad = asinf((avg_delta_phi * LAMBDA) / (2 * M_PI * D));
	double angle_deg = angle_rad * 180.0f / M_PI;
	return angle_deg;
}

void init_beacons(void)
{
	bt_addr_le_from_str("FA:23:5E:09:F1:39", "random", &beacons[0].addr);
	bt_addr_le_from_str("AA:BB:CC:DD:EE:02", "random", &beacons[1].addr);
	bt_addr_le_from_str("AA:BB:CC:DD:EE:03", "random", &beacons[2].addr);
	for (int i = 0; i < NUM_BEACONS; i++)
	{
		beacons[i].sample_count = 0;
	}
}

double distance_to_line(double x, double y, double beacon_x, double beacon_y, double aoa_deg,
						double beacon_rotation_deg)
{
	double absolute_angle_deg = aoa_deg + beacon_rotation_deg;
	double angle_rad = deg2rad(absolute_angle_deg);
	double dx = x - beacon_x;
	double dy = y - beacon_y;
	double dist = dx * sinf(angle_rad) - dy * cosf(angle_rad);
	return dist;
}

double total_cost(double x, double y)
{
	double cost = 0.0f;
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
		double dist = distance_to_line(x, y, beacon_positions[i].position.x,
									   beacon_positions[i].position.y, avg_angle,
									   beacon_positions[i].angle);
		cost += dist * dist;
	}
	return cost;
}

void compute_gradient(double x, double y, double *grad_x, double *grad_y)
{
	double h = 0.001f;
	double dx = (total_cost(x + h, y) - total_cost(x - h, y)) / (2.0f * h);
	double dy = (total_cost(x, y + h) - total_cost(x, y - h)) / (2.0f * h);
	*grad_x = dx;
	*grad_y = dy;
}

void gradient_descent(double *out_x, double *out_y)
{
	double x = 0.5f, y = 0.5f;
	double learning_rate = 0.05f;
	int max_iterations = 50;
	double epsilon = 1e-5f;
	for (int i = 0; i < max_iterations; i++)
	{
		double grad_x, grad_y;
		compute_gradient(x, y, &grad_x, &grad_y);
		double norm = sqrtf(grad_x * grad_x + grad_y * grad_y);
		if (norm < epsilon)
		{
			break;
		}
		x -= learning_rate * grad_x;
		y -= learning_rate * grad_y;
	}
	*out_x = x;
	*out_y = y;
}

void estimate_position(void)
{
	for (int i = 0; i < NUM_BEACONS; i++)
	{
		if (beacons[i].sample_count < 3)
		{
			printk("Nicht genügend AoA-Daten für Beacon %d\n", i);
			return;
		}
	}
	double est_x, est_y;
	gradient_descent(&est_x, &est_y);
	printk("Geschätzte Position: x = %.3f, y = %.3f\n", est_x, est_y);
}

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

static double aoa_window[AOA_SMOOTH_WINDOW];
static int aoa_idx = 0;
static int aoa_count = 0;
const char *const beacon_adress = "FA:23:5E:09:F1:39";

struct own_beacon_data own_beacon;

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

void calculate_yaw(int samplesize, double *phases, double *angle_deg)
{

	double avg_delta_yaw = 0.0;
	/**
	 * Calculate the average phase difference
	 * only compare samples that are in the same row
	 * e.g. skip phases[4]-phases[0], phases[8]-phases[7], ...
	 */
	int num_delta_phases = 0;
	for (int i = 0; i < samplesize; i++)
	{
		if (i >> 2 == 0)
		{
			continue;
		}
		num_delta_phases++;
		double delta_yaw = phases[i + 1] - phases[i];
		if (delta_yaw < -M_PI)
			delta_yaw += 2 * M_PI;
		if (delta_yaw > M_PI)
			delta_yaw -= 2 * M_PI;
		avg_delta_yaw += delta_yaw;
	}
	avg_delta_yaw /= num_delta_phases;

	/**
	 * Calculate the angle
	 * and normalize it to the range [-180, 180]
	 */
	double x = (avg_delta_yaw * LAMBDA) / (2 * M_PI * D);
	if (x > 1.0)
		x = 1.0;
	if (x < -1.0)
		x = -1.0;
	double angle_rad = asinf(x);
	*angle_deg = angle_rad * 180.0 / M_PI;
	return;
}

void calculate_pitch(double *phases, double *angle_deg)
{
	double avg_delta_pitch = 0.0;
	int num_delta_phases = 0;
	/**
	 * Calculate the average phase difference
	 * only compare samples that are in the same column
	 */
	for (int col = 0; col < 3; col++)
	{
		for (int row = 0; row < 3; row++)
		{
			int idx1 = row * 4 + col;
			int idx2 = (row + 1) * 4 + col;
			double delta_pitch = phases[(row + 1) * 4 + col] - phases[row * 4 + col];
			if (delta_pitch < -M_PI)
				delta_pitch += 2 * M_PI;
			if (delta_pitch > M_PI)
				delta_pitch -= 2 * M_PI;
			avg_delta_pitch += delta_pitch;
			num_delta_phases++;
		}
	}
	avg_delta_pitch /= num_delta_phases;

	/**
	 * Calculate the angle
	 * and normalize it to the range [-180, 180]
	 */
	double x = (avg_delta_pitch * LAMBDA) / (2 * M_PI * D);
	if (x > 1.0)
		x = 1.0;
	if (x < -1.0)
		x = -1.0;
	double angle_rad = asinf(x);
	*angle_deg = angle_rad * 180.0 / M_PI;
}

// Returns true if AoA could be calculated, false otherwise
bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report, rot3d_t *angle_deg)
{
	double phases[ANTENNA_PATTERN_LEN];

	if (report->sample_count < 2)
	{
		printk("Not enough valid samples to calculate AoA\n");
		return false;
	}

	double avg_iq[ANTENNA_PATTERN_LEN][2] = {0};

	/**
	 * map each phase to an antenna
	 * the first 8 samples are the refference period
	 */
	for (int i = 0; i < report->sample_count - 1; i++)
	{
		avg_iq[(i - 8) % 16][0] += report->sample[i].i / 2.0;
	}

	/**
	 * Calculate the refference phase
	 * to calculate the base offset for the CTE
	 */
	double ref_phase = 0.0;
	for (int i = 0; i < 8; i++)
	{
		ref_phase += phases[i];
	}
	ref_phase /= 8.0;

	/**
	 * iterate over all samples and calculate the phase for each I, Q pair
	 * and normalize it to the refference phase
	 */
	for (int i = REFFERENCE_SAMPLES; i < report->sample_count; i++)
	{
		int I = avg_iq[i][0];
		int Q = avg_iq[i][1];
		phases[i] = atan2f(Q, I) - ref_phase;
		printk("Antenna %d: I=%d, Q=%d, Phase=%d e-2 rad\n", i, I, Q, (int)(phases[i - 1] * 100));
	}
	double yaw = 0.0;
	calculate_yaw(report->sample_count - REFFERENCE_SAMPLES, phases, &yaw);
	double pitch = 0.0;
	calculate_pitch(phases, &pitch);
	angle_deg->yaw = yaw;
	angle_deg->pitch = pitch;
	return true;
}

void smooth_aoa(double new_yaw, double new_pitch, double *smoothed_yaw, double *smoothed_pitch)
{
	static double yaw_window[AOA_SMOOTH_WINDOW];
	static double pitch_window[AOA_SMOOTH_WINDOW];
	static int idx = 0;
	static int count = 0;

	yaw_window[idx] = new_yaw;
	pitch_window[idx] = new_pitch;
	idx = (idx + 1) % AOA_SMOOTH_WINDOW;
	if (count < AOA_SMOOTH_WINDOW)
		count++;

	double sum_yaw = 0.0, sum_pitch = 0.0;
	for (int i = 0; i < count; i++)
	{
		sum_yaw += yaw_window[i];
		sum_pitch += pitch_window[i];
	}
	*smoothed_yaw = sum_yaw / count;
	*smoothed_pitch = sum_pitch / count;
}

void init_beacon(void)
{
	own_beacon.addr = (bt_addr_le_t){0};
	bt_addr_le_from_str(beacon_adress, "random", &own_beacon.addr);
	own_beacon.write_idx = 0;
	own_beacon.sample_count = 0;
	memset(own_beacon.yaw, 0, sizeof(own_beacon.yaw));
	memset(own_beacon.pitch, 0, sizeof(own_beacon.pitch));
	own_beacon.position.position.x = 0.0;
	own_beacon.position.position.y = 0.0;
	own_beacon.position.position.z = 0.0;
	own_beacon.position.rotation.yaw = 0.0;
	own_beacon.position.rotation.pitch = 0.0;
}
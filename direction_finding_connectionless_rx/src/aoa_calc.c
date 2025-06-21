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

// Returns true if AoA could be calculated, false otherwise
bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report,
				   uint8_t max_antennas, double *angle_deg)
{
	double phases[ANTENNA_PATTERN_LEN];
	int valid_count = 0;

	for (int i = 0; i < max_antennas; i++)
	{
		int idx = i;
		int I = report->sample[idx].i;
		int Q = report->sample[idx].q;
		if (I == 0 && Q == 0)
			continue; // skip zero IQ
		phases[valid_count++] = atan2f(Q, I);
		printk("Antenna %d: I=%d, Q=%d, Phase=%d e-2 rad\n", i, I, Q, (int)(phases[valid_count - 1] * 100));
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

void init_beacon(void)
{
	own_beacon.addr = (bt_addr_le_t){0};
	bt_addr_le_from_str(beacon_adress, "random", &own_beacon.addr);
	own_beacon.write_idx = 0;
	own_beacon.sample_count = 0;
	memset(own_beacon.aoa_samples, 0, sizeof(own_beacon.aoa_samples));
}

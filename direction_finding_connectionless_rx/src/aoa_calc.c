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

double phase_diff_to_angle(double phase_diff)
{
	phase_diff = normalize_angle_180(phase_diff);
	printf("Normaliszed phase_diff: %.3f\n", phase_diff);
	return (phase_diff * LAMBDA) / (2.0 * M_PI * D);
}

// Returns true if AoA could be calculated, false otherwise
bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report, rot3d_t *angle_deg)
{
	for (int i = 0; i < report->sample_count; i++)
	{
		printk("IQ[%d]: %d, %d\n", i, report->sample[i].i, report->sample[i].q);
	}
	double phase_diff = 0.0;
	int ant1_i = 0;
	int and2_i = 0;
	int ant1_q = 0;
	int and2_q = 0;
	double ant1_angle = 0.0;
	double ant2_angle = 0.0;
	int cnt = 0;
	for (int i = REFFERENCE_SAMPLES; i < report->sample_count; i += 2)
	{
		cnt++;
		ant1_i = report->sample[i].i;
		and2_i = report->sample[i + 1].i;
		ant1_q = report->sample[i].q;
		and2_q = report->sample[i + 1].q;
		ant1_angle = atan2f(ant1_q, ant1_i);
		ant2_angle = atan2f(and2_q, and2_i);
		phase_diff += ant2_angle - ant1_angle;
		printf("Sample %d: Ant1 Angle: %.3f, Ant2 Angle: %.3f, Phase Diff: %.3f\n",
			   i, ant1_angle, ant2_angle, phase_diff);
	}
	phase_diff /= cnt;
	printf("Phase difference: %.3f\n", phase_diff);

	/**
	 * calculate the angle in deg
	 */
	angle_deg->pitch = 0.0;
	angle_deg->yaw = phase_diff_to_angle(phase_diff);
	printf("AoA Yaw: %.3f degrees\n", angle_deg->yaw);
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

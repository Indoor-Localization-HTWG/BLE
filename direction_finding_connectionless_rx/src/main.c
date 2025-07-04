/*
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 *  SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include "aoa_calc.h"
#include "ble.h"
#include "est_pos.h"
#include "test/aoa_calc_test.h"

static void aoa_cb(double yaw, double pitch)
{
	own_beacon.yaw[own_beacon.write_idx] = yaw;
	own_beacon.pitch[own_beacon.write_idx] = pitch;
	own_beacon.write_idx = (own_beacon.write_idx + 1) % MAX_AOA_SAMPLES;
	if (own_beacon.sample_count < MAX_AOA_SAMPLES)
		own_beacon.sample_count++;
	double angle_smooth;
	smooth_aoa(yaw, pitch, &angle_smooth);
	printk("Smoothed yaw AoA: %d degrees\n", (int)angle_smooth);
	printk("Smoothed pitch AoA: %d degrees\n", (int)pitch);
}
bool test = false;
int main(void)
{
	if (test)
	{
		aoa_calc_test();
		return 0;
	}

	ble_ctx_t ble;
	memset(&ble, 0, sizeof(ble));
	bt_addr_le_from_str("FA:23:5E:09:F1:39", "random", &ble.own_beacon_addrs);
	ble.aoa_cb = aoa_cb;
	ble_init(&ble);

	point3d_t position = {10.0, 50.0};
	init_beacon();
	init_beacons();

	printk("Starting Connectionless Locator Demo\n");

	printk("Bluetooth initialization...");
	if (bt_enable(NULL) != 0)
	{
		printk("failed\n");
		return 1;
	}
	printk("success\n");

	do
	{
		if (!ble_scan_enable(&ble))
		{
			printk("Scan enable failed\n");
			return 1;
		}

		printk("Waiting for periodic advertising...");
		ble.per_adv_found = false;
		ble_wait_for_adv(&ble);
		printk("success. Found periodic advertising.\n");

		ble.sync_wait = true;
		ble.sync_terminated = false;

		ble_create_sync(&ble);

		printk("Waiting for periodic sync...\n");
		ble_wait_for_sync(&ble);
		printk("success. Periodic sync established.\n");
		ble.sync_wait = false;

		ble_enable_cte_rx(&ble);

		ble_scan_disable(&ble);

		printk("Waiting for periodic sync lost...\n");
		ble_wait_for_sync_lost(&ble);
		printk("Periodic sync lost.\n");
		printk("AoA samples:\n");
		int idx = own_beacon.write_idx;
		int count = own_beacon.sample_count < 5 ? own_beacon.sample_count : 5;
		for (int i = 0; i < 5; i++)
		{
			// Calculate the index of the i-th most recent sample
			int real_idx = (own_beacon.write_idx - count + i + MAX_AOA_SAMPLES) % MAX_AOA_SAMPLES;
			printk("%d, %d\n", (int)own_beacon.yaw[real_idx], (int)own_beacon.pitch[real_idx]);
		}

		printk("Estimating position...\n");
		if (own_beacon.sample_count > 0)
		{
			int last_idx = (own_beacon.write_idx - 1 + MAX_AOA_SAMPLES) % MAX_AOA_SAMPLES;
			beacons[0].yaw[beacons[0].write_idx] = own_beacon.yaw[last_idx];
			beacons[0].pitch[beacons[0].write_idx] = own_beacon.pitch[last_idx];
			beacons[0].write_idx = (beacons[0].write_idx + 1) % MAX_AOA_SAMPLES;
			if (beacons[0].sample_count < MAX_AOA_SAMPLES)
				beacons[0].sample_count++;
		}
		point3d_t new_position;
		// if (estimate_position(position, &new_position))
		// {
		// 	position = new_position;
		// 	printk("Estimated position: x = %d, y = %d, z = %d\n", (int)(position.x * 1000), (int)(position.y * 1000), (int)(position.z * 1000));
		// }
		// else
		// {
		// 	printk("Position estimation failed.\n");
		// }

	} while (true);

	return 0;
}

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/direction.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <math.h>
#include <string.h>
#include "../aoa_calc.h"
#include "aoa_calc_test.h"

#define NUM_SAMPLES 32

void fill_test_iq_report(struct bt_df_per_adv_sync_iq_samples_report *report, double phase_step_rad)
{
    static struct bt_hci_le_iq_sample samples[NUM_SAMPLES];
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        double phi = i * phase_step_rad;
        samples[i].i = (int8_t)(100 * cos(phi));
        samples[i].q = (int8_t)(100 * sin(phi));
    }
    report->sample_count = NUM_SAMPLES;
    report->sample_type = BT_DF_IQ_SAMPLE_8_BITS_INT;
    report->sample = samples; // assign pointer
}

void aoa_calc_test(void)
{
    printk("AoA calculation test start\n");

    struct bt_df_per_adv_sync_iq_samples_report report;
    memset(&report, 0, sizeof(report));

    for (int i = -5; i <= 5; i++)
    {
        double aoa_deg = 18.0 * i;
        double aoa_rad = aoa_deg * M_PI / 180.0;

        // Calculate phase step between antennas for this AoA
        double phase_step_rad = (2.0 * M_PI * D / LAMBDA) * sin(aoa_rad);

        fill_test_iq_report(&report, phase_step_rad);

        rot3d_t angle;
        bool ok = calculate_aoa(&report, &angle);

        if (ok)
        {
            printk("Simulated AoA: %d e-2 deg\n", (int)(aoa_deg * 100));
            printk("Simulated iq samples:\n");
            for (int i = 0; i < NUM_SAMPLES; i++)
            {
                printk("Sample %d: I=%d, Q=%d\n", i, report.sample[i].i, report.sample[i].q);
            }
            printk("Calculated AoA-yaw: %d e-2 deg\n", (int)(angle.yaw * 100));
            printk("Calculated AoA-pitch: %d e-2 deg\n", (int)(angle.pitch * 100));
        }
        else
        {
            printk("AoA calculation failed\n");
        }
    }

    printk("AoA calculation test done\n");
}
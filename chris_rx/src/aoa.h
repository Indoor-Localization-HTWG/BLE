#include <stdlib.h>
#include <stddef.h>
#include <math.h>

/* --- Configuration constants (can be tuned) --- */
#define EST_AOA_FC           2.4e9    /* Carrier frequency (Hz) */
#define EST_AOA_DT           2e-6     /* Antenna-switch delay (s) */
#define EST_AOA_D            0.03     /* Antenna spacing (m) */
#define EST_AOA_SAMPLE_RATE  1e6      /* IQ sampling rate (Hz) */
#define EST_AOA_N_REF        8        /* Number of reference samples */
#define EST_AOA_C            3e8      /* Speed of light (m/s) */

/* --- Error codes --- */
#define EST_AOA_OK           0  /* success */
#define EST_AOA_ERR_NULL    -1  /* null pointer passed */
#define EST_AOA_ERR_LEN     -2  /* not enough samples */
#define EST_AOA_ERR_DIV0    -3  /* division by zero risk */

#ifndef M_PI
#define M_PI 3.14159265358979323846  /* Pi constant */
#endif

/**
 * Estimate AoAs from IQ samples with a fixed reference period.
 *
 * @param iq_i        Array of I-samples, length = total_len
 * @param iq_q        Array of Q-samples, length = total_len
 * @param total_len   Total number of samples in iq_i/iq_q
 * @param aoas        Pre-allocated output array; size >= (total_len - EST_AOA_N_REF)/2
 * @param out_pairs   Pointer to size_t to receive number of AoA pairs computed
 * @param aoa_mean    Pointer to double to receive mean AoA (degrees)
 * @param cfo_est     Pointer to double to receive estimated CFO (Hz)
 *
 * @return EST_AOA_OK on success, or a negative error code.
 */
int estimate_aoa(
    const double *iq_i,
    const double *iq_q,
    size_t total_len,
    double *aoas,
    size_t *out_pairs,
    double *aoa_mean,
    double *cfo_est)
{
    /* --- 1) Sanity checks --- */
    if (!iq_i || !iq_q || !aoas || !out_pairs || !aoa_mean || !cfo_est) {
        return EST_AOA_ERR_NULL;
    }
    if (EST_AOA_SAMPLE_RATE <= 0.0 ||
        EST_AOA_FC <= 0.0 ||
        EST_AOA_D <= 0.0 ||
        EST_AOA_C <= 0.0) {
        return EST_AOA_ERR_DIV0;
    }
    if (total_len <= EST_AOA_N_REF) {
        return EST_AOA_ERR_LEN;
    }

    /* Number of switching samples and antenna-pairs */
    size_t n_sw    = total_len - EST_AOA_N_REF;
    size_t n_pairs = n_sw / 2;
    if (n_pairs == 0) {
        return EST_AOA_ERR_LEN;
    }

    /* --- 2) Calibration from reference period --- */
    double sum_r = 0.0, sum_i = 0.0;
    for (size_t i = 0; i < EST_AOA_N_REF; i++) {
        sum_r += iq_i[i];
        sum_i += iq_q[i];
    }
    /* avoid division by zero */
    if (EST_AOA_N_REF == 0) {
        return EST_AOA_ERR_DIV0;
    }
    double calib_r    = sum_r / EST_AOA_N_REF;
    double calib_i    = sum_i / EST_AOA_N_REF;
    double calib_mag2 = calib_r*calib_r + calib_i*calib_i;
    if (calib_mag2 < 1e-12) {
        return EST_AOA_ERR_DIV0;
    }

    /* --- 3) CFO estimation from reference period --- */
    double Ts      = 1.0 / EST_AOA_SAMPLE_RATE;
    double total_t = 0.0;
    double cfo     = 0.0;
    if (EST_AOA_N_REF > 1) {
        double phi0 = atan2(iq_q[0], iq_i[0]);
        double phi1 = atan2(iq_q[EST_AOA_N_REF-1], iq_i[EST_AOA_N_REF-1]);
        double dphi = phi1 - phi0;
        /* wrap into [-π, π] */
        if (dphi > M_PI)  dphi -= 2*M_PI;
        if (dphi < -M_PI) dphi += 2*M_PI;
        total_t = (EST_AOA_N_REF - 1) * Ts;
        cfo     = dphi / (2.0 * M_PI * total_t);
    }
    *cfo_est = cfo;

    /* --- 4) Precompute antenna-switch phase offset --- */
    double phi_sw = 2.0 * M_PI * EST_AOA_FC * EST_AOA_DT;
    /* wrap into [-π, π] */
    phi_sw = fmod(phi_sw + M_PI, 2.0*M_PI) - M_PI;

    /* --- 5) Compute AoA per Ant0/Ant1 pair --- */
    double lambda = EST_AOA_C / EST_AOA_FC;
    double sum_theta = 0.0;

    for (size_t k = 0; k < n_pairs; k++) {
        size_t idx0 = EST_AOA_N_REF + 2*k;
        size_t idx1 = idx0 + 1;

        /* raw I/Q */
        double r0 = iq_i[idx0], i0 = iq_q[idx0];
        double r1 = iq_i[idx1], i1 = iq_q[idx1];

        /* calibration */
        double d0_r = (r0*calib_r + i0*calib_i) / calib_mag2;
        double d0_i = (i0*calib_r - r0*calib_i) / calib_mag2;
        double d1_r = (r1*calib_r + i1*calib_i) / calib_mag2;
        double d1_i = (i1*calib_r - r1*calib_i) / calib_mag2;

        /* remove CFO ramp */
        double t0   = (2*k)*Ts + total_t;
        double t1   = (2*k+1)*Ts + total_t;
        double p0   = -2.0*M_PI * cfo * t0;
        double p1   = -2.0*M_PI * cfo * t1;
        double c0_r = d0_r*cos(p0) - d0_i*sin(p0);
        double c0_i = d0_r*sin(p0) + d0_i*cos(p0);
        double c1_r = d1_r*cos(p1) - d1_i*sin(p1);
        double c1_i = d1_r*sin(p1) + d1_i*cos(p1);

        /* phase difference */
        double a0   = atan2(c0_i, c0_r);
        double a1   = atan2(c1_i, c1_r);
        double dphi = a1 - a0;
        if (dphi >  M_PI) dphi -= 2*M_PI;
        if (dphi < -M_PI) dphi += 2*M_PI;

        /* correct switch-phase */
        dphi -= phi_sw;
        if (dphi >  M_PI) dphi -= 2*M_PI;
        if (dphi < -M_PI) dphi += 2*M_PI;

        /* to angle */
        double s = (dphi * lambda) / (2.0*M_PI * EST_AOA_D);
        if (s >  1.0) s =  1.0;
        if (s < -1.0) s = -1.0;
        double theta = asin(s) * 180.0 / M_PI;

        aoas[k] = theta;
        sum_theta += theta;
    }

    *aoa_mean  = sum_theta / n_pairs;
    *out_pairs = n_pairs;
    return EST_AOA_OK;
}

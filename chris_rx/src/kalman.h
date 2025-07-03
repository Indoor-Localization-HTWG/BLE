#include <stddef.h>

/* --- Kalman filter tuning (tweak to your needs) --- */
#define KF_PROC_NOISE   0.01   /* Q: process noise variance */
#define KF_MEAS_NOISE   1.00   /* R: measurement noise variance */

/* --- 1-D Kalman filter state --- */
typedef struct {
    double x;   /* state estimate (angle) */
    double P;   /* estimate covariance */
    double Q;   /* process noise covariance */
    double R;   /* measurement noise covariance */
} kalman1d_t;

/**
 * Initialize a 1-D Kalman filter.
 *
 * @param kf        Pointer to your kalman1d_t
 * @param init_x    Initial angle estimate (degrees)
 * @param init_P    Initial covariance (e.g. 1.0)
 */
static void kalman1d_init(kalman1d_t *kf, double init_x, double init_P)
{
    if (!kf) return;
    kf->x = init_x;
    kf->P = init_P;
    kf->Q = KF_PROC_NOISE;
    kf->R = KF_MEAS_NOISE;
}

/**
 * Perform one predict+update step of the 1-D Kalman filter.
 *
 * @param kf    Your filter state
 * @param z     New measurement (angle in degrees)
 * @return      Updated state estimate
 */
static double kalman1d_update(kalman1d_t *kf, double z)
{
    if (!kf) return z;

    /* 1) Predict */
    /* x_prior = x (no dynamics, constant model) */
    /* P_prior = P + Q */
    double P_prior = kf->P + kf->Q;

    /* 2) Update */
    /* K = P_prior / (P_prior + R) */
    double K = P_prior / (P_prior + kf->R);

    /* x_post = x_prior + K*(z − x_prior) */
    kf->x = kf->x + K * (z - kf->x);

    /* P_post = (1 − K)*P_prior */
    kf->P = (1.0 - K) * P_prior;

    return kf->x;
}

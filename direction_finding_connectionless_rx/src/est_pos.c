
#include "est_pos.h"

#include <math.h>

struct beacon_data beacons[NUM_BEACONS];

void distance_to_plane(point3d_t p, point3d_t beacon, double yaw_deg, double pitch_deg, double *dist)
{
    double az = deg2rad(yaw_deg);
    double el = deg2rad(pitch_deg);

    // Normal vector from yaw/pitch
    double nx = cos(el) * cos(az);
    double ny = cos(el) * sin(az);
    double nz = sin(el);

    // Vector from beacon to point
    double dx = p.x - beacon.x;
    double dy = p.y - beacon.y;
    double dz = p.z - beacon.z;

    // Distance from point to plane
    *dist = dx * nx + dy * ny + dz * nz;
}

void total_cost(point3d_t p, double *cost)
{
    *cost = 0.0f;
    for (int i = 0; i < NUM_BEACONS; i++)
    {
        if (beacons[i].sample_count == 0)
        {
            continue;
        }
        int n = beacons[i].sample_count < 3 ? beacons[i].sample_count : 3;
        double sum_yaw = 0.0f;
        double sum_pitch = 0.0f;
        for (int j = 0; j < n; j++)
        {
            int idx = (beacons[i].write_idx - n + j + MAX_AOA_SAMPLES) % MAX_AOA_SAMPLES;
            sum_yaw += beacons[i].yaw[idx];
            sum_pitch += beacons[i].pitch[idx];
        }
        double avg_yaw = sum_yaw / n;
        double avg_pitch = sum_pitch / n;
        double dist;
        distance_to_plane(
            p,
            beacons[i].position.position,
            avg_yaw,
            avg_pitch,
            &dist);
        *cost += dist * dist;
    }
}

void compute_gradient(point3d_t p, point3d_t *grad)
{
    double h = 0.001f;
    point3d_t p_x_plus = {p.x + h, p.y, p.z};
    point3d_t p_x_minus = {p.x - h, p.y, p.z};
    point3d_t p_y_plus = {p.x, p.y + h, p.z};
    point3d_t p_y_minus = {p.x, p.y - h, p.z};
    point3d_t p_z_plus = {p.x, p.y, p.z + h};
    point3d_t p_z_minus = {p.x, p.y, p.z - h};

    double cost_x_plus, cost_x_minus, cost_y_plus, cost_y_minus, cost_z_plus, cost_z_minus;
    total_cost(p_x_plus, &cost_x_plus);
    total_cost(p_x_minus, &cost_x_minus);
    total_cost(p_y_plus, &cost_y_plus);
    total_cost(p_y_minus, &cost_y_minus);
    total_cost(p_z_plus, &cost_z_plus);
    total_cost(p_z_minus, &cost_z_minus);

    grad->x = (cost_x_plus - cost_x_minus) / (2.0f * h);
    grad->y = (cost_y_plus - cost_y_minus) / (2.0f * h);
    grad->z = (cost_z_plus - cost_z_minus) / (2.0f * h);
}

bool gradient_descent(point3d_t prev_xyz, point3d_t *est_xyz)
{
    double learning_rate = 0.08f;
    int max_iterations = 50;
    double epsilon = 1e-3f;
    int i = 0;
    point3d_t grad;
    double norm = INFINITY;
    point3d_t p = prev_xyz;

    // Check if enough data is available
    for (int i = 0; i < NUM_BEACONS; i++)
    {
        if (beacons[i].sample_count < 3)
        {
            printk("Not enough AoA-data for Beacon %d\n", i);
            *est_xyz = prev_xyz;
            return false;
        }
    }

    do
    {
        compute_gradient(p, &grad);
        norm = sqrtf(grad.x * grad.x + grad.y * grad.y + grad.z * grad.z);
        p.x -= learning_rate * grad.x;
        p.y -= learning_rate * grad.y;
        p.z -= learning_rate * grad.z;
    } while (++i < max_iterations && norm > epsilon);

    est_xyz->x = p.x;
    est_xyz->y = p.y;
    est_xyz->z = p.z;
    return true;
}

bool estimate_position(point3d_t prev_xy, point3d_t *est_xy)
{
    return gradient_descent(prev_xy, est_xy);
}

void init_beacons(void)
{
    for (int i = 0; i < NUM_BEACONS; i++)
    {
        bt_addr_le_from_str("FA:23:5E:09:F1:39", "random", &beacons[i].addr);
        beacons[i].position = (beacon_t){{0.0, 0.0, 0.0}, 0, 0};
        beacons[i].sample_count = 0;
        beacons[i].write_idx = 0;
        memset(beacons[i].yaw, 0, sizeof(beacons[i].yaw));
        memset(beacons[i].pitch, 0, sizeof(beacons[i].pitch));
    }
}
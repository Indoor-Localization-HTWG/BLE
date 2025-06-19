
#include "est_pos.h"

#include <math.h>

struct beacon_data beacons[NUM_BEACONS];

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
            sum_angle += beacons[i].aoa_samples[j];
        }
        double avg_angle = sum_angle / n;
        double dist;
        distance_to_line(
            p,
            beacons[i].position.position,
            avg_angle,
            beacons[i].position.angle,
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
            printk("Not enough AoA-data for Beacon %d\n", i);
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

void init_beacons(void)
{
    for (int i = 0; i < NUM_BEACONS; i++)
    {
        bt_addr_le_from_str("FA:23:5E:09:F1:39", "random", &beacons[i].addr);
        beacons[i].position = (beacon_t){{0.0f, 0.0f}, 0};
        beacons[i].sample_count = 0;
    }
}
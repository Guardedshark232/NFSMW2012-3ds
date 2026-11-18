#include "physics.h"
#include <algorithm>
#include <cmath>

static CarPhysicsState g_car;

// Gear top speeds (km/h) for 2012 Porsche 911 Carrera S PDK
static const float kGearMaxSpeed[6] = { 65.0f, 110.0f, 160.0f, 215.0f, 265.0f, 310.0f };

void physics_init(void) {
    g_car.speed = 0.0f;
    g_car.top_speed = 295.0f;
    g_car.acceleration = 75.0f;
    g_car.braking = 120.0f;
    g_car.steering_angle = 0.0f;
    g_car.distance = 0.0f;
    // Spawn on the harbour street at (-148,164), facing southwest up the
    // quay road: buildings and containers on both sides, continuous ground
    // for well over 100 m, and open sky above the skyline.
    g_car.heading = 0.7853982f; // 45 degrees
    g_car.pos_x = -148.0f;
    g_car.pos_y = 0.0f;
    g_car.pos_z = 164.0f;
    g_car.nitrous_active = false;
    g_car.nitrous_level = 1.0f;
    g_car.gear = 1;
    g_car.rpm = 1000.0f;
    g_car.is_drifting = false;
    g_car.drift_angle = 0.0f;
}

void physics_update(float dt, float throttle, float brake, float steering, bool nitrous, bool handbrake) {
    if (dt <= 0.0f) return;
    if (dt > 0.05f) dt = 0.016f;

    // Nitrous depletion and activation
    if (nitrous && g_car.nitrous_level > 0.05f && throttle > 0.0f && g_car.speed > 10.0f) {
        g_car.nitrous_active = true;
        g_car.nitrous_level -= dt * 0.25f; // Lasts ~4 seconds
        if (g_car.nitrous_level < 0.0f) g_car.nitrous_level = 0.0f;
    } else {
        g_car.nitrous_active = false;
        // Passive nitrous regeneration (faster when driving at high speed or drifting)
        float regen_rate = 0.04f;
        if (g_car.is_drifting) regen_rate = 0.15f;
        g_car.nitrous_level = std::min(1.0f, g_car.nitrous_level + dt * regen_rate);
    }

    // Dynamic top speed and acceleration
    float max_speed = g_car.top_speed * (g_car.nitrous_active ? 1.25f : 1.0f);
    float current_accel = g_car.acceleration * (g_car.nitrous_active ? 1.6f : 1.0f);

    // Throttle & Braking
    if (throttle > 0.0f) {
        // Power curve falls off as approaching top speed
        float speed_ratio = g_car.speed / max_speed;
        float power = std::max(0.15f, 1.0f - (speed_ratio * speed_ratio));
        g_car.speed += current_accel * throttle * power * dt;
    } else if (brake > 0.0f) {
        g_car.speed -= g_car.braking * brake * dt;
    } else {
        // Natural rolling resistance and aerodynamic drag
        g_car.speed -= (8.0f + 0.0015f * g_car.speed * g_car.speed) * dt;
    }

    // Handbrake extra braking and slip
    if (handbrake) {
        g_car.speed -= g_car.braking * 0.7f * dt;
    }

    if (g_car.speed < 0.0f) g_car.speed = 0.0f;
    if (g_car.speed > max_speed) g_car.speed = max_speed;

    // Drift Detection (Criterion style: brake tap or handbrake while turning)
    bool initiating_drift = (std::abs(steering) > 0.35f) && (handbrake || brake > 0.2f) && (g_car.speed > 40.0f);
    if (initiating_drift) {
        g_car.is_drifting = true;
    } else if (std::abs(steering) < 0.1f || g_car.speed < 25.0f) {
        g_car.is_drifting = false;
    }

    // Steering and Drift slip calculation
    float steer_speed_factor = std::max(0.2f, 1.0f - (g_car.speed / (g_car.top_speed * 1.4f)));
    float target_steer = steering * steer_speed_factor;

    // Smooth steering response
    g_car.steering_angle += (target_steer - g_car.steering_angle) * (15.0f * dt);

    float turn_rate = 1.8f;
    if (g_car.is_drifting) {
        turn_rate = 3.2f;
        float target_slip = -steering * 0.45f;
        g_car.drift_angle += (target_slip - g_car.drift_angle) * (10.0f * dt);
    } else {
        g_car.drift_angle += (0.0f - g_car.drift_angle) * (12.0f * dt);
    }

    g_car.heading -= g_car.steering_angle * (g_car.speed / 100.0f) * dt * turn_rate;

    // Position updates
    float speed_ms = g_car.speed / 3.6f;
    g_car.distance += speed_ms * dt;

    // Forward and lateral motion
    // heading 0 faces -Z; positive heading turns left.
    float fwd_x = -sinf(g_car.heading);
    float fwd_z = -cosf(g_car.heading);
    float right_x = cosf(g_car.heading);
    float right_z = -sinf(g_car.heading);

    g_car.pos_x += fwd_x * speed_ms * dt;
    g_car.pos_z += fwd_z * speed_ms * dt;

    // Rear-end slide while drifting (velocity points inside the nose).
    float lateral = g_car.drift_angle * speed_ms * 0.6f;
    g_car.pos_x += right_x * lateral * dt;
    g_car.pos_z += right_z * lateral * dt;

    // Full open world: no hard boundary. The streaming engine loads and
    // unloads sectors as the car moves; the world is 4.4 km x 7.3 km.

    // Gear & RPM calculations
    int new_gear = 1;
    for (int g = 0; g < 6; g++) {
        if (g_car.speed >= kGearMaxSpeed[g] * 0.85f && g < 5) {
            new_gear = g + 2;
        }
    }
    g_car.gear = new_gear;

    float prev_max = (g_car.gear == 1) ? 0.0f : kGearMaxSpeed[g_car.gear - 2];
    float gear_max = kGearMaxSpeed[g_car.gear - 1];
    float gear_ratio = (g_car.speed - prev_max) / (gear_max - prev_max);
    if (gear_ratio < 0.0f) gear_ratio = 0.0f;
    if (gear_ratio > 1.0f) gear_ratio = 1.0f;

    g_car.rpm = 1500.0f + gear_ratio * 6100.0f; // 1500 to 7600 RPM
    if (g_car.speed < 1.0f) g_car.rpm = 1000.0f;
}

void physics_set_ground_height(float y) {
    g_car.pos_y = y;
}

void physics_stop_at(float x, float z) {
    g_car.pos_x = x;
    g_car.pos_z = z;
    g_car.speed = 0.0f;
}

const CarPhysicsState* physics_get_state(void) {
    return &g_car;
}

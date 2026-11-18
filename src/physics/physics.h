#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdbool.h>

typedef struct {
    float speed;            // Current speed in km/h
    float top_speed;        // Max speed without nitrous (~290 km/h for Porsche 911)
    float acceleration;     // Acceleration power
    float braking;          // Braking force
    float steering_angle;   // Steering wheel angle (-1.0 left to 1.0 right)
    float distance;         // Total distance traveled (meters)
    float heading;          // Heading yaw angle (radians)
    float pos_x;            // World X position (metres)
    float pos_y;            // Ground height (metres)
    float pos_z;            // World Z position (metres)
    bool nitrous_active;    // Nitrous currently boosting
    float nitrous_level;    // Nitrous fuel level (0.0 to 1.0)
    int gear;               // Transmission gear (1 to 6)
    float rpm;              // Engine RPM (1000 to 7800)
    bool is_drifting;       // Car is in power-slide / drift
    float drift_angle;      // Slip angle in drift
} CarPhysicsState;

void physics_init(void);
void physics_update(float dt, float throttle, float brake, float steering, bool nitrous, bool handbrake);
void physics_set_ground_height(float y);
void physics_stop_at(float x, float z);
const CarPhysicsState* physics_get_state(void);

#endif // PHYSICS_H

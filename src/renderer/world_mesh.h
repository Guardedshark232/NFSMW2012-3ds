#ifndef WORLD_MESH_H
#define WORLD_MESH_H

#include <3ds.h>
#include <citro3d.h>

void world_mesh_init(void);
// Raise draw distance and triangle/chunk budgets on New 3DS-class hardware.
void world_mesh_set_quality(bool high);
void world_mesh_render(const C3D_Mtx* projection, const C3D_Mtx* view,
                       float car_x, float car_z, float forward_x, float forward_z);
// Highest drivable surface at (x, z) that is within GROUND_STEP_UP of ref_y.
float world_mesh_ground_height(float x, float z, float ref_y, float fallback);
// Move a collision circle from (x, z) toward (target_x, target_z).  The
// position is updated in place and the lost fraction of the requested motion
// is returned (0 = clear, 1 = fully blocked); walls are slid along.
float world_mesh_move(float* x, float* z, float target_x, float target_z,
                      float feet_y, float radius);
int world_mesh_chunk_count(void);
int world_mesh_chunk_total(void);
int world_mesh_ground_cell_count(void);
void world_mesh_write_diag(void);
void world_mesh_exit(void);

#endif // WORLD_MESH_H

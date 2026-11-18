#ifndef WORLD_MESH_H
#define WORLD_MESH_H

#include <3ds.h>
#include <citro3d.h>

void world_mesh_init(void);
void world_mesh_render(const C3D_Mtx* projection, const C3D_Mtx* view,
                       float car_x, float car_z, float forward_x, float forward_z);
float world_mesh_ground_height(float x, float z, float fallback);
bool world_mesh_sweep_blocked(float x0, float z0, float y, float x1, float z1);
int world_mesh_chunk_count(void);
int world_mesh_chunk_total(void);
int world_mesh_ground_cell_count(void);
void world_mesh_write_diag(void);
void world_mesh_exit(void);

#endif // WORLD_MESH_H

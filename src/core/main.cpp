#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>
#include <stdio.h>
#include <string.h>
#include "input.h"
#include "renderer.h"
#include "physics.h"
#include "bndl.h"
#include "world_mesh.h"

// Define to build the automated driving verification (holds throttle and
// logs telemetry to sdmc:/drive_diag.txt).  Off for normal builds.
#define DIAG_AUTO_DRIVE 0

int main(int argc, char **argv) {
    Result rc = romfsInit();
    bool romfs_ok = R_SUCCEEDED(rc);

    cfguInit();
    renderer_init();
    input_init();
    physics_init();

    // Inspect & Decompress BNDL from RomFS
    BNDLArchive worldBndl;
    bool bndl_opened = false;
    uint32_t bndl_entries = 0;

    if (romfs_ok) {
        bndl_opened = worldBndl.open("romfs:/WORLDLIST.BNDL");
        if (bndl_opened) {
            bndl_entries = worldBndl.getNumEntries();
        }
    }

    u64 lastTick = osGetTime();
    u32 frameCount = 0;
    float fps = 60.0f;
    u64 fpsTimer = osGetTime();

    bool easyDriveOpen = false;
    int easyDriveSelection = 0;

    while (aptMainLoop()) {
        u64 currentTick = osGetTime();
        float dt = (float)(currentTick - lastTick) / 1000.0f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.016f;
        lastTick = currentTick;

        frameCount++;
        if (currentTick - fpsTimer >= 500) {
            fps = (float)frameCount / ((float)(currentTick - fpsTimer) / 1000.0f);
            frameCount = 0;
            fpsTimer = currentTick;
        }

        input_update();

        // EasyDrive toggle & navigation
        static bool lastStart = false;
        bool startDown = input_is_pressed(ACT_EASY_DRIVE);
        if (startDown && !lastStart) {
            easyDriveOpen = !easyDriveOpen;
        }
        lastStart = startDown;

        if (easyDriveOpen) {
            static bool lastBrake = false;
            bool brakeDown = input_is_pressed(ACT_BRAKE);
            if (brakeDown && !lastBrake) {
                easyDriveSelection = (easyDriveSelection + 1) % 4;
            }
            lastBrake = brakeDown;
        }

        // Reset car
        if (input_is_pressed(ACT_RESET_CAR)) {
            physics_init();
            renderer_reset_view();
        }

        bool drivingInput = !easyDriveOpen;
        float throttle = drivingInput && input_is_pressed(ACT_ACCELERATE) ? 1.0f : 0.0f;
        float brake = drivingInput && input_is_pressed(ACT_BRAKE) ? 1.0f : 0.0f;
        bool nitrous = drivingInput && input_is_pressed(ACT_NITROUS);
        bool handbrake = drivingInput && input_is_pressed(ACT_HANDBRAKE);
        float steering = drivingInput ? input_get_stick_x() : 0.0f;

#if DIAG_AUTO_DRIVE
        // Automated driving verification: full throttle from frame 120 to
        // 900 with a gentle right steer, logging telemetry to the SD card.
        static int diag_frame = 0;
        diag_frame++;
        if (diag_frame > 120 && diag_frame < 900) throttle = 1.0f;
        if (diag_frame > 300 && diag_frame < 500) steering = 0.25f;
        if (diag_frame % 60 == 0) {
            static FILE* df = NULL;
            if (!df) df = fopen("sdmc:/drive_diag.txt", "w");
            if (df) {
                const CarPhysicsState* st = physics_get_state();
                fprintf(df, "f %4d spd %6.1f pos %7.2f %7.2f gnd %7.2f cam %7.2f dist %7.1f gear %d\n",
                        diag_frame, st->speed, st->pos_x, st->pos_z,
                        renderer_debug_car_y(), renderer_debug_cam_y(),
                        st->distance, st->gear);
                fflush(df);
            }
        }
#endif

        const CarPhysicsState* before_drive = physics_get_state();
        float old_x = before_drive->pos_x, old_y = before_drive->pos_y, old_z = before_drive->pos_z;
        physics_update(dt, throttle, brake, steering, nitrous, handbrake);
        const CarPhysicsState* after_drive = physics_get_state();
        if (world_mesh_sweep_blocked(old_x, old_z, old_y,
                                     after_drive->pos_x, after_drive->pos_z))
            physics_stop_at(old_x, old_z);

        // Render Frame
        renderer_begin_frame();
        renderer_render_world_and_car(dt);
        renderer_render_hud(easyDriveOpen, easyDriveSelection, bndl_opened, bndl_entries, fps);
        renderer_end_frame();
    }

    worldBndl.close();
    if (romfs_ok) romfsExit();
    renderer_exit();
    cfguExit();
    return 0;
}

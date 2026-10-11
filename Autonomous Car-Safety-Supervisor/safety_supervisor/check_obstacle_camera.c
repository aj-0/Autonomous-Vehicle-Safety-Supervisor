/*
 * check_obstacle_camera.c - is the command safe in the current world?
 *
 *  - collision guard from physics: can the vehicle still stop before the obstacle?
 *  - camera (nav's eyes) cross-checked against the ultrasonic (supervisor's eyes):
 *      CAMERA_MISS : ultrasonic sees an obstacle ahead, camera reports nothing
 *      PHANTOM     : camera reports a close obstacle, ultrasonic sees nothing
 *  - perception flicker: obstacle flag toggling within a short window
 */
#include "checks.h"
#include "decision_core.h"
#include "vehicle_physics.h"

float chk_allowed_speed(const sup_ctx_t *c, float decel)
{
    if (!c->obstacle_in_range)
        return 1e9f;
    return physics_allowed_speed(c->dist_m - c->P->margin_m, c->P->t_sys_s, decel);
}

void chk_situation_on_sensor(sup_ctx_t *c)
{
    const av_profile_t *P = c->P;

    /* Even maximum braking is barely enough -> emergency stop. */
    float v_ok_max = chk_allowed_speed(c, P->a_max);
    sup_cond(c, C_COLLISION_IMMINENT,
             c->obstacle_in_range && c->v_valid && c->v_true > v_ok_max + P->speed_tol);

    sup_cond(c, C_PATH_BLOCKED, c->obstacle_in_range && c->dist_m < P->recovery_clear_m);

    if (c->v_valid && c->v_true < P->standstill_v)
        sup_cond(c, C_BLIND_COLLISION_RISK, 0);       /* risk resolved by standing still */
}

void chk_perception(sup_ctx_t *c, uint64_t now)
{
    const av_profile_t *P = c->P;

    /* ---- flicker: count obstacle-flag transitions inside the window ---- */
    uint8_t flag = c->perc.obstacle ? 1 : 0;
    if (flag != c->perc_last_flag) {
        c->flick_t[c->flick_idx % SUP_HIST] = now;
        c->flick_idx++;
        c->perc_last_flag = flag;
    }
    int n = 0;
    uint64_t last = 0;
    for (int i = 0; i < SUP_HIST; i++) {
        uint64_t t = c->flick_t[i];
        if (t && now - t <= (uint64_t)AV_FLICKER_WINDOW_MS * 1000u)
            n++;
        if (t > last)
            last = t;
    }
    if (n >= AV_FLICKER_TRANSITIONS)
        c->perc_unstable = 1;
    else if (c->perc_unstable && now - last > (uint64_t)AV_FLICKER_CLEAR_MS * 1000u)
        c->perc_unstable = 0;
    sup_cond(c, C_PERC_UNSTABLE, c->perc_unstable);

    /* ---- cross-check with the independent ultrasonic (straight-ahead cone only) ---- */
    if (!c->us_ok)
        return;
    int cam_center = c->perc.obstacle && c->perc.zone == ZONE_CENTER;
    int us_close = c->obstacle_in_range && c->dist_m <= P->xcheck_max_m;

    c->miss_frames = (us_close && !cam_center) ? c->miss_frames + 1 : 0;

    int phantom = cam_center && c->perc.cam_dist_m <= P->xcheck_max_m &&
                  (!c->obstacle_in_range || c->dist_m > c->perc.cam_dist_m + P->phantom_tol_m);
    c->phantom_frames = phantom ? c->phantom_frames + 1 : 0;

    sup_cond(c, C_CAMERA_MISS, c->miss_frames >= AV_XCHECK_FRAMES);
    sup_cond(c, C_PHANTOM, c->phantom_frames >= AV_XCHECK_FRAMES);
}

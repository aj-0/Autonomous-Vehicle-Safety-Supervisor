/*
 * messages.h - every message, pulse code and enum shared by the AV Safety Supervisor processes.
 *
 * Portable C (only <stdint.h>) so the supervisor core can also be unit-tested on a host PC.
 * All structures are fixed-size and sent with QNX MsgSend()/MsgReply().
 */
#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

#if defined(__QNXNTO__)
#include <sys/neutrino.h>       /* _PULSE_CODE_MINAVAIL */
#endif
#ifndef _PULSE_CODE_MINAVAIL
#define _PULSE_CODE_MINAVAIL 0
#endif

/* ------------------------------------------------------------------ */
/* Registered names (name_attach / name_open)                          */
/* ------------------------------------------------------------------ */
#define AV_NAME_SUPERVISOR "av_supervisor"
#define AV_NAME_LINK       "av_vehicle_link"      /* vehicle_link (real) or vehicle_simulator (Stage 0) */
#define AV_NAME_RECOVERY   "av_recovery"
#define AV_NAME_LOGGER     "av_logger"
#define AV_NAME_NAV        "av_navigation"

#define AV_PROTO_VERSION 1

/* ------------------------------------------------------------------ */
/* Pulse codes                                                         */
/* ------------------------------------------------------------------ */
#define AV_PULSE_TICK         (_PULSE_CODE_MINAVAIL + 1)  /* periodic timer                    */
#define AV_PULSE_NAV_HB       (_PULSE_CODE_MINAVAIL + 2)  /* nav heartbeat, value = nav seq    */
#define AV_PULSE_LOG_KICK     (_PULSE_CODE_MINAVAIL + 3)  /* logger: new events in the ring   */
#define AV_PULSE_RECOV_START  (_PULSE_CODE_MINAVAIL + 4)  /* value = start speed in mm/s       */
#define AV_PULSE_RECOV_STOP   (_PULSE_CODE_MINAVAIL + 5)
#define AV_PULSE_RESTART_NAV  (_PULSE_CODE_MINAVAIL + 6)  /* value = pid of old nav (0 = none) */

/* ------------------------------------------------------------------ */
/* Message types - well above the QNX reserved I/O range (0x000-0x1FF) */
/* ------------------------------------------------------------------ */
#define AV_MSG_BASE 0x4000
enum {
    AV_MSG_HELLO = AV_MSG_BASE + 1,
    AV_MSG_NAV_CMD,
    AV_MSG_SENSOR_STATE,
    AV_MSG_PERCEPTION,
    AV_MSG_RECOVERY_CMD,
    AV_MSG_CLI,
    AV_MSG_VEHICLE_CMD,
    AV_MSG_NAV_CTRL,
    AV_MSG_SIM_CTRL,
};

typedef struct {
    uint16_t type;
    uint16_t version;
} av_hdr_t;

/* ------------------------------------------------------------------ */
/* Enumerations                                                        */
/* ------------------------------------------------------------------ */

/* Roles a client claims with AV_MSG_HELLO. The server only accepts a
 * message type from the connection that owns the matching role. */
enum {
    AV_ROLE_NAV        = 1u << 0,
    AV_ROLE_RECOVERY   = 1u << 1,
    AV_ROLE_SENSOR     = 1u << 2,
    AV_ROLE_PERCEPTION = 1u << 3,
    AV_ROLE_CLI        = 1u << 4,
    AV_ROLE_SUPERVISOR = 1u << 5,   /* claimed at av_vehicle_link: only it may drive the vehicle */
};
#define AV_ROLES_EXCLUSIVE (AV_ROLE_NAV | AV_ROLE_RECOVERY | AV_ROLE_SENSOR | \
                             AV_ROLE_PERCEPTION | AV_ROLE_SUPERVISOR)

typedef enum {
    ST_INIT = 0,
    ST_NORMAL,
    ST_CAUTION,      /* perception doubtful: speed cap, no swerving         */
    ST_DEGRADED,     /* a sensor isolated: limp mode                         */
    ST_MRM,          /* minimal risk manoeuvre: controlled stop by recovery  */
    ST_ESTOP,        /* emergency stop: maximum braking                      */
    ST_SAFE,         /* stopped and latched                                  */
    ST_RECOVERY,     /* conditions clear, waiting for operator ack           */
    ST__COUNT
} av_state_t;

enum { V_PASS = 0, V_CLAMP, V_REJECT };

enum { AUTH_NONE = 0, AUTH_NAV, AUTH_RECOVERY };

enum { BRAKE_NORMAL = 0, BRAKE_COMFORT, BRAKE_MAX };

enum { ZONE_NONE = 0, ZONE_LEFT, ZONE_CENTER, ZONE_RIGHT };

enum { SENS_ENC_L = 0, SENS_ENC_R, SENS_US, SENS_IMU, SENS_COUNT };

enum { H_OK = 0, H_DEAD, H_FROZEN, H_RANGE, H_JUMP, H_UNKNOWN };

/* Verdict reasons (bit mask) */
enum {
    R_RANGE         = 1u << 0,
    R_CORRUPT       = 1u << 1,
    R_SLEW          = 1u << 2,
    R_ENVELOPE      = 1u << 3,
    R_SWERVE_LOCK   = 1u << 4,
    R_STATE_CAP     = 1u << 5,
    R_COLLISION_CAP = 1u << 6,
    R_CAMERA_MISS   = 1u << 7,
    R_NO_AUTHORITY  = 1u << 8,
};

/* Conditions (bit positions) - each one maps to a severity in safety_states.c */
enum {
    C_PERC_UNSTABLE = 0,    /* camera flicker                                   */
    C_PHANTOM,              /* camera sees obstacle, ultrasonic does not        */
    C_CAMERA_MISS,          /* ultrasonic sees obstacle, camera does not        */
    C_PERC_STALE,           /* no fresh perception > 100 ms                     */
    C_PERC_LOST,            /* no fresh perception > 1 s                        */
    C_NAV_SPEED_MISMATCH,   /* nav speed estimate disagrees with encoders       */
    C_IMU_FAULT,
    C_ENC_FAULT_ONE,
    C_ENC_FAULT_BOTH,
    C_US_FAULT,
    C_YAW_IMU_SUSPECT,      /* 2oo3 vote: IMU is the odd one out                */
    C_YAW_ENC_SUSPECT,      /* 2oo3 vote: encoders are the odd one out          */
    C_ACTUATOR_FAULT,       /* 2oo3 vote: vehicle does not follow the command   */
    C_MULTI_FAULT,
    C_SENSOR_LINK_LOST,     /* no sensor frames > 30 ms                         */
    C_OUTPUT_LINK_FAIL,     /* cannot deliver commands to the vehicle           */
    C_NAV_HB_LOST,          /* nav alive but silent (hang)                      */
    C_NAV_CRASHED,          /* nav process gone (kernel disconnect pulse)       */
    C_NAV_UNTRUSTED,        /* too many rejected commands                       */
    C_BLIND_COLLISION_RISK, /* nav drives at an obstacle it does not see        */
    C_COLLISION_IMMINENT,   /* even max braking barely avoids the obstacle      */
    C_BRAKE_INEFFECTIVE,    /* vehicle not slowing although braking             */
    C_ISLAND_TRIPPED,       /* STM32 safety island stopped the vehicle itself   */
    C_ESTOP_BUTTON,
    C_PATH_BLOCKED,         /* obstacle too close to allow recovery             */
    C__COUNT
};
#define CBIT(c) (1u << (c))

typedef enum {
    FAULT_NONE = 0,
    /* navigation software (navigation) */
    FAULT_NAV_SLEW, FAULT_NAV_ENVELOPE, FAULT_NAV_SPEEDEST, FAULT_NAV_HANG, FAULT_NAV_CRASH,
    FAULT_NAV_CORRUPT,
    /* camera / AI (vehicle_simulator now, perception_cam.py later) */
    FAULT_CAM_FLICKER, FAULT_CAM_MISS, FAULT_CAM_PHANTOM, FAULT_CAM_DEAD, FAULT_CAM_FROZEN,
    FAULT_CAM_LAG,
    /* sensors and vehicle (vehicle_simulator now, real hardware later) */
    FAULT_IMU_FROZEN, FAULT_IMU_DEAD, FAULT_ENC_L_DEAD, FAULT_US_DEAD, FAULT_STEER_STUCK,
    FAULT_BRAKE_FAIL, FAULT_LINK_DROP,
    FAULT__COUNT
} av_fault_t;

/* ------------------------------------------------------------------ */
/* Messages                                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    av_hdr_t hdr;              /* AV_MSG_HELLO */
    uint32_t  roles;
    int32_t   pid;
    char      name[16];
} av_hello_t;

/* nav_task -> supervisor. nav stays REPLY-blocked until the verdict arrives. */
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_NAV_CMD */
    uint32_t  seq;
    uint64_t  t_us;
    float     target_speed;     /* m/s                                    */
    float     steer_deg;        /* virtual steering angle, + = left       */
    float     nav_speed_est;    /* what nav believes the speed is (m/s)   */
    uint8_t   nav_obstacle;     /* nav believes an obstacle is ahead      */
    uint8_t   nav_zone;
    uint8_t   pad[2];
} av_nav_cmd_t;

/* supervisor -> nav / recovery (MsgReply) */
typedef struct {
    uint32_t seq;
    uint8_t  verdict;           /* V_PASS / V_CLAMP / V_REJECT */
    uint8_t  state;
    uint8_t  authority;
    uint8_t  pad;
    uint32_t reasons;           /* R_* mask */
    float    out_speed;
    float    out_steer;
} av_verdict_t;

/* recovery_manager -> supervisor (accepted only while in MRM) */
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_RECOVERY_CMD */
    uint32_t  seq;
    float     speed;
    float     steer_deg;
} av_recovery_cmd_t;

/* sensor_monitor (or vehicle_simulator) -> supervisor, every 10 ms */
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_SENSOR_STATE */
    uint32_t  seq;
    uint64_t  t_us;
    float     v_left, v_right;  /* encoder wheel speeds, m/s        */
    float     distance_m;       /* ultrasonic                       */
    float     gyro_z_dps;       /* IMU yaw rate, + = left           */
    float     accel_x, accel_y; /* IMU, m/s^2                       */
    uint8_t   health[SENS_COUNT];
    uint8_t   us_no_echo;       /* healthy sensor, nothing in range */
    uint8_t   island_state;     /* 0 = ok, 1 = tripped              */
    uint8_t   estop;            /* E-stop button pressed            */
    uint8_t   relay_closed;
    uint16_t  link_errors;
    uint16_t  pad;
    uint32_t  hw_latency_us;    /* STM32 hardware stopwatch         */
} av_sensor_state_t;

/* perception gateway (or vehicle_simulator) -> supervisor and nav, ~30 Hz */
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_PERCEPTION */
    uint32_t  seq;
    uint64_t  t_capture_us;
    uint8_t   obstacle;
    uint8_t   zone;             /* ZONE_*                                  */
    uint8_t   confidence;       /* 0..100                                  */
    uint8_t   class_id;         /* COCO class of the detected object       */
    float     cam_dist_m;       /* rough distance from bounding-box size   */
    float     odom_speed;       /* nav's own (visual) odometry speed       */
    float     fps;
} av_perception_t;

/* supervisor -> vehicle link (STM32) / vehicle_simulator, every 10 ms */
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_VEHICLE_CMD */
    uint32_t  seq;
    uint64_t  t_us;
    float     speed_sp;
    float     steer_deg;
    uint8_t   brake_mode;       /* BRAKE_* */
    uint8_t   state;
    uint8_t   motor_enable;     /* 0 -> island opens the motor relay */
    uint8_t   pad;
} av_vehicle_cmd_t;

/* CLI -> supervisor */
enum { CLI_STATUS = 1, CLI_ACK, CLI_INJECT_MARK, CLI_RESET_STATS };
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_CLI */
    uint32_t  cmd;
    uint32_t  arg;
} av_cli_req_t;

typedef struct {
    uint8_t  state, authority, last_verdict, ack_result;   /* ack_result: 0 n/a, 1 accepted, 2 not ready */
    uint32_t conds;
    uint32_t last_reasons;
    char     profile[8];
    /* vehicle */
    float    v_true, dist_m, gyro_dps, out_speed, out_steer;
    uint8_t  health[SENS_COUNT];
    uint8_t  obstacle_in_range, island_tripped, estop, bench;
    /* perception */
    uint8_t  perc_obstacle, perc_zone, perc_unstable, perc_seen;
    uint32_t perc_age_ms;
    /* navigation */
    uint8_t  nav_connected, pad[3];
    int32_t  nav_pid;
    uint32_t nav_hb_age_ms;
    uint32_t nav_rejects_window;
    uint32_t pass_count, clamp_count, reject_count;
    /* timing */
    uint32_t ticks, overruns, output_fails;
    float    cycle_us_avg, cycle_us_max, jitter_us_max;
    float    decision_us_last, decision_us_max;
    float    out_latency_us_last, out_latency_us_max;
    float    inject_ms_last, inject_ms_min, inject_ms_max;
    uint32_t inject_count, last_inject_fault;
    uint32_t uptime_ms;
    float    nav_speed, nav_steer;  /* last command asked by navigation (before the check) */
} av_status_t;

/* CLI -> nav_task */
enum { NAV_CTRL_START = 1, NAV_CTRL_STOP, NAV_CTRL_FAULT, NAV_CTRL_CLEAR };
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_NAV_CTRL */
    uint32_t  cmd;
    uint32_t  fault;
} av_nav_ctrl_t;

/* CLI -> vehicle_simulator */
enum { SIM_CTRL_FAULT = 1, SIM_CTRL_CLEAR, SIM_CTRL_OBSTACLE, SIM_CTRL_OBSTACLE_OFF,
       SIM_CTRL_RESET, SIM_CTRL_ESTOP, SIM_CTRL_STATUS };
typedef struct {
    av_hdr_t hdr;              /* AV_MSG_SIM_CTRL */
    uint32_t  cmd;
    uint32_t  fault;
    float     value;
    int32_t   flag;
} av_sim_ctrl_t;

typedef struct {
    float    x, y, v, dist, obs_x;
    uint32_t faults;
    uint8_t  collided, island_tripped, estop, obs_present;
    uint8_t  obs_known, motor_on, pad[2];
} av_sim_status_t;

/* ------------------------------------------------------------------ */
/* Event record (supervisor -> logger through the shared-memory ring)  */
/* ------------------------------------------------------------------ */
enum {
    EV_CYCLE = 1,      /* periodic snapshot (every tick)                     */
    EV_VERDICT,        /* a CLAMP or REJECT                                  */
    EV_STATE,          /* state transition                                   */
    EV_COND,           /* conditions set (x1 = 1) or cleared (x1 = 0), aux    */
    EV_INJECT,         /* operator injected a fault, aux = fault id          */
    EV_REACTION,       /* first reaction after an injection, x1 = ms         */
    EV_CLIENT,         /* client connect (x1 = 1) / disconnect (x1 = 0)      */
    EV_INFO,
    EV_OVERRUN,        /* supervisor cycle exceeded its budget, x1 = us      */
};

typedef struct {
    uint64_t t_us;
    uint32_t seq;
    uint16_t kind;
    uint8_t  state, prev_state;
    uint8_t  verdict, authority, pad[2];
    uint32_t reasons;
    uint32_t conds;
    uint32_t aux;
    float    v, dist, gyro;
    float    cmd_speed, cmd_steer, out_speed, out_steer;
    float    x1, x2;
} av_event_t;

#endif /* MESSAGES_H */

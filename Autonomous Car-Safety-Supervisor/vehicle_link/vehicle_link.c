/*
 * vehicle_link.c - the real rover on the Raspberry Pi: the ONLY program that touches the motors
 * and the vehicle sensors. It registers as "av_vehicle_link" and speaks exactly the messages of
 * vehicle_simulator, so the supervisor, sensor monitor and navigation do not change.
 *
 *  main thread  (prio 48, fixed-priority channel)
 *     VEHICLE_CMD  from the supervisor - only the SUPERVISOR role may drive
 *     SIM_CTRL     from the CLI - status, link-drop test, clear
 *     GPIO pulses  wheel encoder slot edges (registered QNX events from the GPIO manager)
 *     10 ms timer  motor watchdog, wheel speeds, L298N motors, motor relay, E-stop / ACK
 *                  buttons, RAW_SENSORS -> sensor_monitor
 *  lidar thread (prio 47)  reads TF-Luna frames from the UART, keeps the latest distance
 *
 * Motor watchdog (replaces a separate safety chip): no valid supervisor command for 30 ms ->
 * motor PWM off, relay open; re-armed after 10 good commands in a row.
 * E-stop button (normally closed): pressed or broken wire -> relay opens at once, here, and the
 * supervisor is told (ESTOP). Stopping or killing this program also stops the motors
 * (SIGTERM handler; on SIGKILL the supervisor sees the link vanish and opens the relay).
 *
 * usage: vehicle_link [-s rover] [-l /dev/serN] [-v]      (or environment AV_LIDAR_DEV)
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "qnx_helpers.h"
#include "access_control.h"
#include "hw_protocols.h"
#include "settings.h"
#include "vehicle_physics.h"
#include "pi_gpio.h"

enum { WD_ARMING = 0, WD_ARMED, WD_TRIPPED };

static const av_profile_t *P;
static access_roles_t g_roles;
static int g_verbose;
static const char *g_lidar_dev = "/dev/ser1";
static volatile sig_atomic_t g_quit;

/* last command from the supervisor */
static av_vehicle_cmd_t g_cmd;
static uint64_t g_cmd_rx_us;
static int g_cmd_seen, g_rearm, g_wd = WD_ARMING;

/* sensors */
static enc_est_t g_enc_l, g_enc_r;
static int g_dir_l = 1, g_dir_r = 1;          /* sign of the last commanded wheel direction */
static uint32_t g_seq, g_last_lidar_frames;
static int g_link_drop;                        /* CLI test: stop sending sensor frames */

/* outputs (cached: a GPIO message is only sent when something changes) */
static int g_relay = -1;
static int g_pin_state[32];
static unsigned g_duty_l = ~0u, g_duty_r = ~0u;

/* latest LiDAR frame, written by the lidar thread */
static struct {
    pthread_mutex_t mtx;
    float    dist_m;
    uint16_t amp;
    uint64_t t_us;
    uint32_t frames, bad;
    int      open;
} L = { .mtx = PTHREAD_MUTEX_INITIALIZER };

static void on_signal(int s)
{
    (void)s;
    g_quit = 1;
}

/* ------------------------------------------------------------------ */
/* LiDAR thread                                                        */
/* ------------------------------------------------------------------ */

static int open_serial(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd == -1)
        return -1;
    struct termios t;
    if (tcgetattr(fd, &t) == 0) {
        t.c_iflag &= ~(tcflag_t)(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
        t.c_oflag &= ~(tcflag_t)OPOST;
        t.c_lflag &= ~(tcflag_t)(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        t.c_cflag &= ~(tcflag_t)(CSIZE | PARENB | CSTOPB);
        t.c_cflag |= CS8 | CREAD | CLOCAL;
        t.c_cc[VMIN] = 1;
        t.c_cc[VTIME] = 0;
        cfsetispeed(&t, AV_HW_LIDAR_BAUD);
        cfsetospeed(&t, AV_HW_LIDAR_BAUD);
        tcsetattr(fd, TCSANOW, &t);
        tcflush(fd, TCIFLUSH);
    }
    return fd;
}

static void *lidar_thread(void *arg)
{
    (void)arg;
    qnx_set_priority(AV_PRIO_LINK - 1);
    int warned = 0;
    for (;;) {
        int fd = open_serial(g_lidar_dev);
        if (fd == -1) {
            if (!warned++)
                printf("[VEHICLE] LiDAR: cannot open %s (%s) - retrying every second\n",
                       g_lidar_dev, strerror(errno));
            sleep(1);
            continue;
        }
        printf("[VEHICLE] LiDAR: TF-Luna on %s at %d baud\n", g_lidar_dev, AV_HW_LIDAR_BAUD);
        fflush(stdout);
        pthread_mutex_lock(&L.mtx);
        L.open = 1;
        pthread_mutex_unlock(&L.mtx);
        warned = 0;

        tfluna_parser_t p;
        tfluna_init(&p);
        uint8_t buf[64];
        for (;;) {
            ssize_t n = read(fd, buf, sizeof buf);
            if (n == -1 && errno == EINTR)
                continue;
            if (n <= 0)
                break;
            for (ssize_t i = 0; i < n; i++) {
                tfluna_frame_t f;
                if (!tfluna_feed(&p, buf[i], &f))
                    continue;
                pthread_mutex_lock(&L.mtx);
                L.dist_m = f.dist_m;
                L.amp = f.amp;
                L.t_us = qnx_now_us();
                L.frames++;
                L.bad = p.bad_checksum;
                pthread_mutex_unlock(&L.mtx);
            }
        }
        printf("[VEHICLE] LiDAR: read error on %s (%s) - reopening\n", g_lidar_dev, strerror(errno));
        pthread_mutex_lock(&L.mtx);
        L.open = 0;
        pthread_mutex_unlock(&L.mtx);
        close(fd);
        sleep(1);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* outputs                                                             */
/* ------------------------------------------------------------------ */

static void pin(int p, int v)
{
    if (g_pin_state[p] == v)
        return;
    g_pin_state[p] = v;
    pig_write(p, v);
}

static void set_relay(int on)
{
    /* rewritten every cycle: if the supervisor pulled the relay low while we were tripped,
     * the correct state is restored within 10 ms once we drive again */
    pig_write(AV_GPIO_MOTOR_RELAY, on);
    if (on != g_relay && g_verbose)
        printf("[VEHICLE] motor relay %s\n", on ? "CLOSED (motors powered)" : "OPEN (motor power cut)");
    g_relay = on;
}

enum { W_COAST = 0, W_FWD, W_REV, W_BRAKE };

/* one L298N channel: mode + duty 0..1 */
static void wheel(int left, int mode, float duty)
{
    int fwd = left ? AV_GPIO_L_FWD : AV_GPIO_R_FWD;
    int rev = left ? AV_GPIO_L_REV : AV_GPIO_R_REV;
    int pwm = left ? AV_GPIO_PWM_LEFT : AV_GPIO_PWM_RIGHT;
    unsigned *cache = left ? &g_duty_l : &g_duty_r;

    switch (mode) {
    case W_FWD:   pin(fwd, 1); pin(rev, 0); break;
    case W_REV:   pin(fwd, 0); pin(rev, 1); break;
    case W_BRAKE: pin(fwd, 1); pin(rev, 1); duty = 1.0f; break;  /* L298N fast motor stop */
    default:      pin(fwd, 0); pin(rev, 0); duty = 0.0f; break;  /* coast */
    }
    if (duty < 0.0f)
        duty = 0.0f;
    if (duty > 1.0f)
        duty = 1.0f;
    unsigned v = (unsigned)(duty * AV_HW_PWM_RANGE + 0.5f);
    if (v != *cache) {
        *cache = v;
        pig_pwm_duty(pwm, v);
    }
}

/* speed controller of one wheel: feed-forward + proportional correction from the encoder */
static void drive_wheel(int left, float target, float measured)
{
    float t = fabsf(target);
    if (t < P->standstill_v) {
        wheel(left, W_BRAKE, 1.0f);
        return;
    }
    float duty = t / AV_HW_MOTOR_V_FULL + AV_HW_SPEED_KP * (t - fabsf(measured));
    if (duty < AV_HW_MOTOR_MIN_DUTY)
        duty = AV_HW_MOTOR_MIN_DUTY;
    if (left)
        g_dir_l = target >= 0.0f ? 1 : -1;
    else
        g_dir_r = target >= 0.0f ? 1 : -1;
    wheel(left, target >= 0.0f ? W_FWD : W_REV, duty);
}

static void motors_off(void)
{
    wheel(1, W_COAST, 0.0f);
    wheel(0, W_COAST, 0.0f);
    set_relay(0);
}

/* ------------------------------------------------------------------ */
/* 10 ms cycle                                                         */
/* ------------------------------------------------------------------ */

static int connect_monitor(void)
{
    int coid = name_open(AV_NAME_SENSOR, 0);
    if (coid == -1)
        return -1;
    if (qnx_hello(coid, AV_ROLE_SENSOR, "vehicle_link") != 0) {
        name_close(coid);
        return -1;
    }
    printf("[VEHICLE] connected to %s (raw sensor readings)\n", AV_NAME_SENSOR);
    return coid;
}

static void tick(uint64_t now)
{
    static int mon = -1;
    static uint64_t next_try, last_print;
    if (mon == -1 && now >= next_try) {
        mon = connect_monitor();
        next_try = now + 1000000u;
    }

    /* motor watchdog: valid supervisor commands must keep coming */
    int fresh = g_cmd_seen && now - g_cmd_rx_us <= (uint64_t)AV_ISLAND_TIMEOUT_MS * 1000u;
    if (g_wd == WD_ARMED && g_cmd_seen && !fresh) {
        g_wd = WD_TRIPPED;
        g_rearm = 0;
        printf("[VEHICLE] MOTOR WATCHDOG: no supervisor command for %d ms -> motors stopped, relay open\n",
               AV_ISLAND_TIMEOUT_MS);
    }
    if (!fresh)
        g_rearm = 0;
    if (g_wd != WD_ARMED && g_rearm >= AV_ISLAND_REARM_FRAMES) {
        if (g_wd == WD_TRIPPED)
            printf("[VEHICLE] motor watchdog re-armed after %d valid commands\n", AV_ISLAND_REARM_FRAMES);
        g_wd = WD_ARMED;
    }

    /* buttons: E-stop is normally closed to GND, so 1 (or a read error) means STOP */
    int estop = pig_read(AV_GPIO_ESTOP_IN) != 0;
    int ack = pig_read(AV_GPIO_ACK_IN) == 0;

    /* wheel speeds from the encoders (sign = commanded direction, single-channel encoder) */
    float m_pp = 3.14159265f * AV_HW_WHEEL_DIAMETER_M / AV_HW_ENCODER_SLOTS;
    float vl = g_dir_l * enc_speed(&g_enc_l, now, m_pp, AV_HW_ENC_WINDOW_US, 2.0f * P->a_max);
    float vr = g_dir_r * enc_speed(&g_enc_r, now, m_pp, AV_HW_ENC_WINDOW_US, 2.0f * P->a_max);

    /* motors */
    int powered = g_wd == WD_ARMED && fresh && g_cmd.motor_enable && !estop;
    if (!powered) {
        motors_off();
    } else {
        set_relay(1);
        float tl, tr;
        physics_steer_to_wheels(g_cmd.speed_sp, g_cmd.steer_deg, P->wheelbase_m, P->track_m, &tl, &tr);
        if (g_cmd.speed_sp < P->standstill_v && g_cmd.brake_mode != BRAKE_NORMAL)
            tl = tr = 0.0f;
        drive_wheel(1, tl, vl);
        drive_wheel(0, tr, vr);
    }

    /* raw sensor frame for the sensor monitor */
    av_raw_sensors_t m;
    memset(&m, 0, sizeof m);
    m.hdr.type = AV_MSG_RAW_SENSORS;
    m.hdr.version = AV_PROTO_VERSION;
    m.seq = ++g_seq;
    m.t_us = now;
    m.v_left = vl;
    m.v_right = vr;
    m.fresh = (uint8_t)((1u << SENS_ENC_L) | (1u << SENS_ENC_R));   /* IMU: sensor monitor (MPU6050) */

    pthread_mutex_lock(&L.mtx);
    float dist = L.dist_m;
    uint16_t amp = L.amp;
    uint32_t frames = L.frames;
    pthread_mutex_unlock(&L.mtx);
    if (frames != g_last_lidar_frames) {
        g_last_lidar_frames = frames;
        if (amp != 0xFFFF) {                       /* 0xFFFF = overexposed (sunlight): no sample */
            m.fresh |= 1u << SENS_US;
            /* weak signal or beyond range: nothing reliable in front, like an ultrasonic
             * without echo */
            m.us_no_echo = amp < AV_HW_LIDAR_MIN_AMP || dist >= P->us_max_m;
            m.distance_m = m.us_no_echo ? P->us_max_m : dist;
        }
    }
    m.island_state = g_wd == WD_TRIPPED;
    m.estop = (uint8_t)estop;
    m.relay_closed = (uint8_t)powered;
    m.ack_button = (uint8_t)ack;

    if (!g_link_drop && mon != -1 &&
        qnx_send_timeout(mon, &m, sizeof m, NULL, 0, 5000) == -1 &&
        (errno == EBADF || errno == ESRCH || errno == ENOTCONN || errno == EPERM)) {
        name_close(mon);
        mon = -1;
    }

    if (g_verbose && now - last_print > 1000000u) {
        last_print = now;
        printf("[VEHICLE] wheels L %.2f R %.2f m/s | lidar %.2f m (amp %u, %u frames) | relay %s | "
               "E-stop %s | watchdog %s | cmd %.2f m/s %.1f deg\n",
               vl, vr, dist, amp, frames, powered ? "ON" : "off", estop ? "PRESSED" : "ok",
               g_wd == WD_ARMED ? "armed" : g_wd == WD_TRIPPED ? "TRIPPED" : "arming",
               g_cmd.speed_sp, g_cmd.steer_deg);
        fflush(stdout);
    }
}

/* ------------------------------------------------------------------ */
/* server                                                              */
/* ------------------------------------------------------------------ */

typedef union {
    uint16_t          type;
    struct _pulse     pulse;
    av_hdr_t         hdr;
    av_hello_t       hello;
    av_vehicle_cmd_t cmd;
    av_sim_ctrl_t    ctrl;
} link_msg_t;

static void handle_ctrl(int rcvid, const av_sim_ctrl_t *c)
{
    switch (c->cmd) {
    case SIM_CTRL_STATUS:
        break;
    case SIM_CTRL_FAULT:
        if (c->fault != FAULT_LINK_DROP) {
            MsgError(rcvid, ENOTSUP);             /* only real, physical faults on the rover */
            return;
        }
        g_link_drop = 1;
        printf("[VEHICLE] TEST: sensor frames stopped (link-drop)\n");
        break;
    case SIM_CTRL_CLEAR:
        g_link_drop = 0;
        break;
    default:
        MsgError(rcvid, ENOTSUP);
        return;
    }
    av_sim_status_t st;
    memset(&st, 0, sizeof st);
    pthread_mutex_lock(&L.mtx);
    st.dist = L.amp >= AV_HW_LIDAR_MIN_AMP && L.amp != 0xFFFF ? L.dist_m : -1.0f;
    pthread_mutex_unlock(&L.mtx);
    st.v = 0.5f * (g_enc_l.v * g_dir_l + g_enc_r.v * g_dir_r);
    st.faults = g_link_drop ? 1u << FAULT_LINK_DROP : 0;
    st.island_tripped = g_wd == WD_TRIPPED;
    st.estop = pig_read(AV_GPIO_ESTOP_IN) != 0;
    st.motor_on = g_relay == 1;
    MsgReply(rcvid, EOK, &st, sizeof st);
}

static int hw_init(int self_coid)
{
    if (pig_open() == -1) {
        printf("[VEHICLE] FATAL: /dev/gpio/msg not available (%s)\n", strerror(errno));
        return -1;
    }
    memset(g_pin_state, -1, sizeof g_pin_state);
    int rc = 0;
    rc |= pig_output(AV_GPIO_MOTOR_RELAY, 0);          /* motor power off first */
    rc |= pig_output(AV_GPIO_L_FWD, 0);
    rc |= pig_output(AV_GPIO_L_REV, 0);
    rc |= pig_output(AV_GPIO_R_FWD, 0);
    rc |= pig_output(AV_GPIO_R_REV, 0);
    g_pin_state[AV_GPIO_L_FWD] = g_pin_state[AV_GPIO_L_REV] = 0;
    g_pin_state[AV_GPIO_R_FWD] = g_pin_state[AV_GPIO_R_REV] = 0;
    g_relay = 0;
    if (pig_pwm_setup(AV_GPIO_PWM_LEFT, AV_HW_PWM_HZ, AV_HW_PWM_RANGE) == -1 ||
        pig_pwm_setup(AV_GPIO_PWM_RIGHT, AV_HW_PWM_HZ, AV_HW_PWM_RANGE) == -1) {
        printf("[VEHICLE] FATAL: PWM setup on GPIO%d/%d failed (%s)\n", AV_GPIO_PWM_LEFT,
               AV_GPIO_PWM_RIGHT, strerror(errno));
        return -1;
    }
    g_duty_l = g_duty_r = 0;
    rc |= pig_input_pullup(AV_GPIO_ESTOP_IN);
    rc |= pig_input_pullup(AV_GPIO_ACK_IN);
    rc |= pig_input_pullup(AV_GPIO_ENC_LEFT);
    rc |= pig_input_pullup(AV_GPIO_ENC_RIGHT);
    if (rc) {
        printf("[VEHICLE] FATAL: GPIO setup failed (%s)\n", strerror(errno));
        return -1;
    }
    if (pig_rising_edge_pulse(AV_GPIO_ENC_LEFT, self_coid, AV_PRIO_LINK, AV_PULSE_GPIO_EDGE) == -1 ||
        pig_rising_edge_pulse(AV_GPIO_ENC_RIGHT, self_coid, AV_PRIO_LINK, AV_PULSE_GPIO_EDGE) == -1) {
        printf("[VEHICLE] FATAL: encoder edge events failed (%s)\n", strerror(errno));
        return -1;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    const char *profile = "rover";
    const char *env = getenv("AV_LIDAR_DEV");
    if (env && *env)
        g_lidar_dev = env;
    int opt;
    while ((opt = getopt(argc, argv, "s:l:vh")) != -1) {
        switch (opt) {
        case 's': profile = optarg; break;
        case 'l': g_lidar_dev = optarg; break;
        case 'v': g_verbose = 1; break;
        default:
            fprintf(stderr, "usage: %s [-s rover] [-l /dev/serN] [-v]\n", argv[0]);
            return 1;
        }
    }
    P = av_profile_get(profile);
    enc_init(&g_enc_l);
    enc_init(&g_enc_r);

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    qnx_set_priority(AV_PRIO_LINK);
    qnx_lock_memory();
    access_init(&g_roles);

    qnx_server_t srv;
    if (qnx_server_create(&srv, AV_NAME_LINK, 1) == -1)
        return 1;
    int self = ConnectAttach(ND_LOCAL_NODE, 0, srv.chid, _NTO_SIDE_CHANNEL, 0);
    if (self == -1 || hw_init(self) == -1) {
        motors_off();
        return 1;
    }
    pthread_t th;
    pthread_create(&th, NULL, lidar_thread, NULL);
    qnx_pulse_timer(srv.chid, AV_PRIO_LINK, AV_PULSE_TICK, AV_TICK_MS * 1000u, NULL);
    printf("[VEHICLE] real rover ready: L298N on GPIO%d/%d, encoders GPIO%d/%d, E-stop GPIO%d, "
           "ACK GPIO%d, relay GPIO%d, LiDAR %s  (profile %s, prio %d)\n",
           AV_GPIO_PWM_LEFT, AV_GPIO_PWM_RIGHT, AV_GPIO_ENC_LEFT, AV_GPIO_ENC_RIGHT,
           AV_GPIO_ESTOP_IN, AV_GPIO_ACK_IN, AV_GPIO_MOTOR_RELAY, g_lidar_dev, P->name, AV_PRIO_LINK);
    fflush(stdout);

    while (!g_quit) {
        link_msg_t m;
        struct _msg_info info;
        int rcvid = MsgReceive(srv.chid, &m, sizeof m, &info);
        if (rcvid == -1)
            continue;                             /* EINTR from a signal: loop checks g_quit */
        uint64_t now = qnx_now_us();
        if (rcvid == 0) {
            switch (m.pulse.code) {
            case AV_PULSE_TICK:
                tick(now);
                break;
            case AV_PULSE_GPIO_EDGE:
                if (m.pulse.value.sival_int == AV_GPIO_ENC_LEFT)
                    enc_pulse(&g_enc_l, now, AV_HW_ENC_MIN_GAP_US);
                else if (m.pulse.value.sival_int == AV_GPIO_ENC_RIGHT)
                    enc_pulse(&g_enc_r, now, AV_HW_ENC_MIN_GAP_US);
                break;
            case _PULSE_CODE_DISCONNECT:
                access_drop(&g_roles, m.pulse.scoid, NULL, NULL, 0);
                ConnectDetach(m.pulse.scoid);
                break;
            default:
                break;
            }
            continue;
        }
        if (qnx_handle_system_msg(rcvid, &m))
            continue;

        switch (m.hdr.type) {
        case AV_MSG_HELLO: {
            m.hello.name[sizeof m.hello.name - 1] = '\0';
            int err = access_hello(&g_roles, info.scoid, info.pid, m.hello.roles, m.hello.name);
            if (err)
                MsgError(rcvid, err);
            else
                MsgReply(rcvid, EOK, NULL, 0);
            break;
        }
        case AV_MSG_VEHICLE_CMD:
            if (!access_has_role(&g_roles, info.scoid, AV_ROLE_SUPERVISOR)) {
                MsgError(rcvid, EPERM);                /* only the supervisor may drive */
                break;
            }
            if (info.srcmsglen < (int)sizeof m.cmd) {
                MsgError(rcvid, EBADMSG);
                break;
            }
            g_cmd = m.cmd;
            g_cmd_rx_us = now;
            g_cmd_seen = 1;
            if (g_wd != WD_ARMED)
                g_rearm++;
            MsgReply(rcvid, EOK, NULL, 0);
            break;
        case AV_MSG_SIM_CTRL:
            handle_ctrl(rcvid, &m.ctrl);
            break;
        default:
            MsgError(rcvid, ENOSYS);
            break;
        }
    }
    motors_off();
    printf("[VEHICLE] stopped: motors off, relay open\n");
    return 0;
}
